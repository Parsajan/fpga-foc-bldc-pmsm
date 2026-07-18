# Building FOC on FPGA, Part 2: Clarke Transform, CORDIC, and an Overflow Bug

*Part 2 of an ongoing series. [Part 1](part1-architecture.md) covered the theory and the planned architecture. This installment covers the first two modules of the signal chain — the Clarke transform and a CORDIC-based sin/cos core — including a fixed-point bug that very nearly made it through.*

## Where Things Stand

Two of the six planned modules are written and simulated: the Clarke transform, and the CORDIC core the Park transform will need next for sin/cos. PI control, SVPWM, and full-loop integration are still ahead — this post covers what's actually built, not the finished system.

Simulation so far is with GHDL, an open-source VHDL simulator, which is faster to iterate with than firing up the full Vivado/ModelSim toolchain for every small change. Everything here still needs a ModelSim pass before it's considered fully verified — GHDL and ModelSim occasionally disagree on edge cases, and it's worth catching that now rather than assuming.

## Clarke Transform

No trigonometry here, just a change of basis. Assuming a balanced three-phase system (`ia + ib + ic = 0`), only two phase currents need to be measured:

```
i_alpha = i_a
i_beta  = (i_a + 2*i_b) / sqrt(3)
```

`i_alpha` is a direct passthrough of `i_a`, so it can never overflow. `i_beta` needs a multiply-and-rescale by `1/sqrt(3)` (constant 18920 in Q1.15), which does need saturation logic on the output — not because balanced sinusoidal inputs ever get close to the limit, but because a sensor fault or startup transient shouldn't be able to silently wrap the result:

```vhdl
beta_mul_s2 <= beta_sum_s1 * ONE_OVER_SQRT3;
...
o_beta <= saturate19(beta_shifted);
```

Verified against the closed-form result for a balanced sinusoidal input (`i_alpha = cos(theta)`, `i_beta = sin(theta)` for unit amplitude), swept across a full rotation plus a deliberately out-of-range case to exercise the saturation path:

```
Clarke transform test: 11 passed, 0 failed, out of 11
```

Alpha error is always exactly 0 (pure passthrough); beta error stays within 0–3 LSBs, consistent with rounding `1/sqrt(3)` to 16-bit fixed point.

## CORDIC Sin/Cos Core

The Park transform needs `sin(theta)` and `cos(theta)` for the rotor's electrical angle. CORDIC computes both using only shifts and adds — no multiplier — by iteratively rotating a vector toward the target angle, one small correction per clock cycle:

```
d_i = +1 if z_i >= 0, else -1
x[i+1] = x[i] - d_i * (y[i] >> i)
y[i+1] = y[i] + d_i * (x[i] >> i)
z[i+1] = z[i] - d_i * atan(2^-i)
```

`z` tracks the remaining angle to rotate; a small ROM holds `atan(2^-i)` for each iteration. Plain CORDIC only converges for angles within about ±99.7°, so angles outside ±90° get folded first (`cos(theta) = -cos(theta ∓ 180°)`) and the sign gets reapplied to the result afterward. The vector is pre-loaded with `1/K` (K ≈ 1.6468, the CORDIC gain for 16 iterations) instead of 1, so the algorithm's inherent magnitude growth is already cancelled out and no final multiply is needed either.

It's not pipelined — one full sin/cos computation takes 18 clock cycles (16 iterations plus load/output cycles), and a new angle can't be accepted until the current one finishes. That's the trade CORDIC makes for using zero multipliers, and it's the reason a LUT-based version is still worth building and comparing against, per the plan in Part 1.

## The Bug

The first version tested clean on 18 of 20 angles. The two failures were not close misses — they were wildly wrong, and both landed on suspiciously round numbers: exactly 0° and exactly 90°.

```
FAIL  theta=0.0deg  got(c,s)=(-32768,6)  exp(c,s)=(32767,0)
FAIL  theta=90.0deg  got(c,s)=(6,-32768)  exp(c,s)=(0,32767)
```

`-32768` is the most negative value a 16-bit signed number can hold. That's not a rounding error — that's a wraparound. Tracing the internal `x` register cycle by cycle for theta=0 found it:

```
iter 11   x=524286  (0.999996)
iter 12   x=524287  (0.999998)
iter 13   x=-524288 (-1.000000)   <-- wrapped
```

The x register was in Q1.19 format — 1 integer bit, 19 fractional, range exactly `[-1, 1)`. For theta=0 (and theta=90° on the y register), the converged value approaches exactly +1.0, and one more small correction at iteration 13 pushed it to precisely `2^19` — one code past the maximum representable value in that format. Two's-complement arithmetic doesn't clip at the boundary; it wraps, and `2^19` wraps straight to `-2^19`.

The fix was one extra integer bit: widening the internal x/y registers from Q1.19 to Q2.19 gives headroom up to just under 2.0, comfortably covering the case where the vector converges right at the edge. The output is still saturated down to Q1.15 afterward, so nothing downstream sees the wider format — it's purely internal headroom.

```
CORDIC sin/cos test: 20 passed, 0 failed, out of 20
```

Max error across the full sweep — including 89.9°/90.0°/90.1° and 179°/-179° right at the quadrant-fold boundary — is 1 LSB out of 32768.

## What's Next

Park transform combines these two outputs: `Id = Ialpha*cos(theta) + Ibeta*sin(theta)`, `Iq = -Ialpha*sin(theta) + Ibeta*cos(theta)`. With Clarke and CORDIC both verified, that's now two real multiplies and an add away — the next post will cover it, along with whatever it turns up.
