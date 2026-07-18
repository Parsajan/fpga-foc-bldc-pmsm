# Building FOC on FPGA, Part 3: PI, SVPWM, and a Race Condition That Wasn't in the RTL

*Part 3 of an ongoing series. [Part 1](part1-architecture.md) covered the theory and planned architecture; [Part 2](part2-clarke-cordic.md) covered the Clarke transform and CORDIC core, including an overflow bug. This post finishes the signal chain: Park transform, PI control, SVPWM, inverse Park, and a full end-to-end integration test — plus a bug that had nothing to do with fixed-point math at all.*

## Park Transform

Park transform combines the Clarke and CORDIC outputs into the rotor-synchronous frame: `Id = Ialpha*cos + Ibeta*sin`, `Iq = -Ialpha*sin + Ibeta*cos`. Four multiplies, two adds, then rescale back to Q1.15.

That rescale is where the bug was. Multiplying two Q1.15 values gives a Q2.30 product — 30 fractional bits, not 15 — but I wrote the constant that controls how many bits to drop using the *output's* fractional width instead of the *intermediate sum's*. The shift amount came out as 0 instead of 15, which fed a slice of the wrong width into a 19-bit signal and crashed the simulator outright (`bound check failure`) rather than quietly producing wrong numbers. I'll take a hard crash over a silent wrong answer any day — this one took about two minutes to find. Fixed, it passed 11/11 unit tests plus 5/5 chained with Clarke and CORDIC.

## PI Controller

This one's architecturally different from the others: Clarke, CORDIC, and Park are feedforward (each output depends only on that cycle's input), but a PI controller's output depends on its *own previous output*. That recursive dependency means a new sample can't be pushed in every clock cycle the way the feedforward blocks can — the module has to finish updating its stored state before the next input is valid. In practice this is a non-issue: a 20 kHz current loop on a 100 MHz clock leaves about 5000 cycles between samples, against 5 cycles of latency here. But it's a real constraint worth knowing you're depending on, not just something that happens to work.

I used the incremental (velocity) form — `y(n) = y(n-1) + Kp*(e(n)-e(n-1)) + Ki*e(n)` — specifically because clamping the output automatically clamps the state used next cycle. No separate integral accumulator, no separate anti-windup logic bolted on afterward.

The RTL was correct on the first pass here. The one hiccup was in my *test*: I compared against a reference computed with the exact gain value (Ki=0.05), but the hardware necessarily uses that gain quantized to 16-bit fixed point (0.0498...), and the difference compounds every cycle in a recursive loop. Once the test's reference used the same quantized gain the hardware actually receives, it passed 10/10 — step response, saturation, and anti-windup recovery (output moves immediately off the rail when the error reverses, instead of staying pinned there).

## SVPWM

I skipped the usual sector-table implementation. Instead: inverse-Clarke to three phase voltages, find the max and min of the three, subtract their average from all three ("min-max" or zero-sequence injection), then map to duty cycle. No sector lookup, no angle needed at all — and it's a known equivalent of textbook space-vector PWM, which I cross-checked in Python before writing any VHDL.

Small bonus from that check: my first attempt at a *sector-table* reference implementation (to compare against) had its own bug in the phase-mapping table. Rather than trust one hand-derived method to catch errors in another, I used a more fundamental test instead — reconstruct Valpha/Vbeta from the resulting duty cycles via the (zero-sequence-rejecting) full Clarke transform, and check it reproduces the original command. That passed 45/45 across all six sectors and three modulation depths, with reconstruction error at floating-point precision.

One implementation detail worth keeping: converting a signed Q1.15 voltage to an unsigned duty cycle is normally "add 1, divide by 2." For a two's-complement number, that's exactly equivalent to flipping the sign bit — no adder needed.

## Inverse Park

`Valpha = Vd*cos - Vq*sin`, `Vbeta = Vd*sin + Vq*cos` — the same four products as the forward Park transform, recombined. I verified it two ways: against the closed-form rotation directly, and by feeding its output back into the already-verified forward Park transform and checking I got the original Vd/Vq back out. 7/7.

## Wiring It All Together

With all six modules built, I connected them into the full chain — raw phase currents and rotor angle in, PWM duty cycles out — and checked one control step against a fully hand-computed reference.

First run hung. Not a wrong answer — no output at all, ever. The cause wasn't in any of the RTL: it was in how I'd written the testbench's wait logic.

```vhdl
wait until pid_vout = '1';
wait until piq_vout = '1';
```

Two PI controller instances (d-axis, q-axis), same latency, triggered at the same moment — so both outputs become valid on the *exact same clock edge*. By the time the second `wait until` statement runs, `piq_vout` is already `'1'` with no further transition coming, and `wait until` only re-checks its condition when a new event happens on a signal it's watching. The fix is a single compound wait instead of two sequential ones:

```vhdl
wait until (pid_vout = '1') and (piq_vout = '1');
```

Once fixed, the full chain — Clarke → CORDIC → Park → PI(d) + PI(q) → inverse Park → SVPWM — passed all 8 end-to-end checks, from raw `(ia, ib, theta)` to final PWM duty cycles, matching a reference computed by hand outside the simulator.

## Where This Leaves Things

Tallied across the whole series: 117 individual test checks, all passing, across 6 modules plus 3 integration tests. Three real issues found and fixed along the way — a fixed-point overflow, a mis-derived constant, and a simulation race condition — none of them the kind of thing that shows up by staring at the code.

What this proves: the RTL is functionally correct for one control step, in simulation, against hand-derived references. What it doesn't prove yet: dynamic behavior against an actual motor (needs a plant model, not just a snapshot check), ModelSim agreement (everything here ran on GHDL), and whether it closes timing on real Artix-7 silicon in Vivado. Those are a different kind of work — synthesis constraints and a plant model, not more RTL — and a reasonable place to leave this series and pick up as a separate post if that work happens.

Code for all six modules and every testbench is in the [[https://github.com/Parsajan/fpga-foc-bldc-pmsm](https://github.com/Parsajan/fpga-foc-bldc-pmsm)](../README.md).
