# Designing Field-Oriented Control on FPGA: An Architecture Study (Part 1)

*Part 1 of 2 — this post covers the theory and the planned architecture. Part 2 will cover the VHDL implementation, ModelSim validation, and results.*

## Why I'm Building This

I've spent the last few months building real-time control systems on FPGA: a fixed-point PID controller with an SPI ADC interface and anti-windup protection, and more recently a VHDL-based sensor hub streaming BME280 data over SPI, UART, and PWM — both on a Basys3 (Artix-7) board. Both projects left me with the same question: what happens when a control loop needs to be fast and predictable enough that an MCU's interrupt jitter becomes the actual bottleneck, not the algorithm?

Field-Oriented Control (FOC) for BLDC/PMSM motors is a good test of that question. The math is well-established — Clarke and Park transforms, a couple of PI loops, space-vector PWM — but it has to run with tight, deterministic timing at tens or hundreds of kilohertz. That combination of "well-understood algorithm" and "hard real-time constraint" is exactly where an FPGA implementation earns its extra complexity over a microcontroller.

This is the first of a two-part series. Here I'm laying out the theory and the architecture I'm planning to build in VHDL. Part 2 will cover the actual RTL, the ModelSim testbench, and whatever forces a redesign along the way — something usually does.

## Why FPGA Instead of a Faster MCU

The case for moving motor control onto an FPGA is determinism, not raw clock speed. A microcontroller executes instructions sequentially, and even a fast core has to contend with interrupt latency, cache effects, and RTOS scheduling jitter. For a current loop running at tens of kilohertz, that jitter eats directly into the timing margin available for the FOC math itself.

An FPGA sidesteps this by running each pipeline stage — ADC sampling, the Clarke/Park transforms, the PI loops, SVPWM generation — as concurrent hardware rather than sequential instructions. The latency from a current sample to a PWM update becomes a fixed number of clock cycles instead of a statistical distribution. That predictability, more than sheer speed, is the real argument for FPGA-based motor control.

## FOC in Three Transforms

FOC's core trick is to stop treating the problem as three time-varying phase currents and instead work in a two-axis frame that rotates with the rotor:

- **Clarke transform** — converts the three-phase currents (i_a, i_b, i_c) into two stationary orthogonal components, α and β. Since the phases sum to zero in a balanced system, this is really a change of basis: three redundant signals become two independent ones.
- **Park transform** — rotates the α/β components by the rotor's electrical angle θ, producing d (aligned with rotor flux) and q (producing torque) components. In this rotating frame, steady-state currents become DC values instead of sinusoids, which is what makes ordinary PI control possible.
- **Inverse Park / inverse Clarke** — after the PI loops compute voltage commands in the d/q frame, these transforms convert back to three-phase voltage references, which SVPWM turns into switching signals for the inverter.

```
ADC (phase currents)
        |
        v
Clarke Transform (abc -> alpha/beta)
        |
        v
Park Transform (alpha/beta -> dq, using theta)
        |
   +----+----+
   v         v
 PI (d)    PI (q)
   |         |
   +----+----+
        v
Inverse Park (dq -> alpha/beta)
        |
        v
Inverse Clarke (alpha/beta -> abc)
        |
        v
      SVPWM
        |
        v
 Gate Drive Signals
```

The elegance of FOC is that a genuinely AC problem — three sinusoidal currents, a rotating field — collapses into two DC regulation problems. The cost is that you need θ, the rotor's electrical angle, continuously and accurately, which is its own design decision depending on whether you're using Hall sensors, an encoder, or a sensorless observer.

## What I'm Planning to Build

For this first version, I'm scoping to a Hall-sensor-based design rather than sensorless estimation. It's a smaller problem to get right, and it isolates the FOC math from the added complexity of a flux or sliding-mode observer — sensorless estimation is a reasonable extension once the core loop is validated.

Planned module breakdown:

- **ADC interface** — phase current sampling, likely over SPI, reusing patterns from the PID controller project
- **Clarke/Park transform block** — fixed-point, probably a Q1.15-style format to balance dynamic range against the resolution the current loop needs
- **sin/cos generation for Park** — the one architectural choice I haven't locked in yet. CORDIC avoids multipliers entirely and scales cleanly with bit width, at the cost of multiple cycles per calculation; a LUT is faster but its memory footprint grows quickly with angular resolution. I'll implement both in simulation and choose based on actual slice/BRAM usage on the Artix-7, rather than guessing upfront
- **PI controllers** for the d and q current loops, in incremental (velocity) form to sidestep integrator windup
- **SVPWM generation** driving the three half-bridge PWM outputs

## Validation Plan

I don't have BLDC/PMSM hardware on the bench yet, so the first validation pass will be against a mathematical motor model rather than a physical one — a discretized PMSM plant running as a ModelSim testbench, checked against a floating-point reference in Python or MATLAB before worrying about anything physical. Those waveform comparisons will be the main evidence in Part 2.

Timing closure in Vivado is the other open question. The Clarke/Park/SVPWM chain has enough multiply operations that I expect to need pipelining to close timing at the target clock frequency — I won't know exactly how much until it's actually placed and routed.

## What's Next

I'm tracking the implementation on GitHub as it happens. Part 2 will cover the VHDL, the testbench, and the results — including whatever didn't work the first time.

If you've built FOC on FPGA, or talked yourself out of it partway through, I'd be glad to hear what tripped you up.
