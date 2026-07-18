# FPGA-Based Field-Oriented Control for BLDC/PMSM Motors

VHDL implementation of Field-Oriented Control (FOC) on FPGA (Basys3 / Artix-7) — replacing sequential MCU execution with deterministic, concurrent hardware for the Clarke/Park transform pipeline, current-loop PI control, and SVPWM generation.

## Status: ✅ Signal Chain Complete and Simulated End-to-End (RTL + GHDL phase done)

This project was built and documented in public, as a series tracking real progress — including what broke along the way.

- [x] Theory & planned architecture — [Part 1: Architecture](docs/part1-architecture.md)
- [x] Clarke transform + CORDIC — [Part 2: Clarke + CORDIC](docs/part2-clarke-cordic.md)
- [x] Park, PI controller, SVPWM, inverse Park, full-loop integration — [Part 3: Completing the Chain](docs/part3-completing-the-chain.md)
- [x] Clarke transform (11/11 tests)
- [x] CORDIC sin/cos (20/20 tests, 1 bug found+fixed)
- [x] Park transform (11/11 unit + 5/5 chained)
- [x] PI controller, incremental form w/ clamping anti-windup (10/10 tests)
- [x] SVPWM, min-max method (45/45 tests, all 6 sectors)
- [x] Inverse Park transform (7/7 tests)
- [x] **Full-loop integration**: raw phase currents + angle → PWM duty cycles, 8/8 checks
- [x] **117/117 total test checks passing across the whole series**

## Target Hardware / Toolchain

- Board: Digilent Basys3 (Xilinx Artix-7)
- Toolchain: Vivado, ModelSim

## Related Projects

- Advanced PID Controller — Verilog, Q16.16 fixed-point, SPI ADC, anti-windup *(link)*
- FPGA Sensor Hub — VHDL, BME280 over SPI/UART/PWM *(link)*

## License

MIT
