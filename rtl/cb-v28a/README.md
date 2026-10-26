# Crispy Bacon A (case-study baseline)

| Item | Value |
|---|---|
| CPU parameters | `FAST=5`, `CLOCK_HZ=130136986` |
| Clock | 130.136986 MHz |
| CoreMark iterations | 8,000 |
| Measured ticks | 1,475,625,972 |
| CoreMark | 705.528305 iterations/s |
| CoreMark/MHz | 5.421428 |
| Benchmark-interval RTL IPC | 1.546850 |
| Setup slack | +0.005 ns |
| Hold slack | +0.024 ns |

## CoreMark compilation

Compiler: **GCC 13.2.0**.
The following flags were reported in the board UART output for the measured run:

```text
-O3 -march=rv32im -mabi=ilp32
-funroll-loops --param=max-unrolled-insns=80 --param=max-average-unrolled-insns=40
```

## Files

- [Core implementation](cb_cpu.vh)
- [CPU top module](cpu.v)
