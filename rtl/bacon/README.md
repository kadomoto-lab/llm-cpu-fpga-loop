# Bacon (1-wide)

| Item | Value |
|---|---|
| CPU parameters | `FAST=1`, `CLOCK_HZ=100000000` |
| Clock | 100.000000 MHz |
| CoreMark iterations | 6,000 |
| Measured ticks | 2,087,883,486 |
| CoreMark | 287.372358 iterations/s |
| CoreMark/MHz | 2.873724 |
| Setup slack | +0.230 ns |
| Hold slack | +0.110 ns |

## CoreMark compilation

Compiler: **GCC 13.2.0**.
The following flags were reported in the board UART output for the measured run:

```text
-O3 -march=rv32im -mabi=ilp32
-funroll-loops --param=max-unrolled-insns=80 --param=max-average-unrolled-insns=40
```

## Files

- [Core implementation](fast_cpu.vh)
- [CPU top module](cpu.v)
