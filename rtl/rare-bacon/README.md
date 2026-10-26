# Rare Bacon (predictor-modified core)

| Item | Value |
|---|---|
| CPU parameters | `FAST=6`, `CLOCK_HZ=80000000` |
| Clock | 80.000000 MHz |
| CoreMark iterations | 8,000 |
| Measured ticks | 1,622,954,326 |
| CoreMark | 394.342582 iterations/s |
| CoreMark/MHz | 4.929282 |
| Setup slack | +0.006 ns |
| Hold slack | +0.024 ns |

## CoreMark compilation

Compiler: **GCC 13.2.0**.
The following flags were reported in the board UART output for the measured run:

```text
-O3 -march=rv32im -mabi=ilp32
-funroll-loops --param=max-unrolled-insns=80 --param=max-average-unrolled-insns=40
```

## Files

- [Core implementation](rb_cpu.vh)
- [CPU top module](cpu.v)
