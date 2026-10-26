# Out-of-Bacon (2-wide renamed)

| Item | Value |
|---|---|
| CPU parameters | `FAST=3`, `CLOCK_HZ=25000000` |
| Clock | 25.000000 MHz |
| CoreMark iterations | 2,000 |
| Measured ticks | 523,692,352 |
| CoreMark | 95.475903 iterations/s |
| CoreMark/MHz | 3.819036 |

## CoreMark compilation

Compiler: **GCC 13.2.0**.
The following flags were reported in the board UART output for the measured run:

```text
-O3 -march=rv32im -mabi=ilp32
-funroll-loops --param=max-unrolled-insns=80 --param=max-average-unrolled-insns=40
```

## Files

- [Core implementation](out_cpu.vh)
- [CPU top module](cpu.v)
