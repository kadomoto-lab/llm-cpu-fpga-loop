# Super Bacon (2-wide in-order)

| Item | Value |
|---|---|
| CPU parameters | `FAST=2`, `CLOCK_HZ=100000000` |
| Clock | 100.000000 MHz |
| CoreMark iterations | 6,000 |
| Measured ticks | 1,808,777,942 |
| CoreMark | 331.715677 iterations/s |
| CoreMark/MHz | 3.317157 |
| Benchmark-interval RTL IPC | 0.946456 |
| Setup slack | +0.058 ns |
| Hold slack | +0.077 ns |

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
