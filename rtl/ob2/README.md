# Out-of-Bacon II (3-wide OoO)

| Item | Value |
|---|---|
| CPU parameters | `FAST=4`, `CLOCK_HZ=100000000` |
| Clock | 100.000000 MHz |
| CoreMark iterations | 6,000 |
| Measured ticks | 1,102,335,136 |
| CoreMark | 544.299080 iterations/s |
| CoreMark/MHz | 5.442991 |
| Benchmark-interval RTL IPC | 1.553003 |
| Setup slack | +0.135 ns |
| Hold slack | +0.033 ns |

## CoreMark compilation

Compiler: **GCC 13.2.0**.
The following flags were reported in the board UART output for the measured run:

```text
-O3 -march=rv32im -mabi=ilp32
-funroll-loops --param=max-unrolled-insns=80 --param=max-average-unrolled-insns=40
```

## Files

- [Core implementation](ob2_cpu.vh)
- [CPU top module](cpu.v)
