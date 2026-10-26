# Crispy Bacon B (MW-A fix)

| Item | Value |
|---|---|
| CPU parameters | `FAST=5`, `CLOCK_HZ=130136986` |
| Clock | 130.136986 MHz |
| CoreMark iterations | 8,000 |
| Measured ticks | 1,467,352,742 |
| CoreMark | 709.506215 iterations/s |
| CoreMark/MHz | 5.451995 |
| Benchmark-interval RTL IPC | 1.555572 |
| Setup slack | +0.001 ns |
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
