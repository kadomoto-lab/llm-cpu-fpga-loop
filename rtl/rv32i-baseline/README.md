# Single-cycle RV32I

| Item | Value |
|---|---|
| CPU parameters | `CLOCK_HZ=25000000` |
| Clock | 25.000000 MHz |
| CoreMark iterations | 600 |
| Measured ticks | 440,011,959 |
| CoreMark | 34.089983 iterations/s |
| CoreMark/MHz | 1.363599 |

## CoreMark compilation

Compiler: **GCC 13.2.0**.
The following flags were reported in the board UART output for the measured run:

```text
-O3 -march=rv32i -mabi=ilp32
```

## Files

- [Core implementation](cpu.v)
- [CPU top module](cpu.v)

The original single-cycle module was preserved inside the Bacon archive, alongside its shared RTL dependencies.
