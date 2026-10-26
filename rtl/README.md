# Representative CPU generations

| Design | Parameters | MHz | CoreMark | CoreMark/MHz |
|---|---|---:|---:|---:|
| [Single-cycle RV32I](rv32i-baseline/) | Original single-cycle module | 25.000 | 34.089983 | 1.363599 |
| [Bacon (1-wide)](bacon/) | FAST=1 | 100.000 | 287.372358 | 2.873724 |
| [Super Bacon (2-wide in-order)](super-bacon/) | FAST=2 | 100.000 | 331.715677 | 3.317157 |
| [Out-of-Bacon (2-wide renamed)](out-of-bacon/) | FAST=3 | 25.000 | 95.475903 | 3.819036 |
| [Out-of-Bacon II (3-wide OoO)](ob2/) | FAST=4 | 100.000 | 544.299080 | 5.442991 |
| [Crispy Bacon A (case-study baseline)](cb-v28a/) | FAST=5 | 130.137 | 705.528305 | 5.421428 |
| [Rare Bacon (predictor-modified core)](rare-bacon/) | FAST=6 | 80.000 | 394.342582 | 4.929282 |
| [Crispy Bacon B (MW-A fix)](cb-v28b/) | FAST=5 | 130.137 | 709.506215 | 5.451995 |
| [Crispy Bacon F (final)](cb-v28f/) | FAST=5 | 130.137 | 730.562894 | 5.613799 |

The paper performance table uses the baseline, Bacon, Super Bacon, OB2, and final F rows.
The other rows connect the architectural history and long-run case study.
The original baseline module has no FAST parameter. Each design README lists its integer CLOCK_HZ parameter and the clock used for scoring.
CoreMark scores are calculated as iterations × clock frequency / measured ticks.
