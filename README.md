# LLM Agent-Based Design of RISC-V CPUs with FPGA-in-the-Loop Evaluation

Conversation records and RTL snapshots accompanying the paper by
Leo Itoh and Junichiro Kadomoto, The University of Tokyo.

## Contents

- [Conversations from the long-run case study](long-run-case-study-conversations.jsonl):
  selected exchanges covering the long-run result, diagnosis, and fix.
- [Task instructions](tasks/): the instruction files referenced in the conversations.
- [CPU designs and RTL](rtl/README.md): the representative implementations,
  configuration parameters, and reported performance.

## Layout

```text
long-run-case-study-conversations.jsonl    Selected exchanges in time order
tasks/                 Referenced task instructions
rtl/
  rv32i-baseline/    Original single-cycle RV32I
  bacon/            Single-issue pipeline
  super-bacon/      Two-wide in-order core
  out-of-bacon/     Two-wide renamed core
  ob2/              Three-wide out-of-order core
  cb-v28a/          Baseline for the long-run case study
  rare-bacon/       Predictor-modified core
  cb-v28b/          Load-wait predictor fix
  cb-v28f/          Final design
```

## Conversations

These conversations accompany the long-run predictor case study in Section IV-C.
Their coverage differs from the development activity summarized in Table I;
this file is not the dataset used to calculate that table.

The collection contains 132 messages in five connected exchanges.
Each selected source-thread interval is retained in full.
Statements later corrected by the participants remain
as originally recorded.

| Exchange | Contents | Period (JST, 2026) |
|---|---|---|
| `01-long-run-result` | Board result, full-length RTL comparison, human status questions, and final regression report | Oct 13 |
| `02-initial-interpretation` | Human questions about the regression and the initial explanation | Oct 13 |
| `03-diagnosis` | Per-iteration profiling request, investigation, follow-up instructions, and absorbing-state diagnosis | Oct 17 |
| `04-fix-comparison-and-selection` | Fix request, comparison of alternatives, MW-A selection, and subsequent regression-check discrepancy | Oct 17 |
| `05-board-acceptance` | Timing completion, programming discussion, and acceptance of the fixed design | Oct 17 |

Each line is a JSON record with a timestamp, session label, sender, recipient,
and message text. The `exchange` field identifies the group above.
Records are sorted by timestamp; the JSON timestamps use UTC.
Session labels identify the participants, and `thread` distinguishes the original
sessions when a worker was restarted. The original message role is retained separately from the sender
because a worker receives orchestrator instructions under the `user` role.

The export contains conversation text.
Local paths and machine, account, and device identifiers are anonymized.

The `task_files` field links messages to the referenced instructions in `tasks/`.

## Reading the RTL

Each `rtl/<id>/` directory is a separate source snapshot. Select one
generation at a time; the snapshots define modules with the same names.
Each generation's README lists the CPU parameters, measured clock, and results,
and links to its core implementation.
The final design is `cb-v28f` (FAST=5, 130.137 MHz).

Benchmark source distributions, executable firmware, and FPGA bitstreams are
not included.

The original experiment used a Nexys Video (Artix-7 XC7A200T), Windows Vivado
2024.2, and a WSL2 host. The final design's memory map has 32 KiB of instruction
memory and 64 KiB of data memory. UART output uses 115200 baud, 8N1.

See [NOTICE.md](NOTICE.md) for source attribution.
