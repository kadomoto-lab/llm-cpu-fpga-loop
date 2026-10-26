# T8 — Cycle accounting (CPI stack) of v28-A+MW-A (agent `prof`, worktree `b3-wt/prof`)

## Why
Next lever after MW-A. We know ~3,700 redirects/iter and ~16% wrong-path issue, and
that layout swings come from issue-grant idle cycles. We need one table that splits
183.4k cycles/iter into exclusive causes, so the next RTL change targets the biggest
bucket that does not touch F0 timing.

## Steps
1. On your MW-A sim copy (`build/apex-mw-cb-src`, `+define+MW_A`), add passive probes
   (sim-only, `ifdef`) classifying **every cycle** by the commit slot-0 view, top-down:
   - retiring (k of 4 commit slots used; report slot histogram),
   - head-of-ROB not done because of: load miss/wait (split P_LW wait, store-forward wait,
     mem RS full), mul/div, ALU operand not ready (dependency chain), issued-not-done,
   - ROB empty because of: redirect recovery (cycles since last rd_v ≤ refill depth),
     FQ empty without redirect (fetch bubble: taken-branch block break, BTB miss),
     rename/dispatch stall (RS full per class, ROB full, free-list empty).
   Buckets must sum exactly to total cycles; print per-iteration averages over ITER 52..300.
2. Also per redirect: average penalty (rd_v to first correct-path commit), split by
   cause (cond mispredict, BTB/target miss, JALR/RAS, memory-order replay).
3. Rank the top 20 static PCs by cycles charged at ROB head.
4. Write `results/apex/prof/cpi.md` (tables + 5-line "where the next 5% is" verdict:
   which bucket, what microarchitectural fix, and whether it touches F0/fetch timing).
   Commit. Reply ≤10 lines.

## Rules
Sim-only, probes must not change cycles (ITER lines byte-identical to MW-A ITER 300).
No `rtl/` edits. Write `results/apex/prof/NOTES.md` if context passes ~80%.
