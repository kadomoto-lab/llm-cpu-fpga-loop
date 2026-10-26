# T1c — Make the load-wait predictor non-absorbing (agent `prof`, worktree `b3-wt/prof`)

T1b found Rare Bacon's +13% step is one `mem_pred` entry entering distance 7 ("wait for older
stores to commit") — an absorbing state. v28-A has the same predictor (`cb_cpu.vh`, mem_pred /
mpq / tr_d / P_LW). Smoked Bacon (FAST=7, branch apex-sb) inherits it, and its TAGE will perturb
timing the same way. Also v28-A may *already* hold such entries from iteration 1 (it is flat
from iteration 2, which an early absorbing state would also look like).

## Steps (sim-only on a COPY of the frozen v28-A RTL under build/apex-mw-*/rtl)
1. **Audit v28-A**: over ITER 1..300 on the scored 130 MHz firmware, list every mem_pred entry
   that is ever written (PC, value, cycle/iteration of each write) and the steady-state set of
   conflict/d=7 entries. Per such PC: flagged-load (P_LW) grants and memory-RS row-cycles per
   iteration. Same for the d=0..6 entries. Is any v28-A entry in the absorbing state?
2. **Fix candidates** (keep each small, no new adders on known critical paths; describe the
   timing exposure of each):
   (a) do not write d=7 when the replaying load found no covering store in the SQ (T1b says
       `tr_d` defaults to 7 exactly then) — write "no prediction"/clear instead, or keep the
       previous distance;
   (b) decay: a free-running counter clears the conflict/d=7 state of one entry every 2^k cycles
       (round-robin index), or clears the whole table every 2^N cycles (try N=16..20);
   (c) a d=7 load that issues and finds no older aliasing store in the SQ decrements a 1–2-bit
       confidence and drops to "no prediction" at zero.
3. Measure each on: v28-A scored firmware ITER 2000 (mean d(51..2000), must not be worse than
   184,453; any gain is a free win for the shipped core) and the RB frozen RTL + RB 80 MHz
   firmware ITER 400 (must remove the iteration-238 step). Check CRCs on the 10-iteration
   diagnostic and `scripts/regress_cb.sh` on the variant.
4. Write `results/apex/prof/memwait.md` (audit table, variant table, recommendation with an RTL
   patch against the frozen cb_cpu.vh as `results/apex/prof/memwait-<variant>.patch`). Commit,
   reply ≤8 lines. Do NOT edit rtl/ or the apex-sb branch — the orchestrator routes the patch.
Rules: wait on PIDs, detached long runs, ≤4 parallel sims.
