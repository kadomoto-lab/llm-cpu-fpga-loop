# T1 — Steady-state per-iteration profiler (agent `prof`, worktree `b3-wt/prof`, branch `apex-prof`)

## Why
Every design decision so far used either a 10-iteration diagnostic (~1.9M cycles, not
predictive) or a 4–5 h full 8,000-iteration simulation. Fact established by the
orchestrator: the 10-iteration diagnostic binary, the 130 MHz scored binary and the
80 MHz scored binary have **byte-identical code except 2 constant words**
(`diff build/programs/Coremark-cb-rv32im-diagnostic-cb52/code.hex
build/programs/Coremark-cb-rv32im-130137-8000/code.hex`). So Rare Bacon's −3.2% (10 iter)
vs +9.98% (8000 iter) is NOT layout aliasing — it is run length: the predictor's
behaviour changes over time. We need a fast metric that measures steady state.

## Deliverables (do them in order; commit after each)

1. **Iteration marker in `sim/main.cpp`** (do not change any RTL):
   - Env `ITER_PC=<hex>`: count retirements of that PC across all three commit lanes
     (`debug_commit/debug_pc`, `debug_commit2/debug_pc2`, `debug_commit3/debug_pc3`).
   - Env `ITER_HITS=<k>` (default 1): one iteration = k hits.
   - On every completed iteration print to stderr `ITER <n> <cycle> <retired>`.
   - Env `ITER_STOP=<N>`: after N iterations print `ITER_DONE` and exit 0 (skip the
     UART/MMIO checks and the "SIMULATION TIMEOUT" error in that case only).
   - Behaviour with these env vars unset must be byte-identical to today.
2. **Pick the marker.** Use `prog.dump` of `build/programs/Coremark-cb-rv32im-130137-8000`.
   Find a PC retired a fixed k times per benchmark iteration (e.g. entry of
   `core_bench_list`, called with finger 1 and -1 each iteration in `iterate()`; check
   whether it is inlined). Prove it: on the 10-iteration diagnostic firmware, total
   hits inside the benchmark interval must be exactly 10·k. Write the PC and k into
   `results/apex/prof/marker.md`.
3. **Fast simulator.** Check whether Verilator assertions dominate run time
   (`ifdef VERILATOR` blocks in `rtl/cb_cpu.vh` / `rtl/rb_cpu.vh`). If so, add an
   opt-out define (e.g. `+define+NO_SIM_ASSERT`) that is OFF by default. Report cycles/s
   with and without it. Build into `build/apex-*` directories only.
4. **Validate the metric** on the exact scored firmware
   (`python3 scripts/build_programs.py --coremark-only --march=rv32im --clock 130136986
   --iterations 8000 --unroll --name Coremark-cb-rv32im-130137-8000` if not present):
   - v28-A frozen RTL (`results/optimization/cb-rv32im-v28a-130/rtl`, `-GFAST=5`),
     `ITER_STOP=300`. Mean cycles/iteration over iterations 50..300 × 8000 must be within
     0.2% of the known full-run 1,475,625,972 ticks (184,453.2 cycles/iter).
   - Rare Bacon frozen RTL (`results/rare-bacon/frozen-tage-lite/`, `-GFAST=6`) on the
     80 MHz firmware `Coremark-rb-rv32im-80000-8000` (`--clock 80000000`), `ITER_STOP`
     large enough to see the trend (try 300, then 2000 if it is still drifting).
     Known full-run: 1,622,954,326 ticks (202,869.3/iter).
5. **Report** `results/apex/prof/report.md`: a table of per-iteration cycles for both
   cores at iterations 1,2,5,10,20,50,100,200,300(,1000,2000); whether RB degrades over
   iterations and when; the steady-state estimate vs the known full runs; sim speed.
   Save raw stderr logs next to it.

## Rules
- No RTL behaviour changes. No Vivado, no board access.
- Run long simulations detached: `setsid nohup <cmd> > log 2>&1 & disown`, then poll.
- `make lint` must still pass if you touch any `.v/.vh` (only the assert guard).
- Do not delete or rewrite anything under `results/` that you did not create.
- When finished, reply with a ≤15-line summary and the commit hashes.
