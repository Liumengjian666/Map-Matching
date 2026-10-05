# Read-only adversarial review disposition

Independent reviewer: `/root/p9_r1a_oracle_review`. No writes, optimizer execution, GT access, or network access.

Two actionable initial findings were fixed before the archived solver run:

1. An incomplete pattern polling sweep could be labeled `POLL_STEP_SMALL` when the100-call budget was reached. The solver now preserves `EVALUATION_BUDGET`; a deterministic interrupted-sweep regression test was added.
2. Accepted endpoint trace rows contained the pre-step Newton gradient without stating that provenance. The column is now explicitly `branch_projected_gradient_before_step`; endpoint projected-gradient diagnostics are separately pulled through the global exponential chart.

Final independent recomputation found no new actionable issues. It verified all22 frozen oracle IDs and actual best-score membership; original projection13/inside18/intersection9; two cluster departures leaving GROUP A7; recoveries2/3/4/7/3; the4/7 gate failing70%; pose distances (maximum independent rotation discrepancy5.94e-6deg), FD statistics, boundary neighborhoods, monotonic accepted energies, budgets, diagnostic counts, and CSV hashes.

Scientific limits were reviewed explicitly: proximity-preserving successful refinement is not dynamic stationarity; budgeted inner optimization is not a certified global min_v; support switching is not proven to be the only cause; no transported-W, adaptive grid, posterior weights or production fusion is justified by these results.

The user confirmed the exact invocation, and Codex CLI v0.159.2 (`gpt-6.1-sol`, xhigh, ephemeral, read-only) completed the review in `REVIEW_PROMPT.md`. It independently reproduced all481 primary clusters in the9 target frames,307 major-cluster members,22 targets, zero provenance mismatches, original geometry13/18/intersection9, GROUP A7, and recovery2/3/4/7/3. Pose chart and boundary fields also agreed.

CLI found two additional actionable support-trace issues:

3. ONE_STEP_NEWTON had no trace and therefore falsely reported zero maximum accepted support change. The wrapper now records its existing initial and accepted endpoint support without altering the legacy solver or adding budgeted solver evaluations.
4. Pattern search recorded sweep endpoints rather than every accepted poll/pattern move (102 actual accepts versus75 recorded transitions). It now records every accepted event immediately, preserving its100-call budget. An explicit accepted_move index and completeness checks were added.

The pre-fix archived outputs fail the completeness check on19 method/target cases (RED). After fixing the logging, the full35 comparisons and280 directional FD rows were regenerated. Build/CTest and post-fix completeness/pose-distance/monotonicity/budget/CSV-JSON checks PASS; recovery and all registration poses remain unchanged. This is a reviewed-and-fixed result, not a claim that the initial CLI review returned no findings. No second CLI invocation is claimed.

The independent reviewer then verified218 events:35 initial states plus183 accepted moves (one-step7, Newton74, pattern102). It identified a validator-only compatibility issue: the frozen one-step acceptance threshold is1e-12, whereas the new solvers use1e-10. The validator now applies the method-specific threshold; current numerical results are unaffected. Final consistency checks use this corrected validator.
