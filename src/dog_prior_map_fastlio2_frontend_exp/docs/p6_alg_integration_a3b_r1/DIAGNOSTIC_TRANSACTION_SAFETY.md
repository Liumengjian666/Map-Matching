# Diagnostic transaction safety

- Failure diagnostics are gated by `P6_A3B_R1_DIAGNOSTICS=1`; `capture_optimizer_trace` defaults off. The diagnostics-disabled optimized result is compared to trace-enabled output in the synthetic regression: state, status, iteration count, initial cost and final cost are identical.
- `objectiveBreakdownForDebug()` calls the same block linearized system and reports prior/IMU/LiDAR/visual contributions. The synthetic test checks component sum versus total and exercises all four factor categories.
- `objectiveBreakdownAtStatesForDebug(..., freeze_lidar_bases_at_x0=true)` copies the Window and freezes only copied factor callbacks at x0. The test verifies original H/g/cost and objective remain bit-identical after debug evaluation.
- Directional finite differences perturb copies of the state vector. The 13-point damping sweep also applies steps only to local copied states and uses the same candidate objective evaluators.
- Post-diagnosis maximum state `localDifference` is exactly `0`; test also compares the failed Window state to its pre-diagnosis snapshot. This satisfies the requested `<1e-14` condition.
- Failure trace, objective breakdown, FD rows, sweep rows and summary are flushed before the original optimizer failure is rethrown. The tx83 failure summary records `diagnosis_ok=1`, `diagnostic_transaction_test=PASS`, and `diagnostic_state_local_difference_max=0`.
- Source/audit compatibility: the frozen legacy V2 producer byte-parity audit remains passing; the source audit was made tolerant of the V3-only `std::getenv` include and the existing selected-measurement local variable name (`preview_measurement`). No V2 algorithm branch changed.

The diagnostic additions do not fix the basis derivative and do not change the production objective, damping policy, step limit, factor acceptance, covariance, NIS, or Window length.
