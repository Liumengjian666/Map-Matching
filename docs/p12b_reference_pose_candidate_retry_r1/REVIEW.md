# Bounded P12B interface review

Fresh-context read-only reviewer inspected adapter, caller, freeze script,
tests, CMake and evaluator. No external model CLI invoked. This review is
limited to the actual approximate-pose hypothesis contract; it does not
certify historical extrinsics, reference correctness or exact scan timing.

Findings and reconciliation:

- Actionable: repeated target hashing in candidate logging would add unnecessary
  instrumentation to combination_ms. Cache the already-validated target hash
  once before the experiment. Fixed before any real registrations; no solver
  behavior or parameter changed.
- Actionable: invalid output poses yield NaN geometric increments; summary
  stats previously would abort rather than report the intended failure counts.
  Geometry-only statistics now emit finite contributor counts, excluded count
  and unavailable null values when no valid increment exists. Invalid/nonfinite
  result counters remain separate. Strict timing stats still reject NaN.
  Mixed/all-NaN and strict-timing direct tests passed. No extra author call.
- Trade-off, explicitly accepted by user task: exact map_T_lidar and time
  transport are unproven. B is explicitly approximate reference state pose,
  retains the original translation, and does not apply guessed calibration.
- Trade-off: A cost is historical; it is not a same-process timing control.
  A outcomes and reference inputs are frozen; new solver parameters remain
  identical. No performance superiority claim is drawn.

Five-axis final check: narrowly scoped adapter/caller/tests, validated input
association and SE3, original dependencies reused, no author license/publication
assumption, no production/EKF/map changes. No solver refactoring or new
descriptor/decision model. Fixed18 calls completed, no parameter rerun.

The review influenced failure reporting and instrumentation only. It did not
change DCReg/SOLiD algorithms, experiment sample IDs, thresholds or outcomes.
