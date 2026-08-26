# NDT Corner Guard Experiments - 2026-08-26

Goal: reduce the two remaining loop3 corner mismatches without using dataset pose topics as runtime priors. Runtime inputs remain `/livox/lidar`, `/livox/imu`, and the prior map; FASTLIO2Location `/localization` is used only for offline evaluation.

Baseline kept as current best default config (`f03b54f` parameters):
- Run: `/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`
- FASTLIO2Location aligned comparison: mean 0.417 m, RMSE 0.618 m, median 0.245 m, p90 1.243 m, p95 1.535 m, max 2.694 m.

Full-bag experiments rejected:
- `ndt_finer_step035_loop3_20260826_152941`: global step limit 0.35 m. Worse: mean 0.468 m, p95 1.567 m, max 4.490 m. The 398-403 s corner worsened.
- `ndt_finer_step075_loop3_20260826_154620`: global step limit 0.75 m. Unsafe: mean 0.544 m, p95 1.557 m, max 8.305 m.
- `ndt_finer_guard12_decay08_loop3_20260826_155711`: large raw jump guard with constant-velocity coast. Unsafe drift: mean 0.746 m, p95 3.924 m, max 7.077 m.
- `ndt_finer_accept_jump12_loop3_20260826_160746`: existing acceptance gate enabled. Bad freezes: corrected output drops to 3.55 Hz, mean 43.494 m, max 108.523 m.
- `ndt_confident_step065_loop3_20260826_162040`: conditional low-fitness/small-rotation step relaxation to 0.65 m. Worse: mean 0.427 m, p95 1.541 m, max 5.317 m.
- `ndt_confident_step055_strict_loop3_20260826_163117`: stricter conditional relaxation to 0.55 m. Worse: mean 0.456 m, p95 1.558 m, max 5.465 m.

Diagnostic conclusion:
- Remaining bad windows often coincide with raw NDT jumps that are currently clipped to 0.5 m.
- Simply tightening the clip under-corrects corners; simply loosening it allows wrong attractors elsewhere.
- Reject/coast-style guards accumulate drift or freeze the corrected stream.
- The added `ndt_confident_step_limit_*` parameters are therefore left disabled by default. They are retained only as a reversible experimental hook for future, more selective gating.

Next direction:
- Do not keep tuning scalar step limits alone.
- The next promising route is a true two-stage candidate verifier: run NDT from the motion-propagated initial guess and several nearby hypotheses, then choose by a combined score of NDT likelihood, nearest-map residual, and motion-prior distance. It must be evaluated full-bag before changing the default.
