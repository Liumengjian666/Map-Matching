# NDT candidate/IMU initial guess experiment - 2026-08-26

Goal: test the idea of using motion prediction as a prior when multiple NDT hypotheses have similar scores, without using dataset pose topics at runtime.

Runtime inputs used by validation scripts: `/livox/lidar`, `/livox/imu`, `/clock`; FASTLIO2Location `/localization` was used only offline for comparison.

Baseline saved before these experiments: `8b3daf7` (`safe_step_limit_no_truth_loop3_20260826_010620`): mean 0.646 m, p95 3.515 m, max 5.608 m.

Experiments:

- `candidate_prior_select_loop3_20260826_024729`: multi-initial NDT candidate selection triggered too often from startup. Result: mean 35.723 m, p95 83.314 m, max 92.934 m. Rejected.
- `candidate_prior_gated_loop3_20260826_025957`: candidate selection gated by prediction jumps only. Result: mean 0.710 m, p95 3.529 m, max 7.125 m. Worse than baseline, especially around 401.8 s with amplified Z error. Rejected.
- `imu_initial_candidate_xy_loop3_20260826_031436`: direct `/livox/imu` gyro propagation blended into NDT initial orientation. Result: mean 4.677 m, p95 29.370 m, max 39.228 m. Rejected; likely frame/bias sensitivity.
- `z_step_limit_loop3_20260826_032709`: per-frame Z clamp. Result: mean 3.120 m, p95 25.569 m, max 33.549 m. Rejected; Z clamp caused long lateral drift.
- `commit12154e5_real_loop3_20260826_040054`: separate worktree at `12154e5` with old config and old node. Result: mean 4301.764 m, p95 12847.841 m, max 13986.607 m in this full-bag harness. This does not support reverting directly to `12154e5` for the current loop3 validation path.

Conclusion: simple candidate reselection, direct gyro initial orientation, and Z clamp all degrade full-bag metrics. Keep current saved baseline code; next promising direction is not to let NDT choose a different candidate, but to add an ambiguity detector that freezes or lowers correction weight only inside detected repeated-structure windows, then releases using consecutive consistency.
