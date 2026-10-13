# Scoped source changes

- `core/deskew_trajectory_mode.hpp`: three named modes, conflicting flags reject,
  Deskew-only immutable configuration copy.
- `dog_prior_map_ekf_node.hpp` and node constructor: new default-false
  `deskew/experimental_history_nominal_consistent_enable`; require the existing
  mature time adapter and IMU Deskew only when an experimental mode is selected.
- `imu_deskew_runtime.cpp`: pass local copied config to the same historical
  query in D; retain P30 scan-local branch. No point/SE(3)/time/extrinsic changes.
- `tests/deskew_trajectory_mode_test.cpp`, CMake: minimal branch regression.
- `config/p31_history_nominal_consistent.yaml`: explicit D overlay.
- `scripts/p28_visual/run_replay.py`: bounded P31 D/E mode selection, runtime
  parameter checks and correct P31 execution identity. Old P28/P30 flags retained.
- `scripts/p31_ablation/`: preparation, gates, shared frozen evaluation,
  artifact tests and light archival. No GT search or trajectory rewrites.

Unmodified: core/imu_deskew.cpp, core/imu_propagation.cpp, IMU processor,
MSCKF residual/selection/update, NDT/DCReg/Multi-NDT/Step Limit, velocity_blend,
noise, map, observations, initialization, state-history/OOSM/root mathematics.
No writes to robot/paper workspaces or old archives. Historical full A/B/C are
reused, not rerun. No source capture or stepwise covariance audit in P31.
