# Scope and review

- core/square_root_covariance.hpp/.cpp: shared QR prediction/update and exact DPD factor row scaling.
- core/estimator_types.hpp: authoritative factor and storage tag; dense covariance accessor.
- imu_processor.cpp: initial/current history factor copies and root propagation; legacy paths retained.
- core/selective_relative_visual_update.cpp: projected root covariances and Joseph QR update, no CI/measurement changes.
- fusion/skf_visual.cpp: factor-aware causal endpoint, exact partial Q factor, rollback/replay and timers.
- fusion/ndt_observation.cpp: factor-aware rollback backup, common DPD and replay checks/timers.
- dog_prior_map_ekf_node.hpp / node_core.cpp: default-off flag, current S, initialization, diagnostics lifecycle.
- core/square_root_diagnostics.hpp: lightweight factor checks, stage timers and sparse checkpoints.
- CMakeLists.txt / tests/square_root_covariance_test.cpp: existing Eigen/Catkin organization, focused tests.
- launch / config / runner: independent P27 overlay, bounded 19 s protocol and sparse output paths.
- scripts/p27_covariance/summarize.py: read-only frozen-output analysis, 80-digit checks and reproducible lightweight archive.

Review scope: covariance ownership, QR sign/zero-Q handling, conditional gain,
rank-one partial Q, default-off branch, factor history tags and immutable replay
inputs. No additional cross-model review or new ROS run was added. Legacy P
is zeroed only as an inactive tagged field, never interpreted as zero covariance.
Dense diagnostics are disabled in root mode rather than treated as authoritative.
Untimed validation rows do not represent measured zero computational cost.

No modifications to NDT optimization/source, DCReg, multi-start strategy, nominal
kinematics, visual measurement/noise/NIS, CI weighting, OOSM sensor-time scheduling,
map, input bag, enhanced visual CSV or GT. CMake rebuilding changed binary hashes;
the exact executable paths and hashes are recorded by the actual launch receipt.

Remaining limitations: covariance inflation, inherited 2.37 m adjacent state jump,
no full camera runtime or GT accuracy evidence, only one 19 s prefix. The actual
P26 matrix test is not represented as a reconstruction of its exact historical F.
