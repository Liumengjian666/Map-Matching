# R31 single observation-identity semantic repair

START edb4e913bdbaec2a493252cd97b7c842f9e41109 was fetched from the approved remote branch and matched exactly. Worktree p62_r31_observation_identity_ws, execution branch research/p62-r31-observation-identity-closure. Other worktrees and historical experiments were not reset or overwritten.

Production source CODE bab26e991ef7ac871057ee4f9a94c92831619a62:

- `include/dog_prior_map_localization/observation_identity.hpp`: ROS-free LEGACY/STRICT_FAIL_CLOSED decision and the existing step-limiter arithmetic extracted unchanged for testability. No new motion bound, quality threshold or optimizer.
- `src/dog_prior_map_ndt_node.cpp`: selected candidate's original quality/convergence certificate is retained. STRICT evaluates the old limiter without committing it; an altered result is rejected before `previous_pose_`, `delta_pose_`, current NDT pose, timestamp history or map observation/pose/path/TF publication. Refusal leaves raw candidates and normal IMU processing intact. The final observation on success is the original certified terminal, not the proposed limited pose.
- `publishPose()` records the actual message representation; per-frame identity log records certified/proposed/sent matrices, previous time/age, navigation state invariance and publication counters. The node throws if a strict refusal nevertheless changes navigation output state.
- `tests/observation_identity_contract_test.cpp` and CMake provide an independent ROS-free test executable.
- New standalone `p62_r31_observation_identity.launch` carries explicit policy and log path. Both code and launch default to LEGACY; existing production launch files were not edited. STRICT is restricted to the approved C/G research path with other mechanisms off.

The only later change is an orchestration correction in the new R31 supervisor (commit739b4873dc3f90d924595b9b6dad3657fec89836): it waits until both ROS components have appeared before diagnosing their disappearance. The prior startup guard stopped C's first launch before any cloud/IMU/NDT processing. Its zero-input records remain intact; B is not rerun. This does not change the localization binaries or parameters.

R30's already-accepted target construction fix remains present. R29 used the already-correct Tile path; all intended Gaussian/NDT parameters, source preprocessing, target rules, quality checks, yaw starts, quotas, extrinsics, IMU/Deskew/EKF/OOSM mathematical sources remain frozen. No D/E/F/Coupled behavior is enabled. Full build links existing optional modules but does not run them.

Counterfactual LEGACY compatibility is exact at raw62 (full4x4 difference0). Default LEGACY still publishes the old limited result under the existing upstream admission; it is not secretly subjected to a new Coverage check. No new complete LEGACY trajectory is claimed; controls are the sealed R29/R24 runs.
