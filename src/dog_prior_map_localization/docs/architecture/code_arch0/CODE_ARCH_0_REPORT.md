# CODE-ARCH-0：`dog_prior_map_localization` architecture audit

日期：2026-09-20（Asia/Shanghai）  
范围：`/home/jian/livox_ws/dog_visual_loc_ws/src/dog_prior_map_localization`  
性质：只读静态审计；没有修改源码、配置、launch、CMake，没有编译、没有运行 rosbag、没有 Git commit/push。

## Git

```text
Branch: feature/visual-factor-window
HEAD:   d4e6cfbbe15e0c3bb15e84c5394db78ca2e1e5ff
Remote formal baseline requested by Stage: same SHA
unstaged diff: empty
staged diff: existing user-provided staged work, preserved unchanged
source modified by this Stage: no
```

审计开始前执行了 `git branch --show-current`, `git rev-parse HEAD`, `git status --short`, `git diff`, `git diff --cached --name-only`。当前暂存区仍是用户已有的 58 个路径；本 Stage 没有 `git add`、`git reset`、`git restore` 或提交。

## Current architecture

### Runtime executables

1. `dog_prior_map_ekf_node_cpp`: integrated mode. One `DogPriorMapEkfNode` class spans IMU propagation/covariance, integrated map matching/NDT, map loading, visual frontend, OOSM, directional state machine and all outputs.
2. `dog_prior_map_ndt_node_cpp`: split mode NDT process. It owns PCD/map target, scan timing, pending/prediction/IMU history, PCL NDT, degeneracy/information/Schur diagnostics and NDT ROS outputs.
3. `dog_prior_map_ekf_node.py`: legacy fallback, launchable only through `use_cpp:=false`; not linked to C++ targets.

### Offline tools

- `ndt_uncertainty_probe` and `ndt_direction_alignment_probe` are independent CMake executables.
- `scripts/pcd_to_npz_map.py`, evaluation/comparison scripts, replay shell helpers and translation tooling are not runtime algorithm targets.
- Stage3B-0 metric visual feasibility remains external under `/home/jian/rosbag/loop2/stage3b0_metric_visual_feasibility_20260920/`; it is recorded, not imported into runtime.

### Largest responsibility hotspots

1. `src/dog_prior_map_ndt_node.cpp`, 2202 LOC: map/scan/timing/NDT/diagnostic/ROS God file.
2. `include/.../dog_prior_map_ekf_node.hpp`, 558 LOC: ~190 mutable fields and all EKF/visual/map/output declarations.
3. `src/lidar_matcher.cpp`, 1178 LOC: integrated ROS adapters, deskew, local map, NDT, point/plane/hybrid solver, degeneracy and acceptance.
4. `src/vision_observation.cpp`, 1064 LOC: image frontend plus legacy yaw, directional state machine, NDT OOSM rollback/replay.
5. `src/ros_output.cpp`, 463 LOC: output/CSV adapter that reads most mutable runtime state.

The problem is not merely line count. The same mutable `DogPriorMapEkfNode` state is written by IMU, LiDAR, visual and OOSM code, and split NDT has a second independent pose/history model.

## State ownership

- EKF nominal/covariance owner: `DogPriorMapEkfNode` fields `p_, v_, R_, ba_, bg_, P_`; direct writers are `imu_processor.cpp`, `lidar_matcher.cpp`, `vision_observation.cpp` and snapshot restore.
- OOSM owner: `state_history_` and rollback/replay code in `imu_processor.cpp` + `vision_observation.cpp`.
- IMU history owner: EKF `imu_history_` in `imu_processor.cpp`; independent NDT `imu_history_` in `dog_prior_map_ndt_node.cpp`.
- NDT state owner: `DogPriorMapNdtNode` fields `p_, R_, previous_pose_, delta_pose_`, pending and prediction histories.
- Map owner: integrated EKF `map_cloud_/map_kdtree_` via `map_loader.cpp`; split NDT `map_cloud_/target_cloud_/schur_target_tree_` via NDT `loadMap()`.
- Visual state owner: EKF `last_gray_`, feature vectors, last image pose and all visual telemetry in `vision_observation.cpp`; output reads it in `ros_output.cpp`.
- Schur state: local per-frame result; only KD-tree/build-time persist in NDT node.

## Runtime / diagnostic separation

- **Clean:** independent uncertainty/direction probe executables; Schur result serialization and determinism CSV are output-only when enabled.
- **Mixed:** information analysis can publish telemetry consumed by directional EKF; degeneracy topic drives visual/state flags; all telemetry runs inside sensor callbacks.
- **High-risk runtime controls:** integrated degeneracy projector, NDT information pose projection, step limits, acceptance gates and NDT-difference velocity feedback change state/output. They must not be mislabeled as diagnostic-only.

## Legacy

- **KEEP-FALLBACK:** Python node; integrated NDT branch; timestamp/prediction fallback and local gyro prior.
- **KEEP-EXPERIMENT:** legacy yaw, directional projector/state machine, alternate matcher branches.
- **MOVE-OFFLINE-CANDIDATE:** Schur/information/probe helpers and the disconnected `LivoxImuDeskewer` implementation (after an explicit owner decision).
- **DEPRECATE-CANDIDATE:** stale config families and keys (`lidar_odometry`, `external_odometry`, anchor/vertical relocalization, GICP/ICP fields) not read by current C++.

## Target architecture

Use small concrete modules: `nodes/`, `core/`, `map/`, `ndt/`, `vision/`, `fusion/`, `diagnostics/`, `common/`, `tools/offline/`. Keep ROS NodeHandle/subscribers/publishers in adapters. Keep `State15`, `TimedMeasurement`, NDT timing, map ownership and OOSM as typed objects. Future fusion should enter through one `MeasurementManager`, not through `DogPriorMapEkfNode` or `VisualFrontend` directly.

## Stage3B placement

- **VisualFrontend:** `vision/visual_frontend.{hpp,cpp}`; owns image conversion, feature tracking and relative pose/quality only; no NodeHandle and no EKF writes.
- **LidarDepthAssociator:** `vision/lidar_depth_associator.{hpp,cpp}`; consumes an explicit scan/depth context and returns associations; no map/ROS globals.
- **MetricVisualEstimator:** `vision/metric_visual_estimator.{hpp,cpp}`; converts relative visual geometry plus validated LiDAR/depth scale into a metric relative measurement; no direct state mutation.
- **VisualMeasurement:** `core/measurement_types.hpp` or `vision/visual_measurement.hpp`; immutable timestamp, pose/velocity/uncertainty/validity/reason fields.
- **Future fusion entry:** `fusion/measurement_manager` called by EKF node after timestamp alignment. It is the only place allowed to choose full/reliable/weak subspace update. First version stays diagnostic-only until visual metrics pass.

## Refactor priority

- **BLOCKING:** CODE-ARCH-1 timestamp/measurement records; preserve split baseline and exact lineage. A thin visual frontend boundary is blocking before formal fusion if it can be proven behavior-neutral.
- **NEAR-TERM:** isolate NDT timing/map and EKF OOSM manager.
- **OPTIONAL:** move runtime diagnostics/probes behind immutable telemetry snapshots.
- **POST-RESEARCH:** delete/deprecate stale profiles, remove dead branches only after ablations and reproducibility records.

## Recommended `CODE-ARCH-1` (design only; not executed)

- **Target:** extract timestamped records and snapshot types, not algorithms.
- **Files:** new `core/state_types.hpp`, `core/measurement_types.hpp`; minimal signatures in NDT/EKF headers/sources.
- **Move:** `TimedPrediction`, `PendingLidarFrame`, `ScanTiming`, `PredictionSelection`, `FilterStateSnapshot` and pure pose-difference/validity helpers.
- **Forbidden change:** NDT math, IMU equations, OOSM replay order, topics, YAML values, outputs, visual behavior, Schur/information formulas.
- **Determinism verification:** fixed split bag with diagnostics off/on; cloud hash, initial guess, raw NDT, final used pose, reference stamp and OOSM results must match exactly/tolerance zero; compare frequency and metrics.
- **Rollback/acceptance:** one isolated commit from `d4e6cfb`; build succeeds; no new queue/callback; no metric/resource regression; reject and return to `d4e6cfb` if any mismatch.

## Final answers

1. **Most serious structural problem:** `DogPriorMapEkfNode` and `DogPriorMapNdtNode` combine state ownership, ROS I/O, algorithm policy, timing and diagnostics; direct cross-file state writes make behavior contracts implicit.
2. **Files to split first:** NDT monolith, EKF header/state, then `lidar_matcher.cpp` and `vision_observation.cpp`. Split by ownership boundaries, not arbitrary line count.
3. **Already clean:** `math_utils.cpp`, `map_loader.cpp` responsibility-wise, independent offline probes, and the standalone deskewer internals (though the latter is disconnected from build).
4. **Are runtime and offline diagnostics truly isolated?** No. Probe executables are isolated, but runtime NDT/EKF diagnostics and degeneracy/information topics remain in runtime processes and some can drive control flow.
5. **Which old code pollutes runtime?** Python fallback, alternate matcher/NDT paths, legacy visual yaw, directional state machine, stale prediction/degeneracy branches and unused config families compiled/loaded in the same class.
6. **Best Stage3B visual location:** a ROS-free `vision/VisualFrontend` plus typed `VisualMeasurement`, followed later by depth association/metric estimator.
7. **How to keep visual out of EKF God class?** Frontend returns immutable measurement; only `MeasurementManager` invokes a typed EKF update. Frontend cannot see NodeHandle, `p_`, `R_` or `P_`.
8. **Is target over-engineered?** Not if limited to concrete modules and one measurement boundary; factories/plugins/inheritance/singletons would be overkill.
9. **Refactors required before formal Stage3B fusion:** make timestamp/measurement semantics explicit (CODE-ARCH-1), then isolate OOSM and visual frontend enough to prove no hidden state writes.
10. **First actual refactor:** CODE-ARCH-1 as specified above; design only in this Stage.

## Artifacts

- `source_inventory.csv`
- `cmake_target_graph.md`
- `runtime_ros_graph.md`
- `ndt_node_responsibility_map.md`
- `ekf_responsibility_map.md`
- `state_ownership.md`
- `runtime_call_graph.md`
- `complexity_hotspots.md`
- `parameter_ownership.csv`
- `diagnostic_separation_audit.md`
- `legacy_code_audit.md`
- `target_architecture.md`
- `refactor_stage_plan.md`
