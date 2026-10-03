# Paper Baseline Closure R1

Date: 2026-10-03
Status: `BASELINE_STABLE_AFTER_FIX` for the frozen paper-workspace single-state path on SuperLoc Floor01.

This closes the underlying baseline only. It does not introduce or evaluate Dual-U, visual fusion, recovery, multiple-hypothesis search, fixed-lag estimation, or any new reliability rule.

## Baseline selection and scope

The selected starting point is historical commit `29d1f7def41eced4a197f9dec54f58aec7f1fa32` (`feat: add clean P7 single-state baseline runner`). It is the clean P7-B carrier: current-frame NDT followed by the existing FAST-LIO2/IKFoM full-pose measurement update. P7-C/D/E and the later Dual-U candidate-evidence work are not part of this branch's baseline path. Existing experimental branches and results were not deleted or rewritten.

The replay build used the project-pinned FAST-LIO2 checkout at `7cc4175de6f8ba2edf34bab02a42195b141027e9` with system PCL 1.10.

The baseline remains a scan-triggered, single-state filter: IMU propagation advances the candidate state to the scan-end measurement time, NDT matches one current scan to the frozen prior map, and a successful raw NDT pose is applied at that same time. It does not publish a separately advancing filter state while the NDT transaction is pending.

## Actual execution chain

| Stage | Source and function | Time / state contract |
|---|---|---|
| Input cloud | `src/dog_prior_map_fastlio2_frontend_exp/src/fastlio2_frontend_node.cpp`, `makeTimedCloud()` and `parseScan()` | `PointCloud2.header.stamp` is the raw scan start. The configured point field is `time`, with `point_time_scale_to_seconds=1.0`; each point time is an offset in seconds. The end is the maximum point offset added to the header time and rounded to nanoseconds. |
| Overlap handling | `src/dog_prior_map_fastlio2_frontend_exp/src/scan_processor.cpp`, `prepareScanWindow()` | The committed filter timestamp is the lower bound. Fully stale scans are skipped; points before committed time are removed from a partial overlap. The effective start is `max(raw_start, committed_stamp)`. |
| IMU interval | `fastlio2_frontend_node.cpp`, `imuForScan()`; replay counterpart `src/dog_prior_map_fastlio2_frontend_exp/src/p7_replay_io.cpp`, `imuWindow()` | The chosen causal IMU samples bracket the committed filter time and extend through the exact scan-end timestamp. |
| Prediction and deskew | `src/dog_prior_map_fastlio2_frontend_exp/src/scan_processor.cpp`, `ScanEndProcessor::process()`; `src/dog_prior_map_fastlio2_frontend_exp/src/fastlio2_frontend_ikfom.cpp`, `predictImuSequence()` | A candidate FAST-LIO2/IKFoM state is propagated to `scan_end_ns`. The predicted state stamp is checked against that end time. Deskew uses the predicted pose sequence and fixed nonzero LiDAR/IMU extrinsic. |
| Per-point motion compensation | `src/dog_prior_map_fastlio2_frontend_exp/src/lidar_deskew_geometry.cpp`, `deskewCloudToEndFrame()` | For each point, interpolate IMU position linearly and attitude with quaternion SLERP, form `T_map_lidar(t)=T_map_imu(t) T_imu_lidar`, and transform the point by `T_map_lidar(end)^-1 T_map_lidar(t)`. This includes lever-arm translation and rotation. |
| NDT initial guess and matching | `src/dog_prior_map_fastlio2_frontend_exp/scripts/p7/p7_single_state_runner.cpp` and `src/dog_prior_map_fastlio2_frontend_exp/src/current_frame_ndt.cpp`, `align()` | The only initial guess is the current scan-end IMU-propagated pose transformed to LiDAR: `T_map_lidar_pred`. No previous-NDT pose, previous-pose delta, prediction-age threshold, or fallback branch is used. NDT receives only this deskewed current scan and the long-lived frozen target. |
| Measurement conversion and update | `p7_single_state_runner.cpp`; `src/dog_prior_map_fastlio2_frontend_exp/src/p7_replay_io.cpp`, `lidarMeasurementToImu()`; `src/dog_prior_map_fastlio2_frontend_exp/src/fastlio2_frontend_ikfom.cpp`, `applyPoseMeasurement()` | NDT returns raw `T_map_lidar` at scan end. It is converted as `T_map_imu_meas = T_map_lidar_raw T_imu_lidar^-1`, then the existing full-pose iterated IKFoM update is applied at the unchanged scan-end stamp. The state timestamp and postconditions are checked before the next scan. |

The operational node additionally binds each request/result to a transaction ID and checks result frame and timestamps before committing (`fastlio2_frontend_node.cpp`, `makeRequest()`, result validation, `processOneScan()`; `src/dog_prior_map_fastlio2_frontend_exp/src/frontend_runtime.cpp`, `beginScan()` / `finishScan()`). Only one candidate transaction can be outstanding.

## Time, deskew, frame, and filter audit

### Time / OOSM

There is no measurement/state timestamp mismatch in the closed baseline. The propagated candidate, NDT initial pose, NDT terminal pose, filter correction, and emitted scan odometry all correspond to `scan_end_ns`. A subsequent scan is not processed until the previous terminal result is committed. Therefore this baseline does not exercise a true out-of-sequence measurement: it serializes the scan transaction instead of advancing the state beyond a pending NDT measurement and later rewinding/replaying it. No post-update propagation to a newer timestamp is required because there is no newer committed state yet. This is correct for the current scan-triggered carrier, but it is not a claim that this implementation provides concurrent high-rate output with delayed NDT corrections.

The full replay now starts from original time-stamped points and repeats prediction/deskew for each run. The older XYZ-only request-cloud asset was already deskewed using another run's trajectory and was not used as the baseline replay input.

### Deskew

The existing FAST-LIO2/IKFoM process model performs IMU propagation, including its bias and gravity state, using the pinned FAST-LIO2/IKFoM implementation. `ScanEndProcessor` uses the resulting pose sequence for deskew; this work did not rewrite the process model or invent another deskewer. The raw replay exporter stores XYZ and header-relative point offsets in nanoseconds; the runtime reconstructs each absolute point stamp and checks it lies within the scan. The scan-end pose is an explicit deskew reference. The full extrinsic is used, not a zero-lever-arm approximation.

### Coordinate convention and extrinsic

The filter state is `T_map_imu`. Calibration `T_imu_lidar` maps LiDAR-frame coordinates into IMU coordinates. Thus:

```text
T_map_lidar = T_map_imu * T_imu_lidar
T_map_imu_measurement = T_map_lidar_measurement * inverse(T_imu_lidar)
```

The first equation is used to construct the NDT prediction/deskew pose; the second is used for initialization and NDT measurement conversion. For this Floor01 replay, `params.txt` specifies `T_imu_lidar` translation `(0.080, 0.029, 0.030) m` and quaternion `(0.00134735005987, 0.00258102796952, -0.00453399182125, 0.999985482825)` in `(x,y,z,w)` order. The initial `T_map_lidar` is identity. The nonzero lever arm participates in deskew and pose conversion.

### State, measurement, and update

The pinned IKFoM manifold state is ordered as position, attitude, LiDAR/IMU rotation and translation extrinsics, velocity, gyro bias, accelerometer bias, and gravity (`FAST_LIO2/include/use-ikfom.hpp`). The frontend constrains the calibrated extrinsic to remain fixed. The IMU input is accelerometer and gyro; the pose observation model selects map position and SO(3) attitude. The ordinary baseline measurement covariance is diagonal with position sigma `0.20 m` and rotation sigma `0.10 rad` (variances `0.04 m²` and `0.01 rad²`). The existing `update_iterated()` implementation is used; no covariance/update equation was modified. Failed terminal/postcondition checks retain the existing fail/rollback behavior. No hard reset, step limiter, previous-pose blending, or solution remapping is on this baseline path.

## NDT contract and fixed configuration

The PCL version used for this build is 1.10. Its `NormalDistributionsTransform::setInputTarget()` immediately builds `target_cells_` using the currently configured resolution. `setResolution()` only rebuilds that target grid when the registration source `input_` is already present. In the old map-load order, target was set while resolution was still PCL's `1.0 m` default, then resolution was changed to `0.8 m` before any source had been set; the actual target covariance grid therefore remained at `1.0 m` even though the configured value reported `0.8 m`.

The minimal repair is in `CurrentFrameNdtRegistration::loadMap()`: set the requested resolution before `setInputTarget()`. The actual target-grid leaf before the repair was `1.0,1.0,1.0 m`; after it is `0.8,0.8,0.8 m`. A test accessor reads the actual PCL target-grid leaf size. `current_frame_ndt_test` asserts all three actual axes equal `0.8 m` (within `1e-7`); the test prints `0.8,0.8,0.8 m`. No PCL source was changed.

Frozen current-frame NDT settings (compile-time defaults in `current_frame_ndt.hpp`):

| Setting | Value |
|---|---:|
| Map voxel then target voxel | `0.15 m`, then `0.15 m` |
| Source voxel | `0.25 m` |
| Source range | `[0.5, 80] m` |
| Maximum source points | `1400` (deterministic evenly spaced cap after voxel filtering) |
| Minimum effective source points | `50` |
| NDT covariance-grid resolution | `0.8 m` |
| Newton step size | `0.08` |
| Transformation epsilon | `1e-5` |
| Maximum iterations | `80` |
| Terminal acceptance | converged, iterations in `[1,79]`, finite terminal transform and fitness |

The PCL covariance target is accessed by its `VoxelGridCovariance` and radius search in the PCL implementation, with search radius equal to the configured resolution; there is no separately selected search-method enum on this path. No NDT-specific thread count or threading override is set. `getFitnessScore()` and transformation probability are recorded diagnostics; neither is used as an additional threshold in this baseline. No score threshold, NDT parameter, source preprocessing parameter, filter noise, or initialization parameter was tuned in this closure.

Read-only comparison caveat: the stable machine-dog workspace is untouched, and was at the requested HEAD `41999ea700c66c4cadf0eca9e0c5d73caa2783fd` when inspected. Its Git worktree was already non-clean at inspection time; no assumption is made that uncommitted files are part of its historical Golden run. Its committed config selects scan midpoint, has `deskew_enable: false`, enables OOSM with `0.02 s` maximum alignment, configures NDT resolution `0.8 m`, and enables a `0.5 m / 5 deg` NDT step limiter. Its NDT initial guess is based on `previous_pose * delta_pose`, with an optional local IMU gyro rotation prior; there is no prediction-age cutoff in the current NDT implementation. That is a different carrier from this paper baseline.

The machine-dog EKF has a real high-rate IMU path and an OOSM rollback/replay path (`src/fusion/ndt_observation.cpp`, `src/core/oosm_replay_planner.cpp`, and `src/core/state_history.cpp`). The planner selects the latest stored snapshot at or before the measurement stamp and accepts it only when the alignment error is at most `20 ms`; the measurement correction is then applied at that snapshot time and IMU samples in `(rollback_stamp, state_now]` are replayed. Thus the OOSM mechanism is present and bounded, but it is discrete-snapshot alignment rather than interpolation to the exact NDT measurement time. This closure did not modify or claim to validate that separate OOSM route. The paper replay avoids that approximation by serializing each transaction and applying the scan-end measurement at the exact propagated scan-end state.

The stable machine-dog `dog_prior_map_ndt_node.cpp::loadMap()` also has the same `setInputTarget()`-then-`setResolution()` order. Given PCL 1.10 behavior above and that its source is attached later in `align()`, this is a likely independent target-grid contract issue there. It was not modified or empirically reconfigured here; historical Golden numbers should not be treated as an exactly matched `0.8 m`-grid comparison until separately measured.

## Floor01 replay and repeatability

Input: the frozen Floor01 prior map (recovered from the released `Floor01/map/floor01.pcd`) and the separately captured runtime-topic test bag; the bag/map hashes are retained below. The replay bundle contains 4,127 filter transactions and 113,354,740 original time-stamped source points. Eleven earlier scans overlap the static IMU initialization interval and are excluded from those transactions; four retained scans have a 16-point partial overlap, which the runtime correctly trims against committed filter time. Every run consumes the same 4,127 transactions, raw point-time clouds, IMU stream, fixed `params.txt`, initial pose, and extrinsic. No GT was passed to the runner. The official Floor01 GT was opened only after all three complete runs had exited.

Full outputs and exact raw replay inputs are archived at:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/paper_baseline_closure_r1_20261003/`

Each `Run1`/`Run2`/`Run3` directory contains trajectory, registration, runtime CSVs, and GNU-time log. `inputs/` contains the raw timed scan index/binary, IMU, filter transaction index, parameter file, and the exporter used. The original ROS bag and frozen PCD remain at their canonical paths and are identified by SHA-256 in the manifest below and in the archived report.

| Run | Frames / updates | NDT SUCCESS | Mean / P95 frame ms | Mean / P95 NDT ms | Peak RSS MiB | Wall s |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 4127 / 4127 | 4127 (100%) | 22.8315 / 42.9515 | 16.2641 / 36.5624 | 108.934 | 94.68 |
| 2 | 4127 / 4127 | 4127 (100%) | 22.9646 / 43.2829 | 16.4032 / 36.6981 | 109.113 | 95.22 |
| 3 | 4127 / 4127 | 4127 (100%) | 22.8342 / 43.1903 | 16.2555 / 36.4228 | 109.035 | 94.67 |

All three trajectories match exactly in every serialized state field and timestamp. All non-timing registration outputs (source hash/count, target count, convergence, status, iteration count, fitness/probability, initial and terminal pose, and update flag) also match exactly. Inter-run corrected-position difference is exactly `0 m` maximum and P95; quaternion components are exactly identical in the CSVs. Runtime varies slightly, as expected, but the estimator/NDT decisions do not diverge.

Every frame is `SUCCESS`; iteration count min/median/P95/max is `1/6/16/61`. Source count ranges from 187 to 1400 and target count is 549,606. No frame was prediction-only; no state became nonfinite; no transaction or postcondition failure occurred. The maximum adjacent corrected translation is `0.22216 m`; maximum raw NDT translation correction from its initial pose is `0.97691 m`. Under the existing Floor01 post-hoc evaluator's sustained `>5 m` divergence criterion, there are zero catastrophic divergence crossings. No new per-frame jump threshold was invented.

### Post-hoc Floor01 accuracy

The already-used `scripts/p6_i6a_report.py` helper was reused, with its fixed Floor01 evaluation start (`1660857393.197807074`) and fixed anchor derived from the historical frozen baseline trajectory. Only the CSV translation-column names were adapted in memory; evaluator alignment, GT interpolation, and error definitions were unchanged. The evaluator yields 4,126 overlapping samples (the initial point precedes the fixed evaluation start). GT SHA-256 is `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`.

| Translation error | Result |
|---|---:|
| Mean | `0.7050 m` |
| RMSE | `0.8694 m` |
| Median | `0.5771 m` |
| P95 | `1.5357 m` |
| Maximum | `1.7633 m` at `1660857528.947763205` |
| Final | `0.0299 m` |

| Rotation error | Result |
|---|---:|
| Mean | `2.1148 deg` |
| RMSE | `2.6035 deg` |
| Median | `1.7330 deg` |
| P95 | `5.7858 deg` |
| Maximum | `11.5899 deg` |
| Final | `0.6564 deg` |

All three runs have the same accuracy values because their serialized trajectories are identical.

## Tests and decision

Passed:

- standalone P7 CTest: `current_frame_ndt_test`, `p7_replay_io_test` (2/2);
- P7-A regressions: propagation contract, scan-end deskew runtime, frontend runtime end-to-end, and registration geometry finite difference;
- 1-frame, 20-frame, and 100-frame smoke runs before the three full replays;
- full Floor01 replay x3 and exact trajectory/NDT parity.

The Floor01 baseline has no remaining timestamp/frame mismatch, deskew reuse, NDT-grid resolution mismatch, non-deterministic initialization branch, or filter-state failure demonstrated by these checks. This is a closure of correctness/repeatability on this frozen input and sequence, not evidence that NDT is globally reliable in every environment.

Unsolved algorithmic failure modes intentionally remain research inputs: local geometric degeneracy, a wrong NDT local minimum/basin, repeated-structure aliasing, a correct basin missing from the initial guess's capture range, and the absence of any global uniqueness guarantee. They were not patched with recovery or threshold heuristics. Floor01 alone does not stress-test all of these failure classes.

Corridor01 was not run. Its existing raw bundle is not directly compatible with the Floor01 replay reader: Corridor records are 40-byte little-endian tuples `(float64 x, y, z, intensity, uint64 absolute_sensor_point_stamp_ns)`, while this reader consumes 16-byte tuples `(float32 x, y, z, uint32 scan_relative_offset_ns)`. The metadata aliases (`byte_offset`, `point_count`) were accepted by the reader, but a one-frame preflight stopped at transaction 0 with `invalid_raw_timed_scan_index_sequence` because the payload record contract differs. Converting absolute point stamps to scan-relative offsets would be a separate data transformation, and its clock/timestamp equivalence has not been established; no converter or Corridor-specific tuning was introduced. This is an optional sanity-check limitation, not a Floor01 baseline closure failure.

### Status and Git

Decision: `BASELINE_STABLE_AFTER_FIX` for the paper workspace's corrected P7-B carrier on Floor01. The fix is limited to ensuring PCL's actual target covariance grid matches the configured `0.8 m`; the replay harness was also changed to reconstruct current-run prediction/deskew from raw time-stamped data rather than reusing a prior run's deskewed XYZ cloud.

Initial checkout before baseline isolation: `research/dual-u-evidence-prototype` at `b65e3e12c372eb9492b64efd51ae236aa65d1bb6`; preserved unchanged.

Selected baseline base commit: `29d1f7def41eced4a197f9dec54f58aec7f1fa32`.

Working branch: `research/paper-baseline-v2`.
Protected machine-dog workspace and prior experiment archives: untouched / retained.

Input SHA-256:

- source bag: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`
- frozen map: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`
- IMU CSV: `faf83b341d6084df5a84e431d61a15f6680a7465d83e9fc7bba13a03961722c2`
- filter scan index: `6cf05c6fb23571969263bb0e011c12adb09dcf73d8707420203459f9d41b1ec0`
- runtime parameters: `84819fc178c70a80de075ba02f5e05938b22879ae4238049f537631e7b121c8b`
- raw timed scan index: `711ee35603a0026f2d04e65100dd8ba068b312e06ab0f992e191e6f55c0b8bd6`
- raw timed point binary: `2c64e78c8b7faaf2b43f0e53fda95198de8472270bd9151acaafd12759bdf809`
