# PAPER-P3-R10B-FIX1 full rerun

## Scope

This stage removes the fatal response to ordinary partial LiDAR scan-window overlap. It follows the pinned FAST-LIO reference behavior: the prior scan-end state is the temporal lower bound, points strictly before it are discarded, and a wholly stale scan is skipped without advancing the filter or opening an NDT transaction. NDT configuration, map, IMU propagation, deskew, and measurement update were not changed.

Reference: hku-mars/FAST_LIO, commit 7cc4175de6f8ba2edf34bab02a42195b141027e9, src/IMU_Processing.hpp, ImuProcess::UndistortPcl.

## Code and validation

The implementation is confined to:

- include/dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp
- src/scan_processor.cpp
- src/fastlio2_frontend_node.cpp
- tests/frontend_runtime_end_to_end_test.cpp

The test exercises a 35 us partial overlap, drops the two synthetic pre-commit points, commits the scan at its true end, continues into the next scan, and confirms a fully stale scan does not advance the runtime.

Experimental ON Release build and focused contracts passed:

- IKFOM_POSE_API_SPIKE_PASS
- OVERLAP_BOUNDARY_CONTRACT_PASS
- FRONTEND_RUNTIME_END_TO_END_PASS
- SCAN_END_DESKEW_RUNTIME_CONTRACT_PASS
- FASTLIO2_PROPAGATION_CONTRACT_PASS
- NDT protocol D4 contract suite PASS (A-L, M1-M6, LIM1-LIM6, Q1-Q7, G1-G8)

Default OFF Release build and regression contracts passed:

- OOSM_REPLAY_PLANNER_CONTRACT_PASS
- IMU_DESKEW_CONTRACT_PASS
- IMU_INTERVAL_NUMERIC_TEST_PASS
- FRAME_CONVERSION_AND_TWIST_CONTRACT_PASS

The task-specific Catkin test executables were invoked directly; ctest itself reported no registered tests in these isolated Catkin builds.

## Input and run provenance

- Workspace baseline before edits: cbf9ec3ee2296e12233837b18513ad15d9f93e26
- Raw Floor01 bag: /media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag
- Raw bag SHA-256: 6383fdbdad0e3f8375069b33cc552ed069dafb91aec88f1673bc043e0f314e0b
- Raw bag duration: 417.621513 s
- Map: /tmp/floor01_candidates/floor01_h1_map.pcd
- Map SHA-256: 2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570
- Runtime config: config/floor01_superloc_smoke.yaml
- Config SHA-256: 4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77
- Extrinsic SHA-256: fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414
- Initial-pose file SHA-256: 6da3daf5194b30e27c2b07956738798b4e39d5b2e613c71206da683961737c98
- Captured runtime topic bag: /media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag
- Runtime topic bag SHA-256: 860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db
- Topic bag duration: 417.554945 s

The full graph received 4139 source VelodyneScan messages and emitted 4138 PointCloud2 scans. The first packet-message stamp had no matching emitted cloud; the decoder began output at the next packet boundary. Of the 4138 emitted scans, 11 were intentionally skipped while the static IMU initialization window was established; 4127 were processed. Requests, results, acknowledgments, and corrected odometry each contain 4127 entries; transaction IDs are 1 through 4127.

The decoder's point-cloud conversion sources and VLP16 calibration were byte-identical to the pinned ros-drivers/velodyne 1.7.0 source/calibration. The exact pinned checkout's unrelated driver target could not build because pcap.h was absent, so the running local decoder binary came from checkout 29abd0e1361cb7f5eda451d2b51c35eeca45e0d5 (1.7.0-4-g29abd).

## Scan-window statistics from the full replay

Timing was recomputed from the 4138 captured PointCloud2 messages and their VLP16 per-point time field.

| Quantity | Result |
|---|---:|
| Decoded scans / scan pairs | 4138 / 4137 |
| Partial-overlap pairs | 4 |
| Overlap duration mean / P95 / max | 37.625500 / 43.061150 / 44.420000 us |
| Positive-gap pairs | 4133 |
| Gap mean / P95 / max | 92.284167 / 22.978000 / 100882.858000 us |
| Expected overlap-prefix points dropped | 64 (16 per overlap) |
| Scan duration mean / P95 / max | 100.786532 / 100.839451 / 100.839451 ms |

Full-replay window CSV and markdown are retained beside the run outputs as floor01_scan_window_stats_full_replay.csv and floor01_scan_window_stats_full_replay.md.

## Full-run closure

- Processed transactions: 4127
- Initial static-init skips: 11
- Whole stale scans skipped: 0
- Partial-overlap scans: 4
- Overlap points discarded: 64
- NDT SUCCESS / rejects: 4127 / 0
- Prediction-only commits: 0
- IKFoM measurement updates: 4127
- Sticky fatal / NaN or Inf log tokens: 0 / 0
- Last committed scan-end: 1660857809.927773312
- All recorded request/result/ack/odometry counts match, and the final runtime counter reached tx=4127.

The filter completed the recorded input duration and the transaction worker shut down after the playback. The last state timestamp lies 0.100839 s after the GT endpoint; it is excluded from accuracy statistics without extrapolation.

Resource monitoring was not retained as a durable time series. The live observations were approximately 1.9 GiB peak NDT RSS late in the run and 38 MiB frontend RSS. CPU mean/peak is unavailable and is intentionally not inferred.

## Relative accuracy

The existing evaluate_p3_r3_reference.py evaluator and official-lineage IMU-origin reference were used. Estimated LiDAR poses were converted to IMU origin using the supplied calibration. Relative-from-first-in-GT-coverage metrics use 4126 in-coverage scan-end poses, no extrapolation, and the configured evaluation origin 1660857393.197807. The first in-coverage pose is 0.504279 s after that configured origin. The evaluator JSON has a legacy hard-coded scan-start description; the actual input CSV timestamps were scan-end timestamps, matching the pose/state publication time.

Values are translation metres and rotation degrees.

| Trajectory | Quantity | Mean | RMSE | Median | P95 | Max |
|---|---|---:|---:|---:|---:|---:|
| Predictor | Translation | 14.808157 | 21.966885 | 6.147184 | 46.744637 | 54.296020 |
| Predictor | Rotation | 7.683759 | 9.901068 | 5.209660 | 19.083755 | 22.538754 |
| Raw NDT | Translation | 14.650152 | 21.925248 | 6.028596 | 46.737643 | 54.350925 |
| Raw NDT | Rotation | 8.301130 | 10.826150 | 7.107061 | 20.505381 | 67.847703 |
| NDT used | Translation | 14.644633 | 21.915813 | 6.031024 | 46.767350 | 54.350925 |
| NDT used | Rotation | 8.862746 | 11.392567 | 8.060236 | 21.443513 | 41.068855 |
| Corrected | Translation | 14.660182 | 21.936133 | 6.034344 | 46.757084 | 54.298935 |
| Corrected | Rotation | 7.602126 | 9.906078 | 5.256542 | 19.089644 | 22.682864 |

### NDT-used error minus predictor error

Negative delta means lower error after NDT.

- Translation: 3275 improved, 851 worsened; mean delta -0.163524 m; median -0.124776 m.
- Rotation: 2146 improved, 1979 worsened, 1 unchanged; mean delta +1.178987 deg; median -0.039757 deg.

### Corrected translation threshold crossings

Crossing times are on the configured evaluation-origin axis. Persistent crossing is an above-threshold segment lasting at least 5 s, with a maximum adjacent-sample gap of 0.25 s.

| Threshold | First crossing (s) | Persistent crossing start (s) | Persistent duration (s) |
|---:|---:|---:|---:|
| 0.25 m | 1.714542 | 28.037489 | 16.035842 |
| 0.5 m | 62.327979 | 84.919328 | 331.709778 |
| 1 m | 93.592839 | 93.592839 | 323.036268 |
| 2 m | 141.801217 | 151.483213 | 265.145893 |
| 5 m | 157.433614 | 157.433614 | 259.699792 |

The first-936-frame figures from the previous partial trial remain prefix-only and are not extrapolated: Predictor translation RMSE 0.5463 m, NDT-used 0.3970 m, Corrected 0.4090 m; NDT-used improved/worsened 871/65 frames and mean error delta -0.126786 m.

## Observational classification

PREDICTOR_DOMINATED. The long-horizon trajectory error is already present in the predictor (21.966885 m translation RMSE); NDT-used is only 0.051072 m lower, while corrected is 0.030752 m lower than predictor and remains close to it. NDT-used rotation is worse than predictor by 1.178987 deg mean error, while corrected rotation is close to predictor. This is a full-run comparative classification, not causal proof of the physical reason the predictor drifts.

## Scope and publication

No NDT, EKF, map, deskew, calibration, initial-pose, or runtime algorithm outside scan-window overlap handling was modified. Generated bags and large data remain outside Git. This concise report is intended for version control; full-resolution evaluation CSV/JSON/plots remain in the external results directory.
