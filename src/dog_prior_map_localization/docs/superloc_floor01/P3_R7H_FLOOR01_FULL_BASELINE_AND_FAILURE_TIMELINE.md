# PAPER-P3-R7H — Floor01 full frozen baseline and failure timeline

## Scope

This stage executes the full, causal, baseline-conditioned Floor01 sequence and answers only whether the frozen closed-loop NDT pipeline is deterministic and how its first-pair-relative error evolves. It does not diagnose the physical cause, test initialization dependence, or change any runtime behavior.

The frozen localization source is the paper branch at `d131bc73d06500efe42afbdf554fb4ff80ff1eae`, derived from baseline `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`. NDT source SHA-256 matches the frozen baseline in both workspaces (`40cdf26f9dddb9d06dc16c0bfd89fc88a9dd33e42e5bbd4e9bc3b453f1e4277c`). No baseline or paper runtime source/include changes were made.

The frozen baseline workspace already contained 61 non-clean Git status entries when this stage began; the same count remains, and none was staged, overwritten, or otherwise touched by this stage. The paper workspace's pre-existing dirty/deleted files were also preserved; only this report is intended for the stage commit.

## Input and causal execution

The canonical input contains 4139 timestamp-merged VLP16 scans and 83342 IMU messages. Duplicate timestamps, duplicate packet hashes, within-shard ordering violations, shard overlaps, and header/first-packet mismatches are zero. Runtime starts at stamp `1660857392.592703104` after the two earlier scans lacking initial IMU coverage, and contains 4137 scans through `1660857809.826934099`. One runtime scan at index 1056 (`1660857499.094856024`) is rejected for strict IMU coverage gap; it is not extrapolated or sent to NDT.

The adapter uses scan 0/1 zero-translation bootstrap and thereafter two most recent completed prior NDT poses for causal constant-velocity translation, plus IMU gyro rotation deskew. Each next scan is released by explicit per-scan completion handshakes. Current-scan and future NDT poses are not deskew inputs. Both runs yielded 4136 NDT outputs, 2 bootstrap scans, 4134 CV scans, and 4136/4136 converged NDT outputs. Strict timestamp causality checks found zero violations. Shard boundaries at runtime indices 1467 and 2943 retained CV history without reset.

The retained bags contain all required per-scan decoded-cloud, deskewed-cloud, motion-diagnostic, NDT-odometry, NDT-pose, and NDT-diagnostic messages (4136 each). The optional filtered-cloud visualization topic has 4134 recorded messages; the two-message difference was not independently resolved and is not treated as an NDT output loss.

## Determinism gate

Run A and B used the same canonical manifest, normalized map, H1 operational initialization, NDT binary, and effective parameters. Exact scan order, packet identity, decoded/deskewed hashes, history stamps, mode and velocity all match. Initial/raw/final pose maximum translation difference is 0 m; maximum rotation difference is `2.41483654e-6 deg`; fitness difference is 0; iterations, convergence, and limiter flags match. No first branch divergence was found. The result passes the specified 1e-6 m / 1e-4 deg / 1e-6 fitness gates. GT was read only after this gate passed.

## Relative GT evaluation

The P3-R3B evaluator implementation and its synthetic checks were reused unchanged. All seven synthetic tests passed. GT has official IMU origin; the first eligible NDT final-used stamp `1660857393.197807` (runtime scan index 6) is the first-pair anchor. Position interpolation is linear, orientation interpolation is SLERP, extrapolation and additional time-offset fitting are prohibited. LiDAR poses are converted to IMU poses by `T_map_imu = T_map_lidar * T_lidar_imu`. Initial guess and raw-NDT stages use the same final-used anchor.

There are 4130 GT-supported samples; six early NDT samples precede GT support and none are extrapolated. Maximum GT interpolation bracket width is 0.403440 s, with nearest-sample difference P95/max of 0.100860/0.201720 s.

| Final-used relative metric | Translation | Rotation |
|---|---:|---:|
| Mean | 31.8458 m | 62.7143 deg |
| RMSE | 41.9112 m | 80.4643 deg |
| Median | 32.2549 m | 67.4243 deg |
| P95 | 64.4142 m | 138.7361 deg |
| Maximum | 68.1084 m | 178.7963 deg |

## Failure timeline

Crossings use the existing P3-R3B helper: strict `error > threshold`, continuity gap at most 0.25 s, persistence at least 5 s. This preserves the audited evaluator semantics rather than changing the threshold operator.

| Threshold | Instantaneous first crossing (relative s) | Persistent crossing (relative s) | Persistent crossing stamp |
|---:|---:|---:|---:|
| 0.25 m | 21.280282 | 38.223808 | 1660857431.421614885 |
| 0.50 m | 32.273406 | 112.351752 | 1660857505.549559116 |
| 1.00 m | 32.374265 | 139.078133 | 1660857532.275939941 |
| 2.00 m | 139.481573 | 139.481573 | 1660857532.679379940 |
| 5.00 m | 140.590976 | 140.590976 | 1660857533.788783073 |

Thus a persistent 0.5 m failure window exists and has 304.277375 s of valid frames after its onset. The first instantaneous 0.5 m excursion is transient under the specified persistence rule. Full per-frame timing and error data are kept outside Git under `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/`.

## Descriptive NDT/motion diagnostics

Across 4136 NDT outputs, fitness median/P95/max is 0.6098/21.7759/631.9384, and iteration median/P95/max is 1/11/40; convergence is 4136/4136. Translation limiter activation is 237 frames, rotation limiter 773, and either limiter 845. Transformation probability is unavailable because the frozen NDT implementation does not expose or record `getTransformationProbability`; it is reported as `NA`.

Adapter velocity norm mean/median/P95/max is 1.4575/1.1212/4.9060/5.0337 m/s. Translation per scan mean/P95/max is 0.1436/0.4713/0.5066 m; rotation per scan mean/P95/max is 0.03009/0.10144/0.21912 rad. These are descriptive inputs, not a reliability classifier or root-cause proof.

## Decision

`FULL_RUN_CLASSIFICATION = A: DETERMINISTIC_COMPARABLE_FAILURE_FOUND`.

This stage establishes repeatability and a sustained relative-error window; it does not establish physical root cause or initialization dependence. H1 remains `OPERATIONAL_INIT_SELECTED_BY_MAP_CONSISTENCY`; the released YAML matrix’s official semantic direction is still unconfirmed. The task-defined next review is P3-R7I counterfactual replication, not executed here. `P4_ALLOWED = NO`.

Raw bags, CSVs, logs, and plots are external artifacts and are intentionally not committed. The only file intended for this stage’s commit is this report.
