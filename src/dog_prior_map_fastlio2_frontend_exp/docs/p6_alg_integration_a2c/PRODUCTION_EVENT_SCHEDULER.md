# Deterministic causal producer scheduler

Events use sensor header stamps from prepared inputs. Sorting is lexicographic
by `(stamp,type,source_index)`, with type order:

1. LIDAR_SCAN
2. VISUAL_CURRENT
3. VISUAL_REFERENCE

This lets a same-stamp LiDAR risk inform the visual current event; that image
can then become the next pair's reference. No scan-stamp snapping or bag-record
timestamp replacement is performed.

Before each event, append IMU through exactly the first sample with stamp >=
event stamp. The cursor is persistent, so a previously buffered right boundary
is reused when applicable. Do not prefill future seconds of IMU. Missing right
boundary stops the runner with `event_imu_right_boundary_missing`.

`prepareStateAt` creates only a predicted state and its IMU factor. It does not
admit LiDAR/visual factors. The producer calls it before NDT and before taking
the pre-measurement covariance. Visual reference/current nodes retain their
true asynchronous times. Optimization follows every event; an unsuccessful
optimizer stops rather than exporting an unverified state.

Every terminal LiDAR event must have stamp >= last consumed LiDAR stamp,
including nonconverged/map-insufficient/NIS-rejected cases. Regression is
`REJECTED_CAUSALITY/lidar_timestamp_regression` and cannot advance transaction
watermark or risk history. Tests prove the same transaction remains admissible
after two regressed invalid terminal attempts.

## Explicit experimental runner

```
p6_i6b_closed_loop FULL_FIXED_LAG_V2_EXPERIMENTAL \
  imu.csv filter_scans.csv scans.csv request_xyz_f32.bin \
  map.pcd params.txt trajectory.csv events.csv runtime.csv \
  visual.csv frame_limit initialization_stamp_ns floor01|corridor01 r2_policy
```

This command is documented, not executed on public datasets in A2C. Map identity
is checked using existing frozen hashes; Floor01 also requires frozen source
hash metadata and checks actual preprocessed clouds. The Corridor prepared
bundle retains its existing upstream identity convention; no new derived input
or ROS adapter is created. Source preprocessing/NDT are reused from the mature
runner, with no repeated deskew of prepared clouds.

Trajectory `px..qw` represents **map_T_imu** at scan timestamps, not map_T_lidar.
Predicted pose diagnostics also use map_T_imu; the actual NDT guess multiplies
it by T_imu_lidar. Window position/rotation sigma maxima are square roots of the
largest eigenvalues of their respective P15 3x3 blocks.

Tests cover same-stamp sorting, asynchronous pairs and exact first-right IMU
boundary in all fixture event rows. No formal trajectory is generated here.
