# P10-CORRIDOR01-PROSPECTIVE-CAUSAL-BASELINE — execution report

## Result

`CONTROL_RUN_BLOCKED_LOCAL_ROS_SOCKET_PERMISSION`

The new prospective 40-second input was generated once from the previously
verified P9 raw timed sidecar, hash-checked, and inspected with `rosbag info`.
Both runtime nodes compile against the real package source. The localization
experiment itself did **not** start: ROS 1 `roscore` failed before creating its
XML-RPC server with `PermissionError: [Errno 1] Operation not permitted` at
Python's `socket.socket(...)`. Retrying with `ROS_IP=127.0.0.1` and
`ROS_HOSTNAME=localhost` produced the same failure. No NDT node, EKF node, bag
player, or recording node ran.

This is an execution-environment blocker, not a failure of Corridor01
localization, initialization quality, or Coupled NDT. It is also independent of
the unrecoverable historical P2B v1 dependency lineage.

## Work actually completed

- Revalidated the frozen P9 source manifest and all four source input hashes.
- Generated a new input protocol slice, without reopening or re-extracting the
  original raw bag: 396 scans, 346 evaluation scans, 7,989 IMU samples,
  11,473,279 raw timed points. The true pre-start IMU boundary sample is retained.
- Verified bag index, duration (39.962544 s), message counts, topic types, and
  frozen output SHA256.
- The slice manifest's runtime-source snapshot records source/config hashes at
  packaging time. After that input-only packaging step, the source review found
  the NDT gyro-frame issue below; final runtime code/config hashes are recorded
  separately in `artifact_hashes.json` and `input_manifest.json`. The bag and
  all source sensor data remain unchanged.
- Added a default-off explicit-state option to the existing EKF node. It checks
  pose rotation, finite vectors, gravity norm, state/boundary timestamps, and
  requires the existing IMU-deskew/OOSM/NDT pipeline. Existing default startup
  behavior is unchanged.
- During review, found that the NDT node's optional local-gyro initial-guess
  prior consumes angular velocity in the LiDAR frame, while the new input keeps
  Epson-frame IMU for the EKF. Added a default-off NDT adapter option to rotate
  only gyro samples by `R_imu_lidar^T` before that prior; the new Corridor01
  overlay enables it. The EKF still receives raw Epson gyro/acceleration.
- Added a one-shot slice packer that hashes its source data, runtime config,
  map, extrinsic, initialization artifact, and relevant source files before
  recording those hashes in the slice manifest.
- Built the split NDT and EKF nodes in `/tmp/p10_corridor01_causal_build`.

## Runtime metrics

All localization runtime values below are `NOT_RUN`, not zero-valued scientific
measurements:

| Item | Result |
|---|---:|
| NDT calls | NOT_RUN |
| EKF state updates | NOT_RUN |
| localization frames | NOT_RUN |
| Control first-10-second GT error | NOT_RUN; GT was not loaded |
| drift onset / RMSE / P95 / max | NOT_RUN |
| runtime mean / P95 / max | NOT_RUN |
| localization peak RSS | NOT_RUN |
| Nominal / Weak-only / Coupled | NOT_RUN |

The packer itself completed in 6.30 s wall time with 303,536 KiB maximum RSS.
That is conversion cost, not localization cost.

## Build and test

The first build attempt lacked the `livox_ros_driver2` package in the sourced
prefix. Sourcing the existing read-only `/home/jian/livox_ws/devel` dependency
prefix fixed package discovery; no package was installed or system state
changed. The optimized package build then completed. The following existing contract
tests passed:

```text
FRAME_CONVERSION_AND_TWIST_CONTRACT_PASS
gyro_imu_to_lidar,...,PASS
IMU_DESKEW_CONTRACT_PASS
IMU_INTERVAL_NUMERIC_TEST_PASS
OOSM_REPLAY_PLANNER_CONTRACT_PASS
NDT_PROTOCOL_CONTRACT_PASS
```

The full build command and ROS launch/replay command are in
`control_run_command.txt`. These tests validate existing components and
compilation; they do not substitute for a real-data localization run.

## Required next action

Run the frozen Control in an execution environment that permits local ROS 1
TCP/XML-RPC sockets. Keep this exact slice, configuration, map, extrinsic,
initialization priors, NDT parameters, and evaluation rule. Do not start
Weak-only/Coupled comparisons until Control timing, coordinates, OOSM feedback,
and early evaluation are verified. No GT was opened, and no R6 method was run.
