# P10 Corridor01 prospective causal baseline R1

This is a new, reproducible input/runtime protocol. It does not claim byte- or
field-level reproduction of the unavailable historical P2B v1 runtime.

## Frozen data window

- Source: the already frozen `P9_CORRIDOR01_RAW_SCANEND_V1` sidecar; no raw bag
  re-extraction was performed.
- Source manifest SHA256: `591bfe3fd619966f40e4e2af6b9151937732483f0123c70eb34aa1031742991a`.
- Source point binary SHA256:
  `ce8beed2303ea3de8715932c4d0644d6fe28317014e2383f65942d69bc5ace32`.
- Source IMU CSV SHA256:
  `21b94c0cb0ade931db1586799aa022dff2ba20b9ab76dc1e29eac0e948e132e6`.
- Source scan index SHA256:
  `d49b7b81bb1f5c17eb2da9e5ad665dfb2b3b4813280de4e342eb3158d13c932a`.
- Start: transaction 2, sensor scan start
  `1517157219188978910 ns`; transaction 1 is excluded because its required
  leading IMU coverage is absent.
- Startup: first 5.0 s. Evaluation: next 35.0 s. The fixed slice contains 396
  scans, 346 evaluation scans, 7,989 IMU messages (including the actual sample
  immediately before the start), and 11,473,279 raw timed points.
- Bag record time equals original sensor header time. Point times are the
  preserved raw offsets expressed as float32 seconds from scan start; no
  scan-start/full-SE3 historical adapter, future odometry, or GT deskew is used.

The immutable slice bag SHA256 is
`f6e77c96d97402e551835e81a1ce8d780d075da2f18a9cb5e4fc0646628b86d6`.
Its full generated manifest and ledgers are kept with the persistent local
experiment output under
`/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/input/`.

## Runtime and frame contract

The intended chain is the existing split P2B-style ROS runtime, not a claim of
the historical P2B v1 implementation:

1. The EKF consumes original Epson IMU measurements at their sensor stamps.
2. Its existing IMU propagation and `ekf_imu_fastlio` deskewer use raw point
   time and the official `T_imu_lidar` direction (LiDAR coordinates to IMU).
3. The existing PCL NDT node matches the scan-start deskewed LiDAR cloud against
   the normalized Corridor01 prior map. Its local-gyro initial-guess prior uses
   `omega_lidar = R_imu_lidar^T * omega_imu`; the EKF continues to consume the
   unrotated Epson IMU stream.
4. The NDT LiDAR pose is converted to the IMU pose by the existing frame
   conversion and applied through the existing OOSM measurement path; subsequent
   propagation consumes the corrected state.

The map is the normalized map, SHA256
`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`. The
official extrinsic is SHA256
`59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d`. The
sensor-only first-segment map anchor is
`T_map_lidar = I` at the normalized-map origin; it is not GT and is not asserted
to be an instantaneous motion estimate at later transactions. The filter is
initialized at transaction 2's scan-start epoch with
`T_map_imu = T_map_lidar * inverse(T_imu_lidar)`. Initial velocity and both
biases are disclosed zero-mean priors with the existing configured covariance,
not claimed estimates. The gravity vector is transformed from the source map's
inferred +Z-down convention using the sensor-only initializer rotation; that
coordinate convention remains an explicit inference, not official map-to-world
closure.

NDT settings are fixed to the existing Corridor01 profile: resolution 0.8 m,
step 0.08 m, epsilon 0.001, 40 iterations, source voxel 0.25 m, target/map voxel
0.15 m, and local IMU-rotation prior enabled. The existing EKF uses real
accelerometer propagation, midpoint IMU intervals, OOSM feedback, and the
existing noise/correction settings. No R6 algorithm or feedback branch was
run in this stage.

## Evaluation freeze

GT must remain unloaded until the runtime output and input/config hashes are
frozen. If a Control output becomes available, the predetermined primary
relative-drift comparison is one fixed full-pose SE(3) alignment estimated
from the first 3 s of the 35 s evaluation interval, then held unchanged for all
methods. Primary errors are reported on the subsequent 32 s; the 3 s fit prefix
is reported separately. A position-only Kabsch alignment is a diagnostic for
long-corridor degeneracy, not a selectable alternative primary result. No
absolute map-to-GT transform is currently closed.
