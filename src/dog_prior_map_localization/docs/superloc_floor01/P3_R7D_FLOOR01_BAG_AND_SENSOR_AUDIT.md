# PAPER-P3-R7D: Floor01 bag acquisition and sensor-topic audit

Date: 2026-09-24 (Asia/Shanghai)

This stage acquired and audited the official SuperLoc Floor01 bag. It does not
modify the runtime algorithm, the frozen baseline, the map, GT, calibration, or
any ROS configuration. No adapter, derived bag, localization baseline, pose
evaluator, counterfactual, visual frontend, or P4 work was run.

## Git and protection

- Paper workspace: `/home/jian/livox_ws/dog_loc_paper_ws`
- Branch: `paper`
- HEAD at start: `5d429f3077d0dd64e9725c3669f1a7829d120f95`
- `origin/paper` at start: `5d429f3077d0dd64e9725c3669f1a7829d120f95`
- Frozen baseline remains
  `41999ea700c66c4cadf0eca9e0c5d73caa2783fd` on
  `feature/visual-factor-window`.
- Pre-existing paper dirty/deleted/untracked files were preserved.

Only this documentation file is allowed in the stage commit.

## Official bag acquisition

The official SuperLoc Floor01 Drive ID is
`13QQ8a-dEy56aHg8D0RNW0bfywWz6LKn9`. The HTTP content length and downloaded
size are both `5522264625` bytes. The archive was downloaded once to a
`.part` path, verified, then renamed to:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/raw/SubT_MRS_Hawkins_Multi_Floor_LegRobot.zip`

Integrity:

- file type: Zip archive data
- SHA256:
  `d2d1fd8162e5e7752eded6013874c5e2782201d4b6607dcbfb864852e357124e`
- pre-download free space: approximately 534 GB
- no quota error
- no partial file remains

The archive contains three official ROS bag v2 shards. All three were
extracted and audited; no shard was selected by filename alone:

| shard | size |
|---|---:|
| `raw_data_core_2022-08-18-17-16-31_0.bag` | 3422674923 B |
| `raw_data_core_2022-08-18-17-19-00_1.bag` | 3442739076 B |
| `raw_data_core_2022-08-18-17-21-29_2.bag` | 2783923015 B |

Full `rosbag info` output is retained externally at:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_prerequisite/floor01_rosbag_info.txt`

## ROS bag coverage

The three shards are contiguous in record time:

- combined start: `1660857392.50`
- combined end: `1660857810.02`
- combined duration: `417.52 s`
- total messages: `97498`
- compression: none

Official Floor01 GT covers `1660857393.197807` to
`1660857809.826934` (`416.629127 s`). The bag begins 0.697807 s
before the GT and ends 0.193066 s after it. The complete GT interval is
therefore covered.

No pose interpolation, ATE/RPE, or evaluator was run.

## Topic audit

| topic | type | count | frame | rate |
|---|---|---:|---|---:|
| `/cmu_sp1/velodyne_packets` | `velodyne_msgs/VelodyneScan` | 4139 | `cmu_sp1_velodyne` | 9.9129 Hz |
| `/cmu_sp1/imu/data` | `sensor_msgs/Imu` | 83342 | `epson` | 199.6208 Hz |
| `/cmu_sp1/camera_1/image_raw` | `sensor_msgs/Image` | 10017 | `d` | 23.9987 Hz |

### LiDAR semantics

The LiDAR stream is raw Velodyne packets, not PointCloud2. Each scan contains
76 packets and each packet payload is 1206 bytes. The normal scan period is
approximately 0.10086 s. Scan header and first-packet timestamps are exactly
equal for all audited scans (maximum difference 0 s), supporting scan-start
semantics for the header.

The packet message does not expose per-point `time`, `t`, or
`timestamp` fields. Point-level firing timing therefore requires a
Velodyne packet decoder. The correct classification is:

```text
RAW_PACKET_REQUIRES_DECODER
```

### IMU semantics

The IMU stream is `sensor_msgs/Imu`, frame `epson`, with orientation,
nonzero orientation covariance, angular velocity, and linear acceleration.
Timestamp dt mean/median/P95 are 0.005009498 / 0.004992008 / 0.005007982 s.
The stream covers the full LiDAR and GT interval.

### Camera semantics

The camera stream is `sensor_msgs/Image`, frame `d`, 640 x 480,
`bgr8`, 23.9987 Hz. Images were not decoded and no visual frontend was
enabled.

## Calibration and initial pose

The bag modalities map to the already audited official Floor01 calibration:

- `cmu_sp1_velodyne` -> `laser_to_imu`
- `epson` -> IMU target of the official calibration
- `d` -> `rgb_camera_to_imu` and `rgb_camera` intrinsics

The numerical calibration matrices remain identical to the official SubT-MRS
values. The convention remains:

`p_imu = T_imu_lidar * p_lidar`

The bag start versus GT first timestamp relation is recorded above. The
`multi-floor01.yaml` initial pose has no timestamp lineage, and its direct
`world_darpa` matrix direction remains `PARTIAL`; no new direction
claim is made in this stage.

## Adapter readiness

The official bag, GT lineage, map, calibration, and IMU coverage are ready.
However, the raw packet input still lacks validated point-time semantics.
Therefore the only allowed readiness classification is:

```text
B — LIDAR_DECODER_REQUIRED
```

```text
FLOOR01_READY_FOR_ADAPTER_STAGE = NO
```

The next stage must explicitly select and validate a Velodyne packet decoder
and its point-time convention before creating a FULL-SE3 deskew adapter.
