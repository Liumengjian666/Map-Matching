# PAPER-P3-R7C: SuperLoc Floor01 prerequisite and lineage audit

Date: 2026-09-24 (Asia/Shanghai)

This document records an official-asset and lineage audit for replacing the
blocked Corridor02 sequence with SuperLoc `Floor01`. It does not modify the
runtime algorithm, the frozen baseline, any adapter, or any evaluation code.

## Scope and protection

- Paper workspace: `/home/jian/livox_ws/dog_loc_paper_ws`
- Frozen baseline workspace: `/home/jian/livox_ws/dog_visual_loc_ws`
- Frozen baseline commit: `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`
- Paper branch at audit start: `paper`
- Paper commit at audit start: `0b6b0faa2e20335a6ba2ccb16a216211c275370b`
- Corridor02 P3-R7B remains `PARTIAL/PARKED`; its external GT-origin and bag
  blockers are not inherited or silently resolved here.

Only this documentation file is allowed to be committed for this stage.
Existing deleted tracked documents, `.vscode/`, and untracked P3-R1-V2 reports
were preserved unchanged.

## Official candidate sequence

The official SuperLoc page lists:

| field | value |
|---|---|
| sequence | `Floor01` |
| source | `SubT-MRS` |
| location | `Hawkins` |
| platform | `SP1` |
| sensors | RGB, LiDAR, IMU |
| advertised trajectory | 270 m |
| advertised duration | 480 s |

The corresponding official links resolve to:

- bag: Drive ID `13QQ8a-dEy56aHg8D0RNW0bfywWz6LKn9`
- extrinsics: `1FAtf5IkUzNNrwxyqVAV6Vre5Pmvbp8Mj`
- intrinsics: `1uH4wFmLeQNrIGlsUsO--PQuyEIOSOGvR`
- map: `1F46g0wnJVSedTJubFD_Ne1IwgZAGYvsU`
- SuperLoc GT: `1T-p9TgDwD_9us7U94cT0guwPNTvx3KkI`
- initial-pose folder: `1WZsyEYyU-_8ps1CUqRq3YO7IaARMdGNm`
- resolved initial pose: `multi-floor01.yaml`, ID
  `1lDx6G9O4eOGwWMxDNGJP59z9z4KrmcZ0`

The audit used only official SuperLoc, SubT-MRS/ICCV challenge, and official
code/config/paper sources. No third-party asset was used.

## SuperLoc and official SubT-MRS GT lineage

The official SubT-MRS challenge package is
`SubT_MRS_Hawkins_Multi_Floor_LegRobot.zip` (Drive ID
`1mxxJDL9mEA0kDFkGOZZfTCB0A0fOEYBS`) from the official
`Groundtruth_Trajectory / Multi-Modal-Sensor-Fusion-Track` folder. Its
`ground_truth_path.csv` was compared with the downloaded SuperLoc
`floor01_gt.txt` after converting CSV nanosecond timestamps to seconds.

| check | result |
|---|---:|
| SuperLoc samples | 2061 |
| official SubT samples | 2061 |
| matched timestamps | 2061 / 2061 |
| exact timestamp matches | 2061 / 2061 |
| timestamp delta mean/P95/max | 0 / 0 / 0 s |
| translation difference mean/P95/max | 0 / 0 / 0 m |
| SO(3) difference mean/P95/max | 0 / 0 / 0 deg |
| quaternion absolute-dot minimum | 0.999998634161 |

Both files start at `1660857393.197807` and end at `1660857809.826934`, for
an actual GT span of `416.629127 s`. The 480 s value is the official page
metadata and is not substituted for the measured file span.

Classification:

```text
IDENTICAL_POSE_SEQUENCE
```

This is exact pose-sequence identity, not a filename-based assumption.

## GT frame/origin semantics

The official ICCV challenge evaluation text defines the trajectory in the IMU
coordinate frame: `tx ty tz` are IMU position in world, `qx qy qz qw` is IMU
orientation with respect to world, and the IMU axes are x-forward, y-left,
z-up. Therefore this audit accepts:

```text
GT_POSE_ORIGIN = IMU
evidence = OFFICIAL_LINEAGE
official lineage valid = YES
```

The SuperLoc TUM file and the official SubT CSV have the same 2061 poses, so
the frame statement is attached to the exact same sequence rather than inferred
from a filename.

## Map and calibration

Downloaded official assets are present under:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/`

| asset | size | SHA256 |
|---|---:|---|
| `gt/floor01_gt.txt` | 178532 B | `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f` |
| `map/floor01.pcd` | 19791028 B | `bad163fdcada69b975159365dfd776befde542cd08cdd0a2ce1f702a3d0aaf5d` |
| `calibration/floor01_extrinsics.yaml` | 919 B | `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414` |
| `calibration/floor01_intrinsics.yaml` | 436 B | `04c5865d2d1f3864c8b1a5f2348b8455d400394be94c58313e8d3323b446b059` |
| `initial_pose/multi-floor01.yaml` | 1144 B | `6da3daf5194b30e27c2b07956738798b4e39d5b2e613c71206da683961737c98` |

Map audit: binary PCD, `549637` finite points, no NaN/Inf; bbox is
`x=[-57.656250,206.399597]`, `y=[-227.619858,-14.829542]`,
`z=[-8.916914,24.794086]`. The initial-pose translation
`[39.972755,-175.753494,-5.125434]` lies inside this extent. The raw GT is a
local trajectory near the origin; no unverified absolute transform is claimed.

Calibration audit: `laser_to_imu` matches the official SubT calibration
elementwise (max difference 0), as does `rgb_camera_to_imu`. Its rotation
determinant is `0.999999999558`, orthogonality residual is `1.43458e-09`, and
inverse round-trip residual is `1.11935e-16`. The released calibration uses
the `imu^R_laser`, `imu^T_laser` convention also used by the audited official
SuperOdom source.

## Initial-pose semantics

The audited official SuperOdom source assigns `init_x/y/z/roll/pitch/yaw` to
`T_w_lidar`, uses the same state for the map origin, and publishes
`/laser_odometry`. Thus the runtime startup state expected by that source is
a LiDAR pose (`T_map_lidar`).

The Floor01 initial-pose file is an OpenCV-style 4x4 matrix named
`world_darpa`, not the `start_pose.txt` format parsed by that source. No
official converter or explicit direction statement for this YAML was found.
Consequently its direct YAML lineage status remains `PARTIAL`; the in-map
translation is consistency support only, not proof of matrix direction.

Operationally, the runtime state semantic is understood well enough for a
future startup configuration, but this caveat must remain visible in the next
stage.

## Bag availability and gate

The official SuperLoc Floor01 bag link responds to a valid HTTP HEAD
(`application/octet-stream`, reported size `5,522,264,625` bytes), but the
large bag was intentionally not downloaded in this prerequisite stage. No
`.part` file was used and no `rosbag info` was run.

The official challenge rosbag folder listing did not contain a separate
`Multi_Floor` ROS bag. The acquired 1.24 MB SubT archive is a GT package, not a
ROS bag. Therefore no official SubT bag is substituted for the SuperLoc bag.

```text
lineage ready: YES
GT semantics ready: YES
map ready: YES
calibration ready: YES
initialization semantics ready: YES (operationally; direct YAML lineage PARTIAL)
bag ready: NO
FLOOR01_READY_FOR_P3_R7_VALIDATION: NO
```

The sole blocking prerequisite for an end-to-end P3-R7 validation is a valid
acquired/validated official Floor01 rosbag. The initial-pose direction caveat
must be resolved or explicitly bounded before publication, but it is not
silently promoted to a proven fact here.

## Scientific and algorithm status

- No adapter, baseline, counterfactual replay, relative evaluator, or P4
  implementation was started.
- No NDT, EKF, IMU, deskew, map, bag, or runtime configuration was modified.
- P3-R7B Corridor02 remains `PARTIAL/PARKED` with its external blocker.
- No initialization-dependence, cross-sequence replication, or P4 conclusion
  is claimed from this audit.

The next allowed action is to acquire the official Floor01 bag (with its
checksum and `rosbag info`) and then perform the separately specified P3-R7
validation. Until then the gate remains closed.
