# PAPER-P1-R2 asset status

Checked on 2026-09-22 (Asia/Shanghai).  This document records asset
completion only; no localization, adapter, benchmark, or algorithm change was
performed.

## Storage

- Volume: `/media/jian/HIKVISION` (`/dev/sda2`, exFAT, label `HIKVISION`).
- Capacity at check: 953.9G total, about 400G used, about 554G available.
- Required read/list checks: PASS.  The installed `lsblk` does not recognize
  the requested `MOUNTPOINTS` column, so the equivalent `MOUNTPOINT` column was
  used after recording that tool limitation.
- Write test (`.paper_write_test` create/sync/list/remove/sync): PASS.
- I/O errors/timeouts: none observed.
- Former incomplete NTNU file was preserved as
  `/media/jian/HIKVISION/paper rosbag/NTNU_LiDAR_Degeneracy/tunnel.bag.partial`
  (0 bytes); it is not valid data and was not deleted.

## Dataset catalogs

- Dataset catalog: `/media/jian/HIKVISION/paper rosbag/DATASET_CATALOG.md`
  (created in this stage).
- Algorithm catalog:
  `/media/jian/HIKVISION/comparison algorithm/ALGORITHM_CATALOG.md`
  (created in this stage).
- The catalogs contain official URLs, local paths, sizes/statuses, and the
  read-only Git source checks.  They deliberately do not claim that a source
  checkout is compiled or experimentally validated.

## SuperLoc Corridor01 (P1)

- Official project: <https://superodometry.com/superloc.html>.
- Official sequence: SubT-MRS Hawkins Long Corridor RC, 279 s (trajectory 617).
- Official map, GT, calibration, and initial pose are present locally:
  - map: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/corridor01.pcd`
    (8,121,136 bytes);
  - GT TUM trajectory:
    `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt`
    (123,311 bytes);
  - initial pose:
    `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/initial_pose/corridor01.yaml`
    (332 bytes);
  - extrinsics: `calibration/corridor01_extrinsics.yaml` (922 bytes);
  - intrinsics: `calibration/corridor01_intrinsics.yaml` (480 bytes).
- The official archive
  `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/SubT_MRS_Hawkins_Long_Corridor_RC.zip`
  is complete at 3,903,086,488 bytes and passed `unzip -t`.  It was extracted
  without changing the archive; the ROS1 bag is
  `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag`
  (6,473,379,689 bytes).
- `rosbag info` is saved at
  `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/docs/rosbag_info.txt`:
  `/velodyne_packets` (`velodyne_msgs/VelodyneScan`, 2,777 messages, about
  9.918 Hz), `/imu/data` (`sensor_msgs/Imu`, 55,957 messages, about 199.846
  Hz), and `/camera_1/image_raw` (`sensor_msgs/Image`, 6,720 messages, about
  24 Hz).  The bag has 65,454 messages over a 280.0 s clock interval.
- SHA-256: extracted bag
  `c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`; archive
  `6f83a45868aa5fa7299690bab78b782642295074d14388dd37707a2cc693aa05`.
- The official TUM GT has 1,385 samples over 279.163510 s.  Its timestamp epoch
  (1517157219.794...) differs from the bag epoch (1690254112.82); this is an
  explicit P2 time/frame audit item, not silently corrected in P1.
- The binary PCD has 338,210 points and fields `Intensity rgb x y z _`; no map
  or GT file was modified.  No topic mapping or playback was performed.
- Current readiness: `READY`, `PRIOR_MAP_READY=YES` for asset availability;
  time/frame alignment remains a P2 adapter gate.

## SuperLoc Corridor02 (P2)

- Official project: <https://superodometry.com/superloc.html>.
- Official sequence: Hawkins RC1, 893 s (trajectory 690).
- Official bag `corridor02.zip` is 15,589,213,681 bytes according to the
  official content range; it was not downloaded in this stage.
- Official GT map, trajectory, extrinsics, intrinsics, and initial pose were
  downloaded to the standard subdirectories (map 66,056,109 bytes; GT 475,805
  bytes; initial pose 1,011 bytes).  Their hashes are recorded in
  `SuperLoc/Corridor02/checksum/SHA256SUMS.txt`.
- Current status: `PARTIAL / DOWNLOAD_PENDING`; `PRIOR_MAP_READY=NO` until the
  complete official bag and `rosbag info` metadata are available.

## GEODE

- Official project: <https://thisparticle.github.io/geode/>.
- Official metadata is present at
  `/media/jian/HIKVISION/paper rosbag/GEODE/official_metadata/`.
- The official site documents raw ROS bags, GT trajectories, and GT maps for
  selected indoor scenarios.  It explicitly describes RTC360 maps for stairs.
- Best next candidate: `Stairs_Alpha` (official bag 3.7 GB, 345 s) because it
  combines a stair degeneracy scenario with the released map family; exact
  file-level map/trajectory pairing must be checked before it is labeled ready.
- No GEODE raw data was downloaded here.  Current status:
  `INVESTIGATED / MAP_PAIRING_PENDING`.

## M3DGR Corridor01

- Official project: <https://github.com/sjtuyinjie/M3DGR>.
- Official README: Corridor01 is 6.39 GB and 403 s; rosbag and ArUco GT are
  separate OneDrive/Alipan downloads.
- The official sensor set is close to the robot data: Livox Avia/MID-360 at
  10 Hz with 200 Hz IMUs, plus D435i RGB/depth/IMU.  Official LiDAR topics
  include `/livox/avia/lidar` and `/livox/mid360/lidar`.
- The official download endpoints require interactive/manual handling in this
  environment.  No bypass or third-party mirror was used.
- Current status: `MANUAL_DOWNLOAD_REQUIRED`; `PRIOR_MAP_READY=NEEDS_INDEPENDENT_MAPPING`.

## Other datasets

- **FusionPortable corridor_day:** official page and metadata are available;
  Ouster OS1-128 + stereo/DAVIS + STIM300 + RTK-GPS, with official map/traj
  organization.  A corridor-specific prior map was not confirmed locally;
  large download deferred.  Status `NOT_DOWNLOADED`,
  `NEEDS_INDEPENDENT_MAPPING`.
- **ENWIDE Tunnel:** official ETHZ page provides Ouster OS0-128 at 10 Hz,
  internal IMU at 100 Hz, Leica MS60 position GT and TUM CSV; no direct prior
  map.  Status `NOT_DOWNLOADED`, `PRIOR_MAP_READY=NO`.
- **NTNU tunnel:** official expected file is 3,472,651,034 bytes with SHA-256
  `f7fe86385ed80675929e1ee45808738cfa0386cb3129b10d1b4753aba1e6d1f7`.
  Only the preserved 0-byte `.partial` marker exists.  Status
  `PARTIAL_INVALID`, `PRIOR_MAP_READY=NO`.

## Algorithm source check

Read-only checks of all archived algorithm repositories and the three hdl
dependencies returned a readable Git status and commit.  Exact branch/HEAD,
official URL, role, and environment-risk classification are in
`ALGORITHM_CATALOG.md`.  No third-party repository was pulled, checked out,
reset, cleaned, compiled, or installed.

## Host and workspace protection

- `sudo apt`, `pip`, `pip3`, `conda`, Docker installation: not performed.
- System ROS/Livox/PCL/Eigen/OpenCV/Open3D/GTSAM/Ceres paths and shell profiles:
  not modified.
- Frozen baseline `/home/jian/livox_ws/dog_visual_loc_ws` was not modified.
- No ROS launch, rosbag playback, or localization run was performed.
- Only this file is intended to be added to the paper Git workspace in this
  stage.

## Readiness for the next stage

The first direct prior-map asset set is SuperLoc Corridor01: official GT map,
GT trajectory, initial pose, calibration, and rosbag are complete locally, with
size, `rosbag info`, and SHA-256 recorded.  Corridor01 is the recommended
first sequence for P2 adapter and baseline screening, subject to auditing the
official GT/bag time epoch and frames.  Corridor02 is a valid second sequence
but its official raw archive is intentionally deferred because it is about
15.6 GB.
