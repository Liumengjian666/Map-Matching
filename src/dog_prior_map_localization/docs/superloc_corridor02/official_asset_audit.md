# Official asset audit: SuperLoc Corridor02

Audit date: 2026-09-24 (Asia/Shanghai)

The authoritative source is the official SuperLoc release page:
https://superodometry.com/superloc.html

The official page identifies Corridor02 as SuperLoc / Hawkins / RC1 with RGB,
LiDAR and IMU, trajectory index 690 and duration 893 s. It links the bag,
RC1 extrinsics, camera intrinsics, GT map and GT trajectory. The official
SuperOdom source documents the transform convention as `imu^R_laser` and
`imu^T_laser`; it does not define the origin of the Corridor02 GT file.

Static local assets and checksums are recorded in
`CORRIDOR02_DATASET_AUDIT.md` and
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/checksum/SHA256SUMS.txt`.

The official bag content length is 15,589,213,681 bytes. Google Drive returned
“Too many users have viewed or downloaded this file recently” for the official
download. No third-party mirror was used. Free space on `/media/jian/HIKVISION`
at audit time was approximately 534 GB, so disk space is not the blocker.

Result: `ASSET_AUDIT_PARTIAL_BAG_NOT_DOWNLOADED`.
