# Bounded download attempt log

## Storage check

Before download, `/media/jian/HIKVISION` was verified as the read-write mount for filesystem UUID `F627-4133`; it had approximately 431 GiB available. After the two Long_Corridor attempts, `df -B1` reported 459,366,203,392 bytes available. No existing P45–P55 data were removed or overwritten.

## Ground-Challenge Corridor2

- Official source: [Ground-Challenge README](https://github.com/sjtuyinjie/Ground-Challenge), Corridor2 OneDrive share: `https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/EcWKsukr7nNLrqKloXY3UAIBKakA6ppET_SGJZT-VCYO5Q?e=Ccp0fd`.
- Attempted access check only, not counted as a dataset download: followed redirect/one-byte range request.
- Final response: `login.microsoftonline.com`, HTTP 200 `text/html`, 58,672 bytes.
- Saved dataset bytes: 0. The auth page was not written as a bag/ZIP.
- No authentication bypass, alternate mirror, or repeated retry was attempted.

## SubT-MRS Long_Corridor

- Official source: [SubT-MRS datasets](https://superodometry.com/datasets), [ICCV23 challenge page](https://superodometry.com/iccv23_challenge_Mul), official Drive file ID `1-WF_cIPXVEzcStfYjJWoZfJkld0Ozmxk`.
- Expected file: `SubT_MRS_Hawkins_Long_Corridor_RC.zip`, 3,888,497,930 bytes.
- Official endpoint advertised byte ranges; a one-byte probe returned HTTP 206 with `Content-Range: bytes 0-0/3888497930`.

1. Initial sequential curl attempt to the official Drive download endpoint. It received HTTP 200 and 1,699,970,593 bytes, then exited 18 (partial transfer). The resulting ZIP prefix was retained as `.part`.
2. One resume attempt against the same official endpoint and same `.part`, using `curl --continue-at -` (no parallel download, no overwrite). It received HTTP 206 and another 1,780,770,630 bytes, then exited 56 (receive failure). Cumulative file size: 3,480,741,223 bytes; short by 407,756,707 bytes. The file remains at:

   `/media/jian/HIKVISION/paper rosbag/DegenerateBenchmarks/SubTMRS/LongCorridor/raw/SubT_MRS_Hawkins_Long_Corridor_RC.zip.part`

   Local SHA-256: `6fb37b96d9ac80ebe3fd32cc113504d1c8c6c791919a30b073a0b4a8786f16fa`.

The two-attempt limit is exhausted. No third attempt, alternate downloader/source, extraction, `unzip -t`, or ROS inspection was performed. The partial is preserved for user recovery; it must not be treated as a complete archive.

## Calibration assets

- Ground-Challenge official `calibration.txt`: 932 bytes, SHA-256 `f8f591c46fa21c88d3986e65bf0558c7f79f2830b017165ec86b87190f745e08`, stored at `/media/jian/HIKVISION/paper rosbag/DegenerateBenchmarks/GroundChallenge/official_metadata/calibration.txt`. It contains camera intrinsics and `body_T_cam0`, `body_T_wheel`; it does not provide an explicit LiDAR–IMU extrinsic.
- SubT-MRS Long_Corridor official extrinsics YAML: 584 bytes, SHA-256 `03df4bc1d32ba7148fd90a35f370d319bb83531c774d1848b3da0acf25f390bc`, stored at `/media/jian/HIKVISION/paper rosbag/DegenerateBenchmarks/SubTMRS/LongCorridor/calibration/SubT_MRS_Hawkins_Long_Corridor_RC_Extrinsics.yaml`. It includes `laser_to_imu` and `rgb_camera_to_imu`; transform direction/frame semantics have not been established, so neither was applied.
