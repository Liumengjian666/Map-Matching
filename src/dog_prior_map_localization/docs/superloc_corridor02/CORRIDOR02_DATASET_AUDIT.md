# PAPER-P3-R7 Corridor02 dataset audit

Status: `PAPER-P3-R7-PARTIAL` (hard gates: `GT_REFERENCE_UNRESOLVED` and the
official bag download is blocked by Google Drive quota).

## Official release

- Dataset page: https://superodometry.com/superloc.html
- Official code: https://github.com/superxslam/SuperOdom
- Sequence: `Corridor02`, source `SuperLoc`, location `Hawkins`, platform `RC1`.
- Sensors listed by the official page: RGB, LiDAR, IMU.
- Official trajectory index: 690; official duration: 893 s.
- Official resources: rosbag, extrinsics, intrinsics, GT map, GT trajectory and
  initialization pose.
- Official bag URL:
  https://drive.google.com/file/d/1fbQIjza6zCVZ719VvXfNhAONDZflqGnf/view?usp=sharing
- Official bag content length: `15,589,213,681` bytes. The attempted download
  returned an HTML `Quota exceeded` response, not a bag; that invalid response
  was moved to `raw/rejected_download_20260924_quota/` and is not an input.

## Local static assets

The following files are present under
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/` and match the local
checksum manifest:

| asset | bytes | SHA256 |
|---|---:|---|
| `map/corridor02.pcd` | 66,056,109 | `6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228` |
| `gt/corridor02_gt.txt` | 475,805 | `cddb6739230ded86c57412e768e071e5cd6d62a5de5ff310618032ca2e38be0a` |
| `calibration/corridor02_extrinsics.yaml` | 860 | `892bf743dc0f51eabfe9d8ae71dc98663753f174910ae9cc40ee244c77a9e217` |
| `calibration/corridor02_intrinsics.yaml` | 478 | `8d47e98aed48cf941c8ee642136514b50ea7e6dd0bd26de9266081bd8693fe12` |
| `initial_pose/corridor02.yaml` | 1,011 | `fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1` |

The GT file has 5,522 TUM rows spanning `892.669214 s` (timestamps
`1645999726.984117` to `1646000619.653331`). The PCD header reports 5,223,422
points and a binary-compressed PCD. These checks are inventory only.

## Gates not yet executable

The rosbag is required to audit exact ROS topics, message types, rates, bag
start/end, LiDAR point-time fields, scan reference semantics, and frame IDs.
No Corridor02 adapter or derived bag was generated. No runtime baseline was
started, and no Corridor01 RC2 topic/calibration assumptions were copied.
