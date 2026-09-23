# Obsolete Result Cleanup Manifest

Date audited: 2026-09-23
Reason: old SuperLoc results used ROTATION_ONLY_DESKEW and are superseded as experiment inputs by the verified FULL_SE3 v2 bag. Old files are not silently treated as final baseline evidence.

P3-R1 was interrupted/superseded due to the old input's missing translational motion compensation. Per the explicit stage-switch instruction, keep the old P3/P3-R1 CSV and code for now; preserve `/home/jian/livox_ws/ndt_landscape_ws`. Do not delete Git history, official data, or unknown/user-owned files.

## Cleanup gate evidence

- v2 bag exists and passes timestamp/hash repeatability checks; SHA256 is recorded in its sidecar and baseline metadata.
- Separate v2 smoke: 34.8 s, `rosbag play` exit 0, 346 NDT messages.
- Full Run A: 279 s, `rosbag play` exit 0, 2,776 NDT frames, complete result bag.
- Full Run B and A/B NDT geometry comparison also complete.

## Exact-path manifest

| path | audited size | classification/action | reason |
|---|---:|---|---|
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/derived/corridor01_adapted_v1.bag` | 1.7 GiB | DELETE_AFTER_V2_PASS (deleted) | Large ROTATION_ONLY_DESKEW derived input; SHA, metadata, and summary retained. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/baseline_smoke_20260923_final` | 195 MiB | DELETE_AFTER_V2_PASS (deleted) | Old rotation-only smoke superseded by v2 smoke. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/frozen_baseline_runA_20260923` | 1.9 GiB | DELETE_AFTER_V2_PASS (deleted) | Old rotation-only full Run A. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/frozen_baseline_runB_20260923` | 1.9 GiB | DELETE_AFTER_V2_PASS (deleted) | Old rotation-only full Run B. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/frozen_baseline_derived_runC_20260923` | 99 MiB | DELETE_AFTER_V2_PASS (deleted) | Old rotation-only Run C; metrics summarized and pose-based scan-map sanity recorded. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/frozen_baseline_derived_runD_20260923` | 93 MiB | DELETE_AFTER_V2_PASS (deleted) | Old rotation-only Run D, superseded by v2 A/B. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p3_landscape` | 32 MiB | KEEP | Contains the old P3/P3-R1 CSVs explicitly requested to be retained. |
| `/home/jian/livox_ws/ndt_landscape_ws` | n/a | KEEP | P3 probe source explicitly retained. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/derived/corridor01_adapted_full_se3_v2.bag` | 3.31 GiB | NEVER_DELETE | Current verified full-SE(3) derived input. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/derived/corridor01_adapted_full_se3_v2.pending.bag` | 2,651,504,651 bytes; 2026-09-23 16:27 | KEEP | Earlier 209 s partial v2 processing artifact (2,076 NDT/cloud frames). It is not the accepted input; retained as forensic evidence because it has no independent sidecar provenance and is not required to be deleted. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/baseline_smoke_20260923_01`, `_02`, `_03` | 881,109 / 881,501 / 876,538 bytes; 05:24 / 05:27 / 05:29 | KEEP | Small early smoke records; not among the explicitly confirmed old-result deletion targets, retained rather than guessing their ownership/use. |
| `results/p2c_adapter_smoke_*`, `p2c_adapter_repeat*`, `p2c_adapter_sync*`, `p2c_point_time_*`, `p2c_fullse3_v2_*` | 0.39–672 MiB each; 2026-09-23 15:46–16:50 | KEEP | P2C adapter/time-window/hash diagnostics. Preserve implementation and determinism audit evidence. |
| `results/baseline_full_se3_deskew_20260923_170936` | 1,977,503,152 bytes; 2026-09-23 17:09 | NEVER_DELETE | Accepted full-SE(3) Run A/B, metadata, and diagnostics. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag` | 6.0 GiB | NEVER_DELETE | Official raw input. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt` | 1.3 MiB | NEVER_DELETE | Official evaluation reference. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/corridor01.pcd` | 7.8 MiB | NEVER_DELETE | Official map. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/calibration/corridor01_extrinsics.yaml` | <1 MiB | NEVER_DELETE | Official audited calibration. |
| `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/initial_pose/corridor01.yaml` | <1 MiB | NEVER_DELETE | Official initialization data. |
| `results/p2c_*` adapter/hash/point-time diagnostics | varies | KEEP | P2C implementation and determinism audit evidence; not cleanup targets. |
| `.vscode/` in paper workspace | unknown | KEEP | User/unknown-origin untracked file; not inspected or touched. |

## Other retained top-level result directories

Sizes below are logical bytes (`du -sb`); timestamps are directory mtimes (Asia/Shanghai, 2026-09-23). All are retained. Small baseline smoke folders have unclear relation to the explicitly named rotation-only final smoke, so they were not guessed disposable.

| directory | bytes | mtime | purpose/classification |
|---|---:|---|---|
| `baseline_full_se3_deskew_20260923_170936` | 1,977,503,152 | 17:09:51 | Current full-SE(3) result; NEVER_DELETE. |
| `baseline_smoke_20260923_01` | 881,109 | 05:24:08 | Early smoke; KEEP, provenance not fully established. |
| `baseline_smoke_20260923_02` | 881,501 | 05:27:51 | Early smoke; KEEP, provenance not fully established. |
| `baseline_smoke_20260923_03` | 876,538 | 05:29:31 | Early smoke; KEEP, provenance not fully established. |
| `p2c_adapter_repeat_raw35s_20260923` | 444,189,024 | 16:01:49 | Adapter input/hash repeatability; KEEP. |
| `p2c_adapter_repeat_raw35s_valid_20260923` | 447,426,552 | 16:07:52 | Adapter repeatability after timestamp validation; KEEP. |
| `p2c_adapter_repeatcheck_35s_20260923` | 1,180,236 | 15:58:20 | Short repeatability check; KEEP. |
| `p2c_adapter_smoke_154640` | 151,386,180 | 15:46:51 | Adapter smoke; KEEP. |
| `p2c_adapter_smoke_154803` | 228,819,495 | 15:48:14 | Adapter smoke; KEEP. |
| `p2c_adapter_smoke_155040` | 228,832,563 | 15:50:51 | Adapter smoke; KEEP. |
| `p2c_adapter_smoke_35s_155224` | 672,113,149 | 16:07:50 | 35 s adapter smoke; KEEP. |
| `p2c_adapter_sync35s_20260923` | 445,134,539 | 16:15:37 | Adapter synchronization test; KEEP. |
| `p2c_adapter_syncdelay35s_20260923` | 447,443,801 | 16:18:46 | Adapter synchronization delay test; KEEP. |
| `p2c_fullse3_v2_build_20260923` | 8,778,282 | 16:23:54 | Initial v2 attempt diagnostics; KEEP for debugging provenance. |
| `p2c_fullse3_v2_build2_20260923` | 11,568,593 | 16:32:12 | Accepted v2 build diagnostics; KEEP. |
| `p2c_fullse3_v2_hash_20260923` | 6,420,772 | 16:50:55 | First v2 hash/timestamp audit; KEEP. |
| `p2c_point_time_audit_80s_20260923` | 37,308,155 | 16:28:12 | Point-time boundary audit; KEEP. |
| `p2c_point_time_retest_78s_20260923` | 392,278 | 16:30:21 | Point-time boundary retest; KEEP. |
| `p3_landscape` | 13,663,126 logical (~32 MiB allocated) | 14:20:42 | Old P3/P3-R1 CSVs and outputs; KEEP per stage-switch instruction. |

## Derived directory inventory

Sizes are bytes; timestamps are mtimes on 2026-09-23, Asia/Shanghai.

| file | bytes | mtime | classification |
|---|---:|---|---|
| `corridor01_adapted_full_se3_v2.bag` | 3,546,555,819 | 16:36:58 | NEVER_DELETE; accepted v2. |
| `corridor01_adapted_full_se3_v2.bag.sha256` | 164 | 17:46:38 | KEEP; accepted v2 hash. |
| `corridor01_adapted_full_se3_v2.meta.yaml` | 1,909 | 17:43:49 | KEEP; v2 provenance. |
| `corridor01_adapted_full_se3_v2.pending.bag` | 2,651,504,651 | 16:27:29 | KEEP; partial earlier v2 attempt, explicitly not used as accepted input. |
| `corridor01_adapted_v1.bag.sha256` | 155 | 09:26:11 | KEEP; old v1 hash retained after bag deletion. |
| `corridor01_adapted_v1.meta.yaml` | 1,307 | 09:28:06 | KEEP; old v1 provenance retained. |
| `corridor01_adapted_v1_deskew.csv` | 327,739 | 09:25:54 | KEEP; old v1 diagnostics retained. |
| `corridor01_adapted_v1_rosbag_info.txt` | 649 | 09:26:01 | KEEP; old v1 bag summary retained. |
| `derived_cloud_hashes.csv` | 338,728 | 09:27:00 | KEEP; timestamp/cloud audit record. |
| `derived_imu_hashes.csv` | 5,818,407 | 09:27:00 | KEEP; timestamp/IMU audit record. |
| `derived_timestamp_audit.json` | 349 | 09:27:00 | KEEP; timestamp audit summary. |

Removed the old v1 bag and all result payloads from the five named result directories above, using only their exact paths; no glob or workspace-wide cleanup was used. Two stale Run C/D ROS masters had only `/rosout` registered and were holding their zero-byte `roscore.log` files open. Sent SIGINT only to those four verified processes (masters and rosout PIDs 68456, 68481, 71089, 71114), then removed the now-empty exact Run C/D directories. No other ROS process was touched. The old v1 bag's `.sha256`, `.meta.yaml`, deskew CSV, and rosbag-info text remain. The complete old P3/P3-R1 result directory and probe source remain untouched. Git history remains untouched.

## Space released

Available space increased by 6,195,511,296 bytes (about 5.77 GiB) between the recorded pre-cleanup and post-cleanup `df -B1` samples. Full input data and current v2/A/B results remain available.
