# P61-R5 pre-registered mapping protocol

START_SHA: `019140e7bc9f8e276ce3022ea2b45419c1e00361`.
Author FAST-LIO: `thisparticle/GEODE_Evaluation@1f008a7249e36393a1752622de50660b77b5b7f4`, FAST_LIO (GPL-2.0; retain upstream notices).

No GT input, no external disk, no NDT/Coupled/Recovery/EKF-localizer run.
All source acquisition, build, sensor input and map artifacts are on internal disk.

## Immutable sensor-time gate

Mapping window: [1706671289400000810, 1706671533569869338] ns.
Test window: [1706671587829840123, 1706671831999708652] ns.
Only `/livox/lidar` and external Xsens `/imu/data` may enter the isolated ROS master.
Each admitted LiDAR scan must have its **entire** header+offset interval inside the mapping window. This admits scans 1–2441. Scan 2442 has an in-window header but its tail crosses the immutable boundary; exclude it whole, without modifying points or times. This is a boundary admission rule, not a changed window or result-driven selection.
IMU messages must also have header stamps inside the mapping window. The mapper's original first-scan/IMU initialization stages remain unchanged and are counted separately from valid map updates.
Before ROS, create and independently re-traverse a physically filtered mapping-only bag. Validate every admitted serialized LiDAR SHA against the R2 ledger. The replay tool accepts only this filtered bag and its frozen manifest.

## Stages

1. Audit actual header/offset/IMU source consumption; no timebase-to-epoch substitution or time shifts.
2. Build author code with observation-only per-scan completion/state/deskew logging and final ikd-Tree export. Preserve all estimation, preprocessing, integration and insertion mathematics.
3. Smoke: first 100 admitted scans. Per-consumed-scan acknowledgment, including original initialization skips; do not assume a sleep proves processing completion.
4. If smoke is valid, start a fresh mapper and replay all 2441 admitted scans once. No test inputs. Save the actual final map tree, not GT-transformed scans. Report initialization scans, first tree construction and actual map updates separately.
5. Freeze PCD, trajectory, configurations, input and binaries with SHA256 before any coverage queries.
6. Freeze 50 time-uniform test indices independently of results. Without legitimate map-frame test poses, mark point-cloud coverage uncertified; do not use test GT. Report initialization as unestablished unless actually obtained through a separately documented non-GT causal state chain.

## Fixed author settings

Use frozen `fastlio_carol.yaml`: external Xsens, time_sync=false, time_offset=0, extrinsic estimation=false, lidar_type=1, scan_line=6. Launch parameters: max_iteration=10, feature_extract_enable=false, point_filter_num=1, filter_size_surf=0.3m, filter_size_map=0.3m, cube_side_length=1000m. No tuning. PCD saving changes only the export mechanism: snapshot the actual final map tree, avoiding accumulation of a second dense point history.

Mapper world M is `camera_init`, body is IMU. Record `T_M_I`, actual `T_I_L` and `T_M_L=T_M_I*T_I_L` at scan end, with real integer scan start stamps retained separately. No global alignment.

Disk guard: retain at least 20 GiB free. Map/build memory is measured separately from replay. One grounded adaptation repair is allowed only if the smoke exposes an actual problem. No full localization is authorized.
