# PAPER-P3-R10A Initial Map-Pose Provenance

Status: `FLOOR01_RUNTIME_GATE = BLOCKED`.

This is an engineering provenance gate, not a localization result. GT is not
used as a runtime input or to select an initialization pose.

## Required initial condition

IMU static initialization can establish gyro bias, an acceleration/gravity
direction relationship and (under a validated stationary gate) an initial
velocity prior. It cannot establish global map translation or global yaw.
Before Floor01 runtime, R10A requires a provenance-backed
`initial_map_T_lidar`; with fixed calibration `T_imu_lidar` the initial filter
pose is `initial_map_T_imu = initial_map_T_lidar * inverse(T_imu_lidar)`.

Allowed sources are: (A) an official localization initial pose whose direction,
frames and timestamp are documented; (B) a map-construction record proving
that map origin/orientation equals sequence start; or (C) a documented
non-GT manual/system prior. GT, GT-derived alignment, and values chosen after
viewing GT are prohibited.

## Evidence inspected

1. The released file
   `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/initial_pose/multi-floor01.yaml`
   has SHA-256
   `6da3daf5194b30e27c2b07956738798b4e39d5b2e613c71206da683961737c98`.
   It contains `extrinsicRotation_world_darpa` and
   `extrinsicTranslation_world_darpa`, with translation approximately
   `[39.972755, -175.753494, -5.125434] m`. Existing official-source audit
   found no parser for these keys and no authoritative declaration of their
   direction, endpoint frames or timestamp. That audit is
   `PARTIAL`; the YAML is not yet an eligible official initial pose.
2. The released source map
   `.../Floor01/map/floor01.pcd` has SHA-256
   `bad163fdcada69b975159365dfd776befde542cd08cdd0a2ce1f702a3d0aaf5d`.
   The runtime H1-normalized map used by prior experiments has SHA-256
   `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.
   Its normalizer used the inverse of a candidate `T_map_lidar_H1` to form
   normalized coordinates. The H1 direction was selected by map consistency,
   not closed by official transform semantics. This does not prove that the
   normalized origin/orientation equals the sequence-start pose.
3. R7G/R7H records call H1 an operational initialization selected for map
   consistency and explicitly say official matrix semantics remain
   unconfirmed. A near-origin first NDT result and repeatability demonstrate
   operational behavior only; they do not establish independent pose lineage.
4. The earlier frozen localizer's identity first guess is a record of its
   runtime behavior, not provenance that identity is a valid R10A initial
   `map_T_lidar` for the official map.
5. No independently documented non-GT manual/system prior for this R10A run
   was found in the reviewed configuration or provenance records.

Relevant existing evidence:

- `results/p3_r7_floor01_prerequisite/floor01_initial_pose_audit.md`
- `results/p3_r7_floor01_prerequisite/floor01_map_audit.md`
- `results/p3_r7_floor01_closed_loop_smoke/floor01_operational_initial_pose_audit.md`
- `results/p3_r7_floor01_full_baseline/floor01_runtime_provenance.md`
- Corridor01 `results/p3_reference_lineage_closure/official_initial_pose_audit.md`
  and `map_gt_frame_evidence.md` (used only as corroborating caution; not as a
  Floor01 pose source)

## Decision

```text
official initial-pose source: NOT CLOSED
map-origin == sequence-start convention: NOT PROVEN
approved non-GT prior: NOT PRESENT
identity initialization approved: NO
GT used: NO
GT-independent initial pose: UNRESOLVED (no eligible pose source)
INITIAL_MAP_POSE_GATE: BLOCKED
FLOOR01_RUNTIME_GATE: BLOCKED
FLOOR01_ROSBAG_ALLOWED: NO
```

Protocol documents, interface definitions, and the compile/unit API spike may
proceed. Do not perform the Floor01 startup replay, high-dynamic replay, or full
Run A/B until the project owner supplies or approves a provenance-backed
non-GT `initial_map_T_lidar` and the IMU/LiDAR time-offset gate is closed.
