# P11 SOLiD mature-module integration protocol

Frozen before the first real independent query run. This is engineering
integration and descriptive place-retrieval analysis, not accuracy validation.

- Upstream: sparolab/solid commit
  `9a4f2895e5211f69fbbd6d4732577e1a67e9b882`, BSD-3-Clause.
- Preserve upstream `makeSolid`, `pt2rah`, `loop_detection`, and .4 m voxel.
  Compatibility constants: MID-360 elevation -7 to +52 degrees, range >3
  and <40 m, 40 range / 60 angle / 32 height bins. Reject nonfinite points,
  fewer than 50 compatible/downsampled points, zero height-normalizer range,
  invalid bins, and nonfinite/zero range descriptors. No learned parameter.
- Offline references: independent Loop2 mapping recording
  `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`;
  map and poses from `/home/jian/rosbag/loop2/loop2mapping/pcd/`.
  Existing keyframe extractor: 1 m spacing, nearest raw header within .12 s,
  no source subsample before NDT-style preprocessing. Its pose association
  is approximate and is audited; it is not GT nor an exact deskewed keyframe.
  `LIVMapper.cpp:1293` stores `_state.pos_end/rot_end`, i.e. the mapping
  state/IMU origin, despite the filename `lidar_poses.txt`. The database
  carries `map_T_reference` (mapping state pose), not an asserted exact
  `map_T_lidar`. A historical physical LiDAR-IMU calibration for this map
  generation is not independently proved. These coarse reference locations
  are diagnostic only and cannot directly initialize or replace NDT poses.
- Reject older archived mapping poses whose coordinates differ from the
  active map lineage. Corridor01 has no legal independent keyframe database
  in the available assets, so it is NOT used to manufacture references.
- Queries: complete independent Loop3 recording
  `loop3_2026-08-11-16-09-07.bag`; only `/livox/lidar` and `/livox/imu`
  played. No old recorded localization/odometry used as a prior.
- Source: exact `/dog_livo/filtered_points` cloud consumed by the unchanged
  NDT node. Source preprocessing .5–80 m, .25 m voxel, max 1400 points.
  Current robot baseline has deskew DISABLED and midpoint observation
  timestamps. These are preserved, not described as verified deskewed scans.
- Existing fixed-lag OOSM enabled identically in A/B: .15 sensor seconds,
  max queue64. Replay rate1.0. One fresh isolated ROS master per run.
  Last observation deadlines beyond final genuine IMU coverage are reported,
  never supplied synthetic IMU samples. Late arrivals fail parity certification.
- A: original CONTROL. B: same CONTROL plus independent optional SOLiD
  subscriber, top5 deterministic cosine ranking / reference-ID tiebreak.
  No NDT initialization, EKF observation, TF, or feedback output from SOLiD.
- Nominal NDT params unchanged: .8 m resolution, .08 step, epsilon .001,
  max40. Same map, initialization, noise, voxel filters and step limits.
- Parity: exact same source hashes/counts, NDT numerical/pose fields, OOSM
  release/replay fields, EKF lineage numerical/state fields after excluding
  wall/arrival timing only. No relaxed scientific comparison tolerances.
- Report cosine scores and reference pose separations, not posterior,
  verified NDT attractors, retrieval correctness, or localization accuracy.
  A >5 m Top-K reference separation is descriptive only; it is NOT an
  admission threshold or an ambiguity label.
- GT_LOADED=NO; ACCURACY_GAIN_NOT_EVALUATED. No independent map-frame query
  position truth has been established. No additional full NDT calls.
- Persistent output: `/home/jian/rosbag/p11_solid_r1_20261011/` on ext4.
  Only compact descriptors/poses are loaded online, never raw keyframe clouds.
