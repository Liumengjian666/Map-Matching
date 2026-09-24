# PAPER-P3-R7F — Floor01 full-SE(3) causal CV deskew adapter

Status: `PAPER-P3-R7F-PARTIAL`

This stage audited the existing Corridor01 v2 adapter and the Floor01 sensor
inputs without changing either localization workspace. The VLP16 decoder and
the SE(3) frame mathematics pass the existing synthetic validation. A real
Floor01 full derived bag was **not** generated because the official Floor01
raw shards contain only Velodyne packets, IMU, and camera messages; they do
not contain the causal prior LiDAR pose stream required by the already
validated Corridor01 v2 translation model. Using Floor01 GT, current-scan
NDT output, accelerometer double integration, or a new scan matcher here
would violate the stage contract.

## Corridor01 v2 provenance (resolved)

The adapter is the unversioned workspace
`/home/jian/livox_ws/superloc_adapter_ws` (no Git metadata). The source and
binary hashes are recorded below so that this audit is reproducible:

```text
shared runtime: 58c4073839247098b6f0f545065056be647336cc64631563f5d070d7dcd3b4e8
full_se3_deskew.hpp: 0cba8177a3542bbc9589e0e9418bc1db1727a66c572cad18eac592fe419f596e
synthetic test source: 9dbf585a1d091d7be39c5f13b50a15f4181db6a720752f5134e55d2b70f67ddc
adapter executable: 3378349bb91f11f6a4e830bad0b9c62b665a8de653e0a5fa3c9c00510664aaf9
synthetic executable: a62e32b68bfd120697e573f83bde2b845e5a1b54112de2bdc272a30bffc3cf41
```

The translation source is explicit in the implementation: two most recent
completed prior messages on `odom_history_in` (Corridor01 uses
`/dog_livo/ndt_odom`). With prior LiDAR poses `(p_l^k,R_l^k)` and
`(p_l^{k-1},R_l^{k-1})`, the calibrated IMU-origin positions are

```text
t_l_i = -R_l_i * t_i_l
p_w_i^j = p_w_l^j + R_w_l^j * t_l_i
v_w = (p_w_i^k - p_w_i^(k-1)) / (t_k - t_(k-1))
```

The velocity is then held constant from the last completed pose to the next
scan start and through that scan. The first two scans use the documented
zero-translation bootstrap. The current scan's final NDT pose is never read.
No GT, future trajectory, or accelerometer double integration is involved.

Rotation is gyro trapezoidal SO(3) propagation in the IMU frame. With the
official calibration convention `p_imu = T_imu_lidar p_lidar`, the relative
LiDAR rotation is the IMU increment conjugated by the extrinsic:

```text
R_L0_Li = R_i_l^T R_I0_Ii R_i_l
```

For the full transform, the world-frame pose at point time is formed from
the same causal velocity and gyro increment and mapped back to the first
packet LiDAR frame:

```text
T_L0_Li = inverse(T_W_L0) T_W_Li
p_L0 = R_W_L0^T (p_W_Li - p_W_L0)
```

Point order is never sorted. Scan reference is the first packet/header stamp.
IMU coverage is strict and never extrapolated; a scan with missing start/end
coverage is rejected.

## Floor01 input audit

```text
archive: /media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/raw/SubT_MRS_Hawkins_Multi_Floor_LegRobot.zip
archive SHA256: d2d1fd8162e5e7752eded6013874c5e2782201d4b6607dcbfb864852e357124e
raw topics: /cmu_sp1/velodyne_packets, /cmu_sp1/imu/data, /cmu_sp1/camera_1/image_raw
LiDAR: velodyne_msgs/VelodyneScan, frame cmu_sp1_velodyne, VLP16
IMU: sensor_msgs/Imu, frame epson, about 199.62 Hz
raw scans: 4139 (76 packets/scan)
decoder: ros-drivers/velodyne 1.7.0,
         89faa698688a48d4a5080f73c04d7aed8117eec0
point timing: official VLP16 RawData::buildTimings()/unpack_vlp16()
reference: FIRST_PACKET_REFERENCE (header == packets[0].stamp)
calibration: /media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml
```

The three raw shards have no `nav_msgs/Odometry`, `geometry_msgs/Pose`, or
other prior-pose topic. Thus the required `two_prior_odom_poses_cv` source is
absent. This is a data-lineage blocker, not a decoder or frame-math failure.

## Validation performed

The existing Corridor01 synthetic executable was run unchanged:

```text
FULL_SE3_SYNTHETIC_TESTS_PASS
raw_wall_sigma=0.0937821
corrected_wall_sigma=7.93407e-16
```

This validates zero motion, pure rotation, pure translation, combined
constant-velocity/yaw, extrinsic round-trip, and strict IMU coverage behavior
of the reused implementation. Floor01 packet geometry and timing had already
been independently validated in R7E, including deterministic official VLP16
decode and finite points.

The required real 30 s smoke, full 417 s derived bag, shard-boundary output,
and output repeatability cannot be validly executed until a causal Floor01
prior-pose stream is supplied. Running the adapter with no pose stream emits
only the zero-translation bootstrap and then intentionally waits; treating
that as a full-SE(3) result would mislabel the input.

## Leakage audit

```text
GT path passed to adapter: NO
ground_truth/oracle source in adapter command/config: NO
current-scan final NDT pose: NO
future completed scan: NO
accelerometer position integration: NO
```

## Decision

```text
ADAPTER_DECISION = B
ROTATION_VALID_TRANSLATION_UNRESOLVED
FLOOR01_DERIVED_BAG_READY = NO
```

The next allowed action is to provide a scientifically equivalent causal
Floor01 prior-pose source (or an explicitly approved adapter-side source). No
Floor01 baseline, Run A/B, ATE/RPE, counterfactual, NDT tuning, predictor
change, visual fusion, or P4 work was started.
