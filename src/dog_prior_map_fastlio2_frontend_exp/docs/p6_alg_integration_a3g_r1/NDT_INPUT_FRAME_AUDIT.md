# Logged coordinate and precision contract

No frozen source was edited. The following reads only resolve log meaning:

- p6_a2c_fixed_lag_producer.hpp:568–583: prediction is Window map_T_imu;
  NDT seed is prediction*T_imu_lidar; terminal is map_T_lidar; measurement
  conversion calls poseFromMatrix before lidarMeasurementToImu.
- p6_i1_branched_recovery.cpp:760–764: poseFromMatrix converts NDT float rotation
  to a normalized quaternion first.
- p4_i2_state_contamination_replay.cpp:139–189:27-parameter layout and
  map_T_imu = map_T_lidar * inverse(T_imu_lidar).
- p6_i1_branched_recovery.cpp:1246–1265: matrix fields flatten row-first and
  use10 significant digits; vector fields use10 significant digits. Direct
  quaternion CSV fields use the enclosing stream's17-digit precision.

Official frozen parameter SHA:
`7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d`.
T_imu_lidar translation [0.08,0.029,0.03] m; quaternion xyzw
[0.00043559155384906891,0.002000771806067129,
0.00025894112483084737,0.99999787005856766], normalized by the same convention.

For innovations we first normalize the logged LiDAR rotation through quaternion
conversion, then apply the inverse extrinsic; compare position in map and
rotation Log(R_pred^T R_measured_imu). Raw terminal matrices and raw predicted
fields are also retained. They are not directly subtracted across sensor frames.

The initial offline helper normalized only after composing the raw float rotation
with the extrinsic. Directed review identified this ordering mismatch; it was
corrected **only in the new analysis helper**, with a non-SO3 regression. Maximum
old/new innovation-vector discrepancy was about1.02e-8 m /2.51e-10 rad in the
window, far too small to explain support collapse. Final CSVs use corrected order.

Seed-return innovations at~1e-6 m/~1e-8 rad are at float/log rounding scale, not
exact-zero physical observations. No high-precision or bit-exact pose agreement
with original in-memory states is claimed. Accuracy, basin membership and
calibration validation were not estimated in this task.
