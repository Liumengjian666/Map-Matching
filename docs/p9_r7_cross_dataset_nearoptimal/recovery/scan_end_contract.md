# Scan-end contract: NOT_RUN_INPUT_BLOCKED

Historical v1 is rotation-only scan-start data. It is not already a P9
scan-end SAME_OBJECTIVE source. The restored payload was not available, so no
field-level parity, end-state conversion, source/T0/U_obs or repeatability claim
is possible in this attempt.

## Static inspection facts (not historical payload certification)

Pinned Velodyne rawdata.cc computes VLP16 point time as packet timestamp minus
scan-start timestamp plus the fixed firing offset (55.296 us cycle / 2.304 us
single firing). This is per-point relative time, not bag record time.

Current shared adapter emits x/y/z/intensity/ring/time and headers at scan start.
It has both rot_out and full_out; points_rot_only_pub publishes the former,
while default points_pub publishes full_out. It also waits for the preceding
two odometry stamps after bootstrap. A simple topic rename is not established
as exact v1 reconstruction. No such rename or code patch was performed.

Current hash_derived_bag.py hashes PointCloud2 stamp/frame/layout/field schema
and all data bytes; its IMU hash covers orientation, gyro, acceleration and
their covariance arrays. This is more than an XYZ-only hash. However, its exact
historical implementation is not pinned by the P2B source manifest. Current
function inspection alone cannot prove historical hash-algorithm identity.
Header sequence number and bag record time are not covered by that current
content hash; required propagation-field coverage must be explicitly assessed
before any content-equivalence admission. Archived hashes cannot reveal the
missing bag's actual ring/time schema by themselves.

## Required future chain, not executed

1. Identify scan start and end from validated point timing, in sensor time.
2. Use verified raw measurements and LiDAR-to-IMU extrinsics for deskew into
   the IMU/LiDAR state at scan end; do not double-deskew recovered v1 blindly.
3. Preserve T_map_lidar = T_map_imu * T_imu_lidar and the explicit source frame.
4. Bind prediction, nominal NDT terminal T0, objective, prepared-source hash,
   U_obs and spatial chart to that same scan-end state and point ordering.
5. Compare two whole-sequence runs under predeclared numeric tolerances.

This is a pending contract, not an implementation or a PASS certificate.
