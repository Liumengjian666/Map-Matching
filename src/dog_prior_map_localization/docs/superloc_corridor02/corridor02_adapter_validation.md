# Corridor02 adapter validation

Status: `NOT_STARTED`.

The adapter is intentionally not generated because the official Corridor02
rosbag is unavailable (Google Drive quota response) and the GT origin is
unresolved. Consequently there is no trustworthy topic/point-time audit from
which to select a scan reference, no RC1 packet conversion, and no derived
`corridor02_adapted_full_se3_v1.bag`.

The planned adapter, once the bag is obtained, is constrained to:

- IMU gyro rotation integration with the official RC1 LiDAR/IMU extrinsic;
- causal constant velocity from previously completed LiDAR poses only;
- scan-start reference with strict IMU coverage and no extrapolation;
- no GT, no future data, no current-scan final NDT feedback, and no raw
  accelerometer double integration;
- synthetic rotation, translation, extrinsic round-trip, point-time,
  first-scan coverage, and finite-point tests before runtime replay.

No runtime source, NDT parameter, EKF, predictor, limiter, or frozen baseline
was modified.
