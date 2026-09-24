# P3-R9B FAST-LIO IMU Semantics Audit

Audited local reference checkout:

```text
Repository tree: /media/jian/HIKVISION/comparison algorithm/FAST_LIO2
Commit: 7cc4175de6f8ba2edf34bab02a42195b141027e9
Reference file: src/IMU_Processing.hpp
License: GPLv2
```

This report records algorithm semantics only. No source code was copied into this project.

## `IMU_init()`

The reference updates running means for both accelerometer and gyro samples. It initializes gravity direction from `-mean_acc / norm(mean_acc) * G_m_s2` and gyro bias from `mean_gyr`. It also accumulates measurement covariance statistics and initializes its own state covariance. R9B adopts the static gyro-mean bias initialization in its isolated Floor01 experiment, while preserving this package's existing gravity initialization and not adding an accelerometer-bias estimator.

## `UndistortPcl()` interval semantics

The reference prepends the prior frame-tail IMU sample to the current scan's IMU group. For each adjacent `head`/`tail` pair it computes componentwise `0.5 * (head + tail)` for gyro and acceleration, chooses `dt` from the sample interval (or clips the first interval to the prior LiDAR end), sets those values as the filter input, predicts forward, and stores the state at the tail IMU timestamp in its `IMUpose` sequence. It then predicts from the final IMU to the LiDAR frame end and traverses saved IMU poses backward to compensate points.

The reference normalizes acceleration by `G / mean_acc.norm()` because that implementation explicitly normalizes its startup acceleration representation. R9B's Floor01 input is `sensor_msgs/Imu` in SI units, with measured first-200 mean acceleration norm `9.8195039 m/s²`, close to standard gravity. Therefore the R9B profile records `ACC_SCALE_NOT_PORTED`: blindly multiplying by `G / mean_acc.norm()` would rescale already-SI input without evidence that its magnitude representation needs that normalization. This is a unit/measurement decision, not a claim that a different scale could never be calibrated.

## R9A → R9B difference

R9A used a single timestamp sample (ZOH) for partial intervals and had no static gyro-mean bias initialization. R9B adds explicit raw-sample versus interval-input state semantics; shared head/tail mean interval input for experimental EKF propagation, OOSM replay, and scan-bounded partial deskew; and optional gyro bias initialization from the accepted static window. A partial fragment at scan end cannot consume a tail sample beyond scan end. This preserves the maturity pattern without copying FAST-LIO's entire estimator or its backward deskew implementation.

## Provenance and claims

- Direct FAST-LIO source copied: **NO**.
- Mature interval/bias concepts referenced: **YES**.
- Novel contribution: **NO**; this is mature infrastructure hardening.
- Ground truth used in this audit: **NO**.
