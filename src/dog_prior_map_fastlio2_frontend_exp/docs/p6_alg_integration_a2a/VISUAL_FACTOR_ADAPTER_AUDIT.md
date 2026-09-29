# Visual factor adapter audit

The input `translation_ref_imu` is interpreted as the metric relative
translation from reference IMU to current IMU in the reference IMU frame.  The
factor residual is the existing two-state expression:

```text
r = p_j - p_i - R_i z_ij
```

It connects both endpoint states and uses the event's 3x3 covariance directly.
The adapter never calls `applyProjectedPositionMeasurement()` and never feeds
the measurement through the legacy IKFoM innovation-noise path first.

Admission uses `assessVisualQuality()` with source validity, timestamp age,
tracked/inlier counts and ratio, spatial distribution, depth association,
parallax, reprojection error and prediction consistency.  A rejected visual
event is logged with its reason and contributes no objective term.

The A2A test includes a same-timestamp LiDAR/visual ordering and rejects a
visual endpoint that is no longer in the active window.
