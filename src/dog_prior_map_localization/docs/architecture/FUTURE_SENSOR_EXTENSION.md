# Future Sensor Extension Boundary

The delivery EKF intentionally contains only IMU propagation, external NDT
observation fusion, OOSM replay, and state/output publication.  Future visual
or other sensor work must enter through a typed boundary:

```text
sensor frontend
      -> typed measurement
      -> fusion / estimator
```

A future frontend should expose timestamp, relative or absolute measurement,
covariance/information, validity, and a reason code.  It must not directly
access estimator internals such as `p_`, `R_`, or `P_`.  Direction selection and
measurement weighting belong in the fusion layer, not in a camera callback.

Possible future source locations are `vision/visual_frontend.cpp` and
`vision/metric_visual_estimator.cpp`, but this document deliberately adds no
placeholder classes or runtime code.
