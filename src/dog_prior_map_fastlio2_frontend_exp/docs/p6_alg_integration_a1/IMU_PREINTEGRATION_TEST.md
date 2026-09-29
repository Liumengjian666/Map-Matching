# IMU preintegration test

`p6_i6f_imu_preintegration_test` uses static, constant angular-rate,
constant-acceleration, irregular-timestamp, bias-correction, Jacobian
finiteness, and non-monotonic-input cases.  Release result:

```
I6F_IMU_PREINTEGRATION_TEST_PASS static=1 constant_gyro_angle=0.2 irregular_dv=0.8 irregular_dp=0.16 covariance_trace=0.0127552 bias_jacobian_norm=0.69282 gyro_bias_fd_error=1.47523e-16 accel_bias_fd_error=0 jacobian_norm=4.22079
```

The continuous-time noise-density discretization is checked against
`sigma^2 * duration`, and gyro/accelerometer bias Jacobians are checked by
central finite differences.  This is a synthetic math test; it is not a
public-bag replay.
