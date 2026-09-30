# One-time IKFoM to window initialization

Implementation: `FastLio2IkfomFrontend::makeFixedLagInitializationSeed`, inside
the actual pinned IKFoM translation unit, not a detached layout mock.

The implementation queries each field using `MTK::getStartIdx` and checks the
pinned 23D layout: pos 0, rot 3, offset_R 6, offset_T 9, vel 12, bg 15, ba 18,
gravity S2 tangent 21. A mismatch fails the bridge.

Window local coordinates are `[right/body rotation, map position, velocity,
gyro bias, accel bias]`. Selector S chooses blocks `[rot,pos,vel,bg,ba]`:

```
Pxx = S P23 S^T
Pxg = S P23[:,gravity tangent]
Pcond = Pxx - Pxg solve(Pgg, Pxg^T)
Lambda0 = solve(Pcond, I15)
initial gradient = 0
```

Both solves use LLT; no production `matrix.inverse()` is used. The original
covariance must be finite and symmetric; gravity and conditional covariance
must be SPD. Fixed extrinsic cross blocks must be <=1e-12, otherwise the bridge
fails. Values are copied from the actual initialized snapshot, including stamp,
rotation, position, velocity, both biases and gravity.

This conditions on the initialized gravity estimate being fixed. It does not
integrate out uncertain gravity and does not preserve an equivalent 23D
posterior. Extrinsics remain fixed as required by the existing frontend. No
15D-to-23D posterior feedback is needed or claimed by this init-only producer.

The initialization test supplies genuine nonzero position/gravity,
rotation/gravity and position/velocity covariance blocks to the pinned filter,
compares the reordered conditional covariance with an independent reference,
checks information times covariance, all state fields and unchanged filter P.
Measured conditioning difference norm: 0.0309436. Release and Debug PASS.

## IMU noise mapping

| Window parameter | Actual RuntimeParameters field | Source field unit |
|---|---|---|
| gyro_noise_density | gyro_noise_std_rad_s | rad/s |
| accel_noise_density | accel_noise_std_m_s2 | m/s² |
| gyro_bias_random_walk | gyro_bias_rw_std_rad_s2 | rad/s² |
| accel_bias_random_walk | accel_bias_rw_std_m_s3 | m/s³ |
| gravity | initialized FilterSnapshot.gravity | m/s², map frame |

The direct numerical mapping follows the authorized protocol; no hidden default
0.01/0.10/1e-4/1e-3 substitution or sampling-rate conversion is applied. The
existing preintegrator consumes these scalars in its noise-density model. Source
field labels alone do not prove continuous-time spectral-density calibration;
statistical calibration and cross-sampling-rate equivalence are not claimed.

Static initialization consumes only samples not later than the handoff epoch.
If an explicitly requested epoch lies between IMU samples, held-input prediction
is allowed only during initialization, for at most two median sample periods.
The initializer is destroyed before the producer loop starts.
