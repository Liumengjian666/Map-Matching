# Window-owned deskew mathematics

State order: `[right-body rotation, map position, map velocity, body gyro bias,
body accelerometer bias]`, 15 dimensions; gravity is fixed. `T_imu_lidar`
maps LiDAR coordinates into IMU coordinates. No EKF object is used by deskew.

## One integration implementation

`window_imu_factor.cpp::integrateBoundedImuSequence` is the single bounded loop
called by both `preintegrateImu` and `integrateWindowImuTrajectory`. It retains
the original boundary interpolation, midpoint input average, left-knot rotation,
bias Jacobians, noise-density convention, covariance transition and PSD check.
For each interval dt:

```
omega = (gyro_a + gyro_b)/2 - bg_s
a     = (accel_a + accel_b)/2 - ba_s
Dp'   = Dp + Dv dt + 0.5 DR a dt²
Dv'   = Dv + DR a dt
DR'   = DR Exp(omega dt)
```

`propagateWindowState` is shared by the adapter and trajectory builder:

```
R(t) = R_s DR(t)
v(t) = v_s + g dt + R_s Dv(t)
p(t) = p_s + v_s dt + 0.5 g dt² + R_s Dp(t)
```

Biases are held at the re-queried scan-start state's values for this bounded
scan integration, as specified by A2D. This is the frozen discrete integration
model, not a claim of exact continuous-time integration under arbitrary motion.

## Geometry

`T_M_L(t) = T_M_I(t) T_I_L` and
`p_end = T_M_L(end)^(-1) T_M_L(point_sensor_stamp) p_sensor`.
Intermediate knot poses use the original linear-position / quaternion-slerp
interpolation. The pure geometry was extracted into `lidar_deskew_geometry.cpp`;
the old `ScanEndProcessor` calls that helper as well. Only geometry is shared:
the new processor does not instantiate, query or propagate IKFoM.

## Causal ownership

V3 schedules a real `LIDAR_SCAN_START` node. Later visual events can change it
through joint Window optimization. At scan end, `adapter.activeStateAt(start)`
reads the current optimized node, not a cached start snapshot. An expired start
fails with `WINDOW_SCAN_START_NOT_IN_ACTIVE_WINDOW`. IMU input is limited to a
left bracket and the first required right bracket; no future scan buffer is read.

Exact timestamp ties use an explicit priority function: scan start, scan end
(or legacy V2 scan), visual current, visual reference. Non-ties are ordered by
sensor timestamps. This preserves V2's historical tie contract.

All output is committed only after successful validation, integration and
geometry. A zero/missing point timestamp fails `RAW_POINT_TIME_UNAVAILABLE`;
XYZ-only data is never assigned guessed times.
