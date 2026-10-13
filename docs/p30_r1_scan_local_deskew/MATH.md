# Scan-local continuous IMU motion

The geometric convention is unchanged: `T_relative=T_ref_lidar^-1*T_point_lidar`,
where `T_world_lidar=T_world_imu*T_imu_lidar`. The original production helper
`transformPointToReferenceLidar()` is unmodified, including its lever arm.

One finite snapshot with `stamp<=scan_start` and age<=the existing20ms gate
provides p,v,R,ba,bg. Raw IMU history must have strictly increasing finite
timestamps, bracket the start and cover the end, with existing20ms gap checks.
A copied anchor is causally propagated to start if needed. Each subsequent
node comes only from this anchor and raw IMU, NEVER from a later EKF pose.
Known IMU boundary timestamps establish coverage, but post-end measurement
values never enter propagation. Point queries also require source_stamp<=point.

Reuse `propagateImuKinematics()` with the EFFECTIVE P28 mature convention:

    omega = gyro_head-bg
    R_next = R*Exp(omega*dt)
    a_world = R_next*(acc_head-ba)+g_world
    p_next = p+v*dt+0.5*a_world*dt^2
    v_next = v+a_world*dt

This is causal ZOH/end-rotation nominal integration, not an exact continuous
acceleration solution. Bias/gravity/extrinsic and filter noise are unchanged.
It matches `propagateMsckfImu()` per-step nominal mathematics. The old midpoint
flag and default gravity-alignment flag are unused by that mature propagator;
scan-local uses its effective convention without modifying the filter itself.
The old per-point Deskew reconstruction did use its midpoint flag. Thus this
experiment includes an integration-convention difference within the single
trajectory-construction variable; it is NOT a pure visual-rewrite-only switch.

Queries copy a cached preceding node and use a causal fractional ZOH interval,
without updating the stored nodes. Result is independent of query/point order.
Approximately20 nodes per scan, no covariance propagation or extra filter update.
Scan-start state may legitimately include earlier visual/NDT updates; equal
Source across visual ON/OFF is neither expected nor imposed.

Directed tests passed: stationary; constant velocity; constant angular rate;
combined rotation/acceleration vs0.1ms reference (p<3e-5m,v<3e-4m/s over100ms);
common-left SE(3) with transformed velocity/gravity and fixed body extrinsic;
future IMU and later EKF-state poisoning; partial image-clone anchor; reversed
query order; out-of-range, duplicate and unordered timestamps. Existing old
Deskew contract test passes. Default-off source/formulas are retained; no
default-off ROS trajectory parity replay was performed or claimed.

Selected-scan diagnostic queries compare local and legacy motion on the SAME
CURRENT C history. Boundary residuals propagate the1us left limit to the node
and compare with its exact pose; they are not adjacent EKF position jumps.
Residuals also contain midpoint/ZOH and camera subdivision conventions, NOT
only visual corrections. These diagnostics do not reconstruct P29 history or
use GT, and never change the published point cloud or filter state.
