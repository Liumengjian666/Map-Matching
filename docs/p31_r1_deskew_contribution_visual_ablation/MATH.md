# Three Deskew trajectory modes

All modes keep `T_relative=(T_world_imu_ref*T_imu_lidar)^-1 *
(T_world_imu_point*T_imu_lidar)`, original ordered points/times, gravity,
extrinsic and safety bounds. No new covariance or NDT update is introduced.

LEGACY_HISTORY: both experimental flags false. `interpolateImuDeskewPose()`
queries corrected EKF history; the global config is passed unchanged.
Original midpoint/tail-horizon behavior and gravity-alignment heuristic remain.

HISTORY_NOMINAL_CONSISTENT (D): scan-local false, new history-consistency flag
true. The SAME historical query function remains. Exact stored nodes are
returned unchanged; fractional nodes start from their preceding historical
snapshot. A LOCAL copied config disables midpoint and continuous alignment,
so fractional propagation uses causal head input and the mature nominal model:

    R_next = R * Exp((gyro_head-bg)*dt)
    a_world = R_next*(acc_head-ba)+g
    p_next = p+v*dt+0.5*a_world*dt^2
    v_next = v+a_world*dt

There is no `buildScanLocalImuTrajectory()` call in D. Historical corrections
can still appear at successive nodes. Global flags/noise/gravity and filter
propagation/history are never mutated. This ablates the two local convention
changes together, not midpoint vs gravity heuristic individually.

SCAN_LOCAL (C/E): scan-local true, new flag false. Reuse P30's one causal
scan-start state and continuous raw-head/ZOH trajectory. P30 already forces
midpoint=false inside its builder, so passing the explicit local false/false
copy does not change that trajectory. Later historical nodes are not used.
The legal anchor can include preceding visual updates. It is not a visual-OFF
trajectory or an assertion that C/E Source must match.

E disables only `mature_visual/enable`. `mature_visual/time_adapter_enable`
stays true, preserving image-time subdivision, clones and cross-covariance,
nominal IMU/right-error propagation, square-root storage and OOSM scheduling.
`processMsckfLocked()` returns DISABLED before measurement/state correction.
Old PnP initial guess and CI updates remain disabled in both D/E.

The modes are mutually exclusive; enabling both flags throws before playback.
Default-off remains legacy. Directed tests establish these branch properties,
including admitted-tail poisoning and no global/history mutation. No new
default-off full replay is run or claimed. Core trajectory/transform functions,
IMU/MSCKF/EKF mathematics and NDT are byte-unchanged from P30.

B→D, D→C and E→C change closed-loop histories. RMSE differences are descriptive
conditional contrasts, not independent additive causal fractions. No GT enters
motion construction or parameters; one frozen Control V2 transform is shared.
