# Reused visual motion and limited NDT integration

P4's functions `load_calibration`, `rectify`, `estimate`, `associate_depth`
and `pnp` are reused unchanged. This work is compatibility/integration, not
a new visual algorithm. The former P6 482-row input is not used.

For a firing at t, transport raw LiDAR coordinates into the reference image's
sensor-local LiDAR frame by

`p_Lref = R_IL^T [R_Iref_It (R_IL p_Lt + t_IL) - t_IL]`.

The rotation comes exclusively from raw gyro, with the pre-existing midpoint
integrator and the same official projected SO(3) LiDAR extrinsic. The image
is rectified using the official MEI calibration. No map pose, NDT correction,
publication decision or GT supplies the depth. Unknown translational motion
is deliberately not compensated; therefore depth is approximate despite
its independent source. Zero gyro bias is an unestimated model assumption.

PnP transforms reference camera points to current camera coordinates. Its
inverse, conjugated by `T_imu_camera`, is the physical IMU relative motion.
The short image/scan timing bridge in PROTOCOL.md rotates the visual average
translation velocity with actual gyro and includes the LiDAR lever arm.
It is not claimed exact under acceleration. It does not introduce an EKF,
VIO optimizer, new fusion weight or long-term gyro/CV localization chain.

The runtime adapter replaces only translation of `initial_guess`, when exact
previous/current scan stamps match a valid row and the original IMU rotation
prior succeeded. Missing/invalid rows leave the guess unchanged. Existing
NDT align, step limit, published measurement and EKF/OOSM update are retained.
Whether this actually improves an initial guess or localization requires the
unfinished causal replay and post-freeze GT evaluation, not PnP success alone.

The optional geometry log uses `coupledNominalJet` with the existing NDT
object, source and target. It includes the existing map-product pullback
second-order terms. Its eigenbasis is diagnostic only; it cannot affect
visual selection. Two synthetic consecutive aligns with/without native
derivative calls passed exact transformation/convergence/iteration/fitness
parity. No real Corridor geometry claim can be made from that unit test.
