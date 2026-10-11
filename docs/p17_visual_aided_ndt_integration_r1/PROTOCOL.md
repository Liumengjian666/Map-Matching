# P17 visual-aided NDT: pre-run contract

Start: `5453aec8ad6158d41716ca3b6c5f805f1381c96d`. P10 fixed-lag V2
396-scan input, map, initialization, deskew, NDT/EKF parameters remain fixed.
No velocity-feedback ablation. No historical P6 visual measurements are used.

Reuse P4 MEI rectification, Shi–Tomasi/KLT/FB, depth association and PnP
without changing their parameters. Image selection is latest sensor header
at or before each scan-start, never nearest/future. Raw bag record timestamps
are not sensor timestamps. Depth uses the latest *completed* raw scan before
the reference image. Its points are transported to that image by midpoint
raw-gyro rotation and the official LiDAR–IMU lever arm. IMU samples after the
image are excluded, terminal hold <=10 ms, real gaps <=20 ms. Gyro bias is
not estimated by this module. Translation deskew is unestimated, not silently
supplied by NDT/GT. Depth age <=150 ms; image-to-scan lag <=50 ms on both
ends; positive image interval <=250 ms. These are bounded timing interfaces,
not GT-tuned quality thresholds. Invalid intervals fall back to Control.

PnP returns `T_Ccur_Cref`. Convert using the official camera extrinsic:
`D_imu = T_imu_camera * inverse(T_Ccur_Cref) * inverse(T_imu_camera)`.
For the short image/scan endpoint mismatch, assume constant mean inertial
translation velocity *measured by that PnP interval*, not a known filter
velocity. Rotate it using actual gyro transport from reference image to
previous scan; scale translation by scan_dt/image_dt. This is an explicit
short-interval approximation, NOT full inertial propagation or VIO. Compose
the LiDAR lever arm with the gyro rotation over the actual scan interval:
`d_L = R_IL^T [R_Iref_Iprev^T d_imu * scan_dt/image_dt + (R_scan-I)t_IL]`.
Candidate translation is `p_previous_used + R_previous_used*d_L`. Keep the
existing live IMU rotation prediction exactly unchanged. No extra align.
Offline extraction may use all recorded data for IO, but every accepted row
has source point, image and IMU sensor timestamps <= its decision time.
Precomputed vision is NOT claimed to be a real-time camera frontend.

Default-off runtime adapter requires exact previous/current scan timestamps,
finite translation and independently validated timing bounds. Missing or
invalid rows restore the original initial guess. No EKF/OOSM, limitNdtStep,
NDT objective, noise, covariance, Anchor or R6 modifications.

After unit tests: short disabled/vision smoke, then one full visual-feedback
replay at the frozen 0.25x playback rate and 150 ms fixed sensor-time lag.
Read-only Hessian diagnostics reuse the current NDT source/target/backend;
never influence selection. Freeze outputs before GT evaluation. Primary
evaluation uses the original Control V2 0–3 s full-pose transform, 3–35 s
relative-drift metrics; no new alignment. Floor/Corridor are development data.
