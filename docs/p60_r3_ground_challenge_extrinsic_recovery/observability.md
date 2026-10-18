# Bounded calibration feasibility and counter-evidence

## What is and is not measured

This is one IMU-only motion-excitation probe of the existing internal bag.
No LiDAR odometry, calibration optimization or new NDT registration was run.
We cannot manufacture full joint observability or calibration uncertainty from
IMU-only data. LiDAR motion and its uncertainty, gyro/accelerometer biases,
gravity, temporal offset and map/scene constraints would be joint nuisance states.

Gyro is in rad/s in the reported Xsens frame. Relative seconds are formed by
integer subtraction before float conversion. A 100 Hz analysis grid and fixed
0.21 s Savitzky–Golay filter are used only for diagnostics, never written to input.
Windows 0–30, 30–60, 60–end and first5s were fixed before metrics. First5s is
**not assumed stationary**. Raw metrics are retained as a counter-check.

For small gyro-alignment rotation error, sensitivity is `-[omega_I]x`.
Its mean Gram has units `(rad/s)^2`. This is not a full extrinsic Fisher matrix.
Real angular motion satisfies `omega_I = R_I_L omega_L` if clocks are aligned.
No `omega_L` has been estimated/verified here, so no rotation solution is claimed.

Let `t_I_L` point from I to L in I axes. Rigid-body acceleration gives:

`a_L^I - a_I = ([alpha_I]x + [omega_I]x^2) t_I_L = J_p t_I_L`.

`J_p` has units s^-2; its mean Gram has units s^-4. It assumes available true
co-timed angular acceleration and point accelerations, neither of which a noisy
gyro derivative alone guarantees. No rank threshold was used to accept a calibration.

## Exact planar special case vs measured finite data

In ideal single-axis rotation `omega = w*k`, `alpha = a*k`:
`[omega]x*k = 0` and `J_p*k = 0`. Rotation about k is ambiguous in gyro-only
alignment; lever arm along k is invisible in this acceleration design. Further
gravity/ground constraints can change the rotation conclusion, so this is not
a universal theorem that every planar estimator loses the same rotation DoF.

Real Corridor2 is **not exactly this model**. Smoothed gyro principal energy is
98.0486% along `[0.044356,0.007998,0.998984]`. Rotation sensitivity ratio is
0.0196305, while lever sensitivity ratio is 0.724541 (numerically full rank).
Raw lever eigenvalues `[1402.984,1757.406,3074.550]` fall to
`[0.389767,0.417953,0.537950]` after fixed smoothing. Substantial high-frequency
gyro derivative energy makes interpreting the raw matrix as reliable lever-arm
information especially unsafe. It could include real rig vibration; it is not
proven to be sensor noise. Smoothing may also remove real useful motion.

Auxiliary device orientation roll/pitch range is 0.693764°/0.969363°, but device
internal filtering is not independent validation of LiDAR motion. Therefore:

- Dominant planar/yaw excitation: supported.
- Strict full-sequence mathematical rank deficiency: **not established**.
- Full rotation and 3D lever-arm observability with quantified uncertainty: **not certified**.
- Inter-sensor temporal lag: **not estimated**, not set to zero.
- No particular extrinsic DoF is reported as reliably estimated.

This is why the final label is `EXTRINSIC_SOURCE_NOT_FOUND`, not an overstated
`EXTRINSIC_UNOBSERVABLE` or a fabricated calibrated result.

## Bounded tool feasibility

[LI-Init official README](https://github.com/hku-mars/LiDAR_IMU_Init) and
[paper](https://arxiv.org/abs/2202.11006) require adequate excitation and estimate
time/extrinsic jointly. No installation or solver run was justified by this
IMU-only feasibility check and missing producer/time identity. Success cannot
be inferred from the numerical full rank of a raw gyro Gram.

[GRIL-Calib official paper](https://arxiv.org/html/2312.14035), section V-B,
requires **measured IMU ground height** as a prior to constrain the planar
unobservable component. Its [example config](https://github.com/Taeyoung96/GRIL-Calib/blob/master/config/velodyne16.yaml)
contains `imu_sensor_height: 0.1`, an example for another setup, not a measurement
of Ground-Challenge. The platform photograph is not a certified height measurement.
No such trusted numerical prior was recovered in this bounded source set.
Thus GRIL-Calib cannot be honestly run by copying its example height.

## Candidate validation status

No candidate `T_I_L` exists. SO(3), inverse, held-out LiDAR/gyro consistency,
calibration uncertainty and full-6DoF approval are all N/A / NOT VALIDATED,
not zero errors or implicit passes. Nothing is fed into Deskew or EKF.
