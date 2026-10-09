# Reused R6 algorithm, new scan-to-map orchestration

This benchmark does not introduce a new objective, optimizer, Anchor rule,
measurement covariance or filter. It reuses the existing R6 implementation,
including its rejection paths, and records independent causal scan-to-map
pose recurrences. Full IKFoM state propagation and correction are NOT RUN.

## Unchanged refinement

The dimensionless map-product chart has translation divided by 0.8 m and a
left SO(3) rotation carrier. At the nominal terminal, the complete pullback is

`H = J^T H_native J + sum_a g_native[a] K_a`.

The positive-curvature eigenbasis splits `eta = W u + S v`; the existing
eigenvalue ratio chooses k=1 or k=2. Nonpositive nominal curvature rejects the
refinement rather than fabricating a valid subspace. Rho is unchanged:

`rho = max(mean(positive weak eigenvalues), 1e-4)`.

Stable LLT solves form the regularized weak Schur system without an explicit
inverse. Weak displacement is bounded by 0.15 m / 2 degrees. For C, a second
joint jet at the displaced weak point uses the **same** dynamic PCL objective
and the complete product-chart second derivative; one bounded conditional
strong correction is evaluated. B omits this step. No Newton20/full extra
align is introduced. Original R6 quality/total-bound guards remain decisive.

The C-arm records its own pre-strong weak point for a paired objective
comparison. A candidate-stage `strong_selected` flag is not sufficient for
execution: final `feedback=1` must also hold. Independent B/C history diverges
after feedback; cross-arm later differences are not all single-frame strong
correction effects.

## Common source and causal prediction

For `E=T_imu_lidar`, scan-end rotational compensation uses

`p_Lend = R_E^T (R_Iend_Itime (R_E p_Ltime + t_E) - t_E)`.

Midpoint gyro samples are restricted to current scan end; no predicted future
or registration result enters the common source. The extrinsic lever arm is
retained. Provisional gyro bias is zero, **not estimated**. Scan translation
deskew is absent. Each arm receives byte-identical prepared source count/hash.

Prediction converts the previous executed LiDAR pose to IMU origin using
`T_map_imu = T_map_lidar E^-1`, applies causal gyro rotation and constant
velocity from prior accepted IMU-origin positions, then returns through E.
This is a minimal gyro/CV predictor, not a certified full inertial state.
Unchanged R6 Anchor therefore supplies only a diagnostic gyro/CV reference.

## What the experiment can establish

Strict successful optimization, failure-to-success transitions, finite pose
recurrence, common objective identity, actual conditional correction and
computation costs can be compared. A lower NDT objective or fewer failed
optimizations does not establish correct global location in a repeated corridor.

Absolute localization error requires an independently closed map/GT transform,
which available evidence does not yet supply uniquely. No GT is loaded, no
per-frame fit is performed, and no absolute RMSE or true recovery count is
manufactured. This preserves the distinction between executable engineering,
convergence robustness and validated positioning accuracy.
