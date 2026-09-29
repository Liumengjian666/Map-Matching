# PAPER-P6-ALG-INTEGRATION-A1 source and mathematics audit

Status: implementation audit for the A1 prototype (30 September 2026)

## Scope and frozen boundaries

The audit was performed against `4fe65d8c298bdd7a4a8f67e06fd3af7cfba69cb6`
in worktree `dog_loc_p6_i6b_ws`.  The stable ROS workspace
`dog_visual_loc_ws` and the formal `FORMAL_FULL_LEGACY` path were not changed.
The fixed-lag code is an independent library/test path; it is not silently
inserted into the public rosbag runner.

## Existing online call chain

1. `src/fastlio2_frontend_ikfom.cpp` implements
   `FastLio2IkfomFrontend::initializeStatic`, `predictInterval`,
   `predictImuSequence`, and `predictHeldInputTo`.  IMU samples are consumed
   in sensor-header time order.  The exact scan-end tail is held and propagated
   to the requested end stamp, so the state stamp is the scan-end time.
2. `FrontendRuntime::beginScan` in `src/frontend_runtime.cpp` clones the
   committed IKFoM state, predicts the candidate, and keeps the committed
   state untouched.  `finishScan` applies a terminal pose measurement to the
   candidate and commits it only after postconditions pass.  Rejects remain
   prediction-only and do not mutate the committed terminal.
3. The pose update implementations are
   `applyPoseMeasurement`, `applyPositionMeasurement`,
   `applyProjectedPoseMeasurement`, and
   `applyProjectedPoseMeasurementLinearizedChecked` in
   `src/fastlio2_frontend_ikfom.cpp`.  Pose rotation innovations use the
   pinned FAST-LIO2/IKFoM right/body SO(3) error convention; position is in
   map axes.  The checked projected API accepts a measurement basis and a
   covariance in that residual chart.
4. `src/dual_reliability.cpp` computes the reliability decision through
   `assessLocalRisk`, `buildReliableMeasurementBasisFromWeak`,
   `makePoseMeasurementNoise`, and `decideDualReliability`.  The local metric
   is the Schur-decoupled U_obs geometry from `reliability_metrics.cpp`.
   U_nonlocal contributes a finite +/- terminal-response warning covariance;
   it is not a basin/minimum proof.
5. `scripts/p6_i1_branched_recovery.cpp` is the closed-loop replay tool.  It
   calls one common decision pipeline for the configured modes and evaluates
   against GT only after a trajectory has been generated.
6. `p6_i6d_prepare_floor01_visual.py` freezes P4-I3 metric relative
   translations with causal reference-depth stamps and no GT columns.
   `p6_i6d_prepare_corridor01_visual.py` reads image/cloud `header.stamp`,
   ignores bag record time, enforces causal depth age, and exports translation
   only.  `p6_i6d_run.py` validates input identities, dispatches the existing
   `FULL_ALGORITHM_V1` runner, and checks that B4 performed real non-zero
   visual updates.  A1 does not alter these frozen preparation products.

## Existing coordinate and covariance semantics

The R3 pose residual chart is ordered `[map position XYZ, right/body SO(3)
residual]`.  The physical LiDAR chart is `[map-spatial rotation,
normalized map translation]`.  `A_exact` maps the latter into the former;
`J_l_inverse` and the exact log residual are implemented in the R3 math
headers (`scripts/p6_i6e_r3_math.hpp`, `scripts/p6_i6e_r3_actual_state.hpp`).

`decision.measurement_noise.covariance` is an engineering covariance already
in `IKFOM_RIGHT_POSE_RESIDUAL` coordinates.  A1 now tags it explicitly as
`EMPIRICAL_POSE_RESIDUAL`, with `statistically_calibrated=false`.  It must not
be passed through `A_exact` a second time.  A physical LiDAR covariance must
instead use `A_exact * Q_lidar * A_exact.transpose()` and retain the complete
rotation/translation cross blocks.

## A1 fixed-lag state and factors

The new independent API is in `include/.../window_factors.hpp` and
`include/.../fixed_lag_window.hpp`.

* `WindowState` is `{R_i,p_i,v_i,b_g,i,b_a,i}` with 15D local order
  `[right rotation, position, velocity, gyro bias, accelerometer bias]`.
  `R_i` maps IMU vectors to map coordinates; position and velocity are map
  vectors; biases and gravity are expressed in the IMU/body convention used
  by the preintegrator.
* `preintegrateImu` in `window_imu_factor.cpp` uses midpoint integration over
  the actual sample timestamps, including boundary interpolation and
  irregular intervals.  It propagates `Delta R`, `Delta v`, `Delta p`, the
  first-order gyro/accelerometer bias Jacobians, and a full 15x15 covariance
  with continuous gyro/accelerometer and bias random-walk noise.  The residual
  is the standard manifold IMU residual (rotation log, velocity, position,
  bias random walks) with covariance whitening deferred to normal-equation
  assembly.  `linearizeImuFactor` supplies central finite-difference Jacobians
  over the same right-perturbation chart.
* `window_lidar_factor.cpp` builds a 6D pose-chart residual
  `[p_i - p_meas, Log(R_meas^T R_i)]`.  It projects this residual through
  `measurement_basis`, checks the declared reliable rank and SPD covariance,
  and skips invalid/degenerate factors with a reason.  Rank-deficient factors
  require an upstream U_obs/R3 `basis_relinearizer`: it is called once at each
  outer factor linearization and its basis is frozen through the local
  central-difference Jacobian.  It does not reimplement PCL NDT; the map
  matcher remains the producer of the measurement.
* `window_visual_factor.cpp` implements the cross-state metric translation
  residual
  `r = p_j - p_i - R_i z_ij`.  Here `z_ij` is the reference-IMU to current-IMU
  metric translation in the reference body chart.  The analytic local
  Jacobians are `J_p_i=-I`, `J_p_j=I`, and
  `J_theta_i=R_i skew(z_ij)`; they were checked against central differences.
  The source semantic is required to be
  `METRIC_PNP_RELATIVE_TRANSLATION_FACTOR`.
* `fixed_lag_window.cpp` assembles one joint objective containing the retained
  prior, each unique IMU factor, accepted LiDAR factors, and accepted visual
  factors.  Observation IDs are deduplicated across all factor families, so a
  terminal measurement cannot be counted twice.  LM-style damped normal
  equations are accepted only when the objective decreases.  The numerical
  Hessian rank is reported, not interpreted as a global observability proof.
* `window_marginalization.cpp` removes old states with a Schur complement
  solved by LDLT.  Retained cross-state information is kept in the prior;
  a tiny solve-only jitter is allowed for a gauge-like oldest block but is
  never stored as information.  The window is bounded by 2 seconds and 48
  nodes.

## Prediction/optimization feedback

`fixed_lag_experiment.hpp` defines the explicit mode boundary.  The formal
mode `FORMAL_FULL_LEGACY` rejects fixed-lag operations, while
`FULL_FIXED_LAG_EXPERIMENTAL` owns one `FixedLagWindow`.  Its
`predictionFeedbackSeed` returns only the latest optimized state.  The
existing IKFoM frontend exposes `setWindowPredictionSeed` for a future
explicit integration point; it validates stamp monotonicity, finiteness,
covariance PSD, gravity, and fixed extrinsic invariants before changing the
state.  No ROS callback currently calls this setter, so online feedback is
not claimed as complete.

## Verification and limits

The A1 tests cover R4 covariance semantics, IMU preintegration, visual
Jacobian finite differences, a 9-node joint window, duplicate rejection,
rank diagnostics, Schur marginalization, retained cross information,
feedback-seed export, and a full-batch versus incrementally marginalized
window comparison.  No full public dataset or rosbag was run.  Global
relocalization remains explicitly `NOT_VERIFIED_GLOBAL_RELOCALIZATION`.

`READY_FOR_FORMAL_EXPERIMENT = NO`.
