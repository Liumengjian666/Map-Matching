# PAPER-P6-ALG-INTEGRATION-A1 summary

## Result

`R4_MEASUREMENT_CONSISTENCY = IMPLEMENTED_AND_TESTED`

`I6F_FIXED_LAG_FOUNDATION = SYNTHETIC_MATH_AND_WINDOW_TEST_PASS`

`READY_FOR_FORMAL_EXPERIMENT = NO`

## CONFIRMED_MATH_IMPLEMENTATIONS

* Physical LiDAR covariance uses `A_exact Q_lidar A_exact^T`, retaining
  rotation/translation cross covariance.
* Existing adaptive decision covariance is explicitly tagged empirical in the
  IKFoM right/body residual chart and is not re-propagated.
* IMU preintegration handles irregular timestamps, boundaries, bias Jacobians,
  and full 15x15 covariance propagation.
* LiDAR factor is rank/SPD/degeneracy aware and uses a supplied reliable basis.
  Rank-deficient bases are rebuilt at every outer linearization and frozen for
  the inner Jacobian.
* Visual factor is a real cross-state metric translation residual with tested
  right-SO3 Jacobian; visual rotation is not fused.
* Fixed-lag joint objective, duplicate-ID rejection, Schur marginalization,
  retained cross information, and explicit prediction feedback seed are
  implemented.

## CONFIRMED_RUNTIME_INTEGRATIONS

* `FULL_FIXED_LAG_EXPERIMENTAL` is an explicit configuration/mode boundary.
* `FORMAL_FULL_LEGACY` remains the default and rejects experimental window
  operations.
* `setWindowPredictionSeed` provides a validated future adapter boundary.

The ROS runtime does not yet construct or feed the fixed-lag window, so no
online algorithm improvement is claimed.

## Test status

Standalone Release build, the direct catkin-package FAST-LIO2 build, and both
five-test CTest suites passed.  The workspace-level localization Catkin build
could not configure because `livox_ros_driver2` is not installed in this
checkout; this is recorded in `BUILD_AND_CTEST_RESULTS.txt`.  See
`BUILD_AND_CTEST_RESULTS.txt` and the individual test notes.  No full rosbag
or public dataset was run.

## UNFINISHED_ALGORITHM_COMPONENTS

* Production adapter from scan transactions/IMU/visual frontend into the
  window is unfinished.
* Marginalized prior relinearization policy and robust loss policy are still
  prototype-level.
* LiDAR residual covariance calibration and map-support statistics require
  real-data validation.
* Vision metric-scale and timestamp/extrinsic provenance must be supplied by
  a real metric frontend before enabling visual factors.
* Global relocalization is not implemented or verified.

## KNOWN_APPROXIMATIONS

The optimizer uses dense normal equations and finite-difference factor
Jacobians for the IMU/LiDAR audit path, damped Gauss-Newton/LM acceptance,
fixed noise parameters, and a
solve-only marginalization jitter for gauge-like blocks.  These choices are
explicitly logged and are not formal convergence or global-minimum proofs.

## CURRENT_BLOCKERS

The missing production runtime adapter, real-data covariance calibration,
and relocalization validation block any formal experiment.  They do not
invalidate the synthetic mathematical tests.
