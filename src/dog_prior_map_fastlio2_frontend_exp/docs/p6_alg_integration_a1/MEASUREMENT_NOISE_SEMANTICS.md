# R4 measurement-noise semantics

`PHYSICAL_LIDAR_PERTURBATION` is the source model
`δ=[δθ_map, δt_normalized]` with covariance `Q_lidar`.  Its target residual
covariance is exactly

`R_residual = A_exact Q_lidar A_exactᵀ`.

The complete matrix is retained, including rotation/translation cross blocks.
`MeasurementNoiseModel::propagatePhysicalLidarCovariance` and the R4 test
exercise this path.

`EMPIRICAL_POSE_RESIDUAL` is already expressed in the target
`IKFOM_RIGHT_POSE_RESIDUAL` chart.  The adaptive covariance created by
`makePoseMeasurementNoise` is tagged with this semantic and
`statistically_calibrated=false`; it is not transformed through `A_exact`
again.  This is an engineering risk-weighting model, not a claim of calibrated
probability.

The R4 test verifies symmetry/PSD, non-zero lever-arm cross terms, zero
lever-arm identity behavior, Monte-Carlo propagation, and NIS invariance when
residual and covariance are transformed together.
