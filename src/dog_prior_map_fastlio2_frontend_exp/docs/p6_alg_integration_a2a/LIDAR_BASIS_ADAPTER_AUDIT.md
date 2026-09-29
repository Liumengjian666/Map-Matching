# LiDAR basis adapter audit

`FrozenLidarEvent` contains the frozen NDT terminal, residual covariance,
`LocalRisk`, convergence flag and map-support flag.  A valid event is converted
to `LidarWindowMeasurement` without a second NDT invocation.

For full rank, the measurement basis is the identity.  For a weak subspace, the
adapter finite-differences the existing pose residual with respect to the
normalized physical LiDAR perturbation chart at the current predicted
`WindowState`, then calls the existing
`buildReliableMeasurementBasisFromWeak()` routine.  The resulting callback is
installed on the factor and is invoked again by the window at each linearization
point.  Thus the outer mapping basis follows the current state while the
frozen NDT terminal itself is unchanged.

The adapter rejects rank zero, invalid risk, non-converged NDT and insufficient
map support.  It does not turn `hasConverged()` into a mathematical minimum
claim and does not recompute NDT curvature.
