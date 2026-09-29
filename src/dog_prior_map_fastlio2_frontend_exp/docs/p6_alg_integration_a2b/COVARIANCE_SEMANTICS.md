# Covariance semantics

`ImuPreintegratedMeasurement::physical_covariance` preserves the covariance
produced by physical preintegration. `covariance` is the factor-whitening
covariance. When the rank-deficient physical covariance cannot be Cholesky
factored, only the latter receives a `1e-9 I` floor.

The event adapter records:

- `factor_covariance_regularized`;
- regularization magnitude;
- physical and factor minimum eigenvalues;
- norm of the change from physical pseudoinverse information to regularized
  factor information;
- status `NUMERICAL_FACTOR_COVARIANCE_REGULARIZATION_1E-9`.

This is factor-level numerical regularization, not physical sensor calibration or
solve-only damping. The physical covariance remains available unchanged.

LiDAR covariance follows the R4 window residual coordinates
`[p_m-p_i, Log(R_i^T R_m)]`. Directional projection applies the same basis to the
residual, Jacobian, and covariance.
