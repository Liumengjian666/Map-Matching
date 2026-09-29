# Source/math correspondence

The production implementation is
`include/dog_prior_map_fastlio2_frontend_exp/lidar_residual_math.hpp`.
`scripts/p6_i6e_r3_math.hpp` now delegates to it, preventing the R3 audit helper
and adapter from carrying independent formulas.

For residual

`r = [p_m - p_i, Log(R_i^T R_m)]`,

and normalized LiDAR perturbation `[dphi, du]`, where physical translation is
`l du`, the implemented exact mapping is

```
A = [ skew(R_m t_il)              l I ]
    [ J_l^-1(phi) R_i^T             0 ]
phi = Log(R_i^T R_m).
```

The top-left block includes the IMU-to-LiDAR lever arm and the bottom-left block
uses the same SO(3) log operand order as the window LiDAR residual. Residual
covariance is interpreted in that same `[map position, local SO(3) log]` order;
the reliable basis is applied consistently to residual, Jacobian, and covariance.

Near-pi residuals are rejected because the selected log chart/Jacobian inverse is
ill-conditioned there. Runtime basis reconstruction is analytic; central finite
differences exist only in the test executable.
