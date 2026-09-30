# Analytic Jacobian audit

Runtime **IMU and LiDAR state Jacobians are analytic**. Independent central-FD
functions retain the original 1e-7 perturbation as TEST_ONLY_REFERENCE; runtime
assembly never calls them. The fixed marginal-prior SO3 chart still uses its
existing, separately audited FD derivative.

## IMU derivation

State retraction: `R' = R Exp(dtheta)`; p/v/bg/ba increment additively.
Let `beta=J_R_bg (bg_i-bg0)`, `C=DR Exp(beta)`,
`phi=Log(C^T R_i^T R_j)`, and `L=J_l(phi)^-1`, `U=J_r(phi)^-1`.
Nonzero blocks (row order R,p,v,bg,ba; column order theta,p,v,bg,ba):

```
J_i(R,theta) = -L C^T
J_j(R,theta) = U
J_i(R,bg)    = -L J_r(beta) J_R_bg
J_i(p,theta) = skew(R_i^T(p_j-p_i-v_i dt-0.5 g dt²))
J_i(p,p)     = -R_i^T              J_j(p,p) = R_i^T
J_i(p,v)     = -R_i^T dt
J_i(p,bg)    = -J_p_bg             J_i(p,ba) = -J_p_ba
J_i(v,theta) = skew(R_i^T(v_j-v_i-g dt))
J_i(v,v)     = -R_i^T              J_j(v,v) = R_i^T
J_i(v,bg)    = -J_v_bg             J_i(v,ba) = -J_v_ba
J_i(bg,bg)   = -I                  J_j(bg,bg) = I
J_i(ba,ba)   = -I                  J_j(ba,ba) = I
```

The `J_r(beta)` term is essential for the nonlinear bias correction; it is not
approximated by identity. `J_l(phi)^-1 = I - 0.5[phi] + c[phi]^2`,
`c=(1-0.5 theta cot(theta/2))/theta²`, with the small-angle series.

## LiDAR derivation

Raw residual: `[p_measured-p, Log(R^T R_measured)]`.
Its Jacobian has `-I` in position columns and `-J_l(phi)^-1` in rotation columns.
At each outer iteration the reliable basis is rebuilt, then frozen for the
local derivative exactly as before: `r_selected=Q^T r`, `J_selected=Q^T J`,
`R_selected=Q^T R Q`. No derivative of the outer basis is introduced.

## Evidence and domain

Every column of both IMU endpoints is compared against the independent old FD
oracle with nonzero pose/velocity/bias residuals, nonzero bias correction, small
and large rotations, and a 3.1413 rad near-pi case. Maximum IMU error:
**3.95194e-9**, below the predeclared 2e-7 test bound.
LiDAR full/projected and near-pi cases: **2.19452e-9**, same bound. Residual and
noise covariance equality are checked independently.

Exactly at the principal SO3 Log branch cut, a unique derivative does not exist.
The implementation reports `imu_rotation_log_branch_cut` or
`lidar_rotation_log_branch_cut` instead of silently selecting an FD branch.
There is no broad 1e-4 near-pi rejection band. Residual/objective definitions
and research acceptance thresholds were not changed.

A fresh-context read-only mathematical review found no sign, frame or bias
correction error; it specifically identified the near-pi guard risk before
implementation. A separate review checked solver/provenance wiring. External
cross-model review is reserved for the user's final manual handoff.
