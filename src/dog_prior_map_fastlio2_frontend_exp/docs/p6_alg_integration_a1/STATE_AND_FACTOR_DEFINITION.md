# State and factor definition

For node `i`,

`X_i = (R_i, p_i, v_i, b_g,i, b_a,i)`.

The local increment is
`δx_i = (δθ_i, δp_i, δv_i, δb_g,i, δb_a,i)` with
`R_i <- R_i Exp(δθ_i^)`.  The 15 columns are ordered
`[δθ,δp,δv,δb_g,δb_a]`.

IMU factors use midpoint preintegration between real sample timestamps and
include the requested bias Jacobians and full covariance.  LiDAR factors use
only the reliable measurement basis supplied by U_obs/Schur; rejected rank or
non-SPD inputs are recorded rather than silently filled.  Visual factors use
the cross-state residual

`r_v = p_j - p_i - R_i z_ij`,

not an absolute map pose and not an independent per-frame position prior.
Only one observation ID may contribute one factor across the window.
