# PCL NDT Hessian Convention — P6-I3 R1

## Audited PCL convention and Hessian sign

The installed PCL is 1.10.0. PCL's six-vector uses raw order `[tx, ty, tz, rx, ry, rz]`; the rotation is `Rx(rx) Ry(ry) Rz(rz)` with radians. NDT maximizes its scalar score. The canonical order is `[rx, ry, rz, tx, ty, tz]`; after the corresponding permutation, `H_euler = -sym(H_score_canonical)`. This is local negative score curvature at the converged score maximum, not Fisher information, a covariance inverse, or a universal observability matrix. Raw asymmetry and non-finite/negative curvature remain visible diagnostics; no eigenvalue clamp is applied.

## Fixed coordinate pipeline

The analyzed pipeline is exactly:

```text
PCL raw Hessian, [tx, ty, tz, rx, ry, rz]
  -> reorder to Euler canonical, [rx, ry, rz, tx, ty, tz]
  -> sign/symmetrize: H_euler = -sym(H_score_canonical)
  -> Euler increments to map-frame spatial tangent:
       A = blockdiag(J_spatial^-1, I3)
       H_phys = A^T H_euler A
       physical order [d_phi_x, d_phi_y, d_phi_z, dt_x, dt_y, dt_z]
  -> fixed dimensionless translation coordinate u=t/r, r=0.8 m:
       S = diag(I3, r I3)
       H_bar = S^T H_phys S
  -> RAW6 / BLOCK / SCHUR analysis, all on H_bar
```

`J_spatial` and its finite-difference validation are defined in `rotation_coordinate_convention.md`. Invalid/ill-conditioned Jacobian samples are marked invalid, not silently inverted. The translation scale is isotropic and fixed; no scale sweep is performed.

## Schur systems

Partition `H_bar = [[H_RR, H_Rt], [H_tR, H_tt]]`:

```text
S_R = H_RR - H_Rt H_tt^-1 H_tR
S_t = H_tt - H_tR H_RR^-1 H_Rt
```

The implementation solves the right-hand systems with LDLT, then column-pivoted QR as a recorded fallback. It uses no `matrix.inverse()`, regularization, eigenvalue clamping, DCReg threshold, or preconditioner. Fallbacks and failures are counted. BLOCK and SCHUR physical eigenbases use the DCReg-style one-to-one greedy axis matching and aligned eigenvalues; weak subspaces still use principal-angle agreement.

RAW6 remains one coupled 6D spectrum. Its projection is an evaluation-only comparison that uses the preregistered expected weak-subspace dimension; it is not an online unknown-dimensional U_obs estimator.

## Interpretation limits

This is analytic objective curvature from a specific PCL NDT implementation and configuration. Synthetic expected directions are hard labels for validation; sampled Floor01 has no ground-truth degeneracy labels. This analysis does not establish calibrated covariance, a runtime degeneracy detector, or completed Dual Reliability.
