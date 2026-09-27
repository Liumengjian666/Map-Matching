# PCL NDT Hessian Convention

## Audited implementation

- Installed library: PCL `1.10.0` (`libpcl-dev 1.10.0+dfsg-5ubuntu1`).
- Source audited locally: `/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp` and `ndt.h`.
- PCL's `computeTransformation` extracts translation from the current transform and obtains Euler XYZ angles using `rotation().eulerAngles(0,1,2)`. Its 6-vector is `p = [tx, ty, tz, rx, ry, rz]`; angles are in radians. Its incremental transform is translation followed by `AngleAxis(rx,X) * AngleAxis(ry,Y) * AngleAxis(rz,Z)`.
- `computeDerivatives` returns the scalar NDT probability score, its gradient, and the score Hessian with respect to that translation-first Euler vector. NDT solves `H_score * delta = -gradient`; the line search uses `phi=-score` and `dphi=-gradient·direction`, so the optimizer maximizes score (equivalently minimizes negative score).

## Canonicalization and sign

Canonical order is `[rx, ry, rz, tx, ty, tz]`. Let `P` reorder raw indices `[3,4,5,0,1,2]`:

```text
H_score_canonical = Pᵀ * H_score_raw * P
H_information_raw = -0.5 * (H_score_canonical + H_score_canonicalᵀ)
```

The minus sign converts local score curvature at a maximizing mode into a local information/negative-curvature convention. This does not guarantee positive semidefiniteness: negative eigenvalues, failed convergence, and non-finite outputs remain visible diagnostics, not silently clamped values. The pre-symmetrization asymmetry is retained as a numeric diagnostic.

## Fixed unit normalization

Keep `H_information_raw` in rad/m coordinates. With the mandated NDT resolution `r=0.8 m`, use exactly:

```text
D = diag(1, 1, 1, 1/r, 1/r, 1/r)
H_bar = Dᵀ * H_information_raw * D
```

No normalization parameter is tuned. All RAW6, BLOCK, and SCHUR comparisons use `H_bar`; the unscaled information matrix remains available in the output for audit.

## Schur solve policy

The two 3x3 Schur systems use LDLT first and column-pivoted QR as the recorded fallback. No diagonal regularization, eigenvalue clamping, or silent pseudoinverse is applied. If both solves fail the Schur component is marked non-finite and the failure is counted; conditioning estimates, solve method, fallback count, and factorization-failure count are emitted per sample.

## Limitations

This is PCL's analytic objective curvature, not DCReg's ICP `JᵀJ`, a calibrated covariance, or a universal physical observability scale. It is analyzed only at a converged local NDT mode under the fixed PCL configuration; no eigenvalue threshold or runtime trigger is introduced.
