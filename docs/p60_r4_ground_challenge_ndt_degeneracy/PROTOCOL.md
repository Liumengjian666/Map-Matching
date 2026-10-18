# P60-R4 preregistered geometry screen

Frozen before any R4 align: 100 nearest-in-time source indices to an evenly
spaced grid from scan2 header to scan974 header, earliest tie; target strictly k-1.
Input bag SHA256: 6ef85c76a36bdcef4811d411ac0f09219803377f5b7b9bd0cf2a5258cc826cea.
No pseudo-GT, IMU messages, Deskew, EKF, extrinsics, HIKVISION or online localization.
This is RAW_ADJACENT_SCAN_REGISTRATION_DIAGNOSTIC, not prior-map accuracy.

## Frozen processing and methods

- Raw XYZ from bag; finite target XYZ; production target VoxelGrid 0.15m twice.
- Source: production finite/range [0.5,80]m filter, no height cut, VoxelGrid
  0.25m, production rounded evenly spaced point cap1400.
- A FULL_360 = this complete preprocessed Source.
- B FOV_90 = subset of A with x>=0 and abs(y)<=x (fixed +/-45 degrees about
  **scanner frame +X**, inclusive). Not a certified robot-forward FoV. Crop
  AFTER the common production preprocessing/cap to isolate direction and count.
- C POINT_COUNT_MATCHED = sample exactly len(B) unique points from A using
  std::mt19937 seed=6042026 + original scan index; std::shuffle indices, retain
  first len(B), sort selected indices to preserve original point order.
- All share identical full target, identity initial guess in target scanner
  coordinates, PCL1.10 NDT resolution0.8m, step0.08, max_iterations40,
  transformation_epsilon0.001, native default outlier_ratio0.55. No state
  propagation between pairs, post-match step limit or candidate refinement.
- Exactly one align per frame/group (300 total maximum). First A (k=2) is
  checked against the existing production R2 first output, without extra align.
- Primary Hessians for A/B/C are evaluated at the SAME A raw result. Secondary
  Hessians at each group's own result are separately labeled; never pooled.
- Exact existing ObservablePclNdt and coupledNominalJet *derivative-only* helper
  reused. No Coupled candidate/refinement functions invoked.
- Mean energy = -native_score_sum/N; chart p=p0+0.8 eta_t [m],
  R=Exp(eta_r [rad])*R0. Hessian includes nonlinear carrier's gradient term.
  Also report total curvature N*H to distinguish normalization from point loss.

## Validity, analysis and limits

Minimum source points30; valid rigid finite A reference; finite native jet,
nonzero score and >=50% source points with valid Gaussian-neighbor support
at the common pose. These fixed overlap safeguards do not alter align results.
Finite indefinite Hessians are retained as legitimate nonconvex diagnostics,
NOT renamed positive information matrices or assigned artificial positive ratios.
Positive-definite comparisons are reported separately. Insufficient comparable
triplets (<80) stops scientific inference and yields GEOMETRY_SCREEN_INSUFFICIENT.

All 100 pairs and failures retained. Primary continuous comparisons: six-DoF and
translation-block min/max ratios, smallest eigenvalues, log-condition numbers,
sign-invariant weakest-axis similarity, matched B/C point counts, support.

Conservative descriptive screening labels, not production triggers or universal
physical thresholds: full-SPD six-DoF ratio<=1e-3 and translation-block ratio<=1e-2.
Three successive *sampled* pairs with this label and adjacent translation weak-axis
similarity>=cos20deg give a native weak-direction candidate interval (unsampled
frames are not certified). This can support NATIVE_DEGENERACY_EVIDENCE_OBSERVED,
but cannot prove motion error or optimizer failure.

Otherwise FoV-specific support requires >=80 SPD comparable triplets, a median
B/C six-DoF ratio<=0.5, B lower than C in >=75% of those triplets, and >=20 B
pairs meeting both weak screening labels while C does not. Label only
FOV_INDUCED_DEGENERACY_OBSERVED / INDUCED_FOV_DEGENERACY. Else
DEGENERACY_NOT_DEMONSTRATED, reporting continuous trends and counter-evidence.
No parameters or criteria will change after examining these results.

No optimizer convergence, Hessian anisotropy or low fitness alone is interpreted
as true localization error, severe native corridor degeneracy, or nonlocal ambiguity.
