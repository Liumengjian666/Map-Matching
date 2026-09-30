# PAPER-P6-ALG-INTEGRATION-A3C-R1

## Outcome

`A3C_R1_MARGINALIZATION_ROOT_CAUSE_ISOLATED_PASS`.

The single authorized Corridor01 P3-100 forensic replay reproduced the same
logical failure at transaction 90 and timestamp `1517157228164951397`. The
optimizer first returned `ACCEPTED_UPDATE` (8 iterations; surrogate cost
`17.972697968746459` → `13.976734288274939`). The subsequent failure was in
fixed-lag marginalization, not optimizer candidate acceptance.

The primary cause is **A3C-R1-G — OTHER_IDENTIFIED_CAUSE**: the in-place Eigen
expression used to symmetrize the raw Schur matrix left a maximum elementwise
symmetry defect of `1.1175870895385742e-8`. The unchanged production validator
rejects defects above `1e-8`. Reconstructing the same Schur input from the saved
capsule and applying out-of-place symmetrization produced zero symmetry defect
and passed the same PSD validator. No negative Schur eigenvalue caused this
failure.

No production algorithm, estimator parameter, NDT setting, jitter, PSD
tolerance, or frozen A3B-R2 behavior was changed. All added work is diagnostic,
test, and analysis plumbing. A final diagnostic-only refinement makes the
stage classifier use scale-relative checks for intermediate matrices and the
exact production validator for M7. The already completed replay was not
repeated; its raw trace is preserved as captured, and the corrected M7
attribution is backed by the capsule-based C++ emulation described below.

## Repeated marginalization result

- First successful elimination in the captured run: tx71,
  event stamp `1517157226248733355`, removing state stamp
  `1517157224188979000`.
- Before tx90: 19 successful enforcement episodes and 38 successful
  `marginalizeOldest()` attempts. Each of those 19 events removed two oldest
  states.
- tx90: enforcement index 78, attempt 1 of 1; zero earlier successful
  removals in that same enforcement. Trigger was duration limit only
  (`2.017077923 s`, 41 nodes; duration=true, node limit=false).
- Across the 38 successful attempts, stored/new prior `lambda_min` and
  relative-negative ratio remained exactly zero in the recorded eigensolver
  output. There is no evidence of accumulated tolerated negative prior modes.
- Historical `H_mm` remained rank 15 by the primary machine-epsilon rank
  rule, with positive LDLT pivots and no jitter. Its conditioning varied
  substantially; tx90 itself was well-conditioned relative to earlier
  attempts.

## tx90 matrix evidence

- Incoming prior and charted prior: finite, `lambda_min=0`, no relative
  negative mode.
- Touching-factor contribution: one IMU, one rank-5 LiDAR, zero visual
  factors. Its small negative eigenvalue was `-2.724743895e-7` against a
  spectral scale of `2.0000000125e9` (`1.36e-16` relative); the IMU and LiDAR
  contributions show the same scale-relative round-off pattern.
- Consumed system: finite, numerical rank 30/615, `lambda_min=0`.
- `H_mm`: full rank 15/15 at the primary threshold and all three sensitivity
  thresholds. `lambda_min=7.589490168e6`, `lambda_max=1.000015137e9`, condition
  proxy 131.76.
- LDLT: 15 positive, 0 negative, 0 near-zero pivots; min/max absolute `D`
  `7.589491108e6` / `1.000015116e9`; pivot ratio `7.5894e-3`; jitter was 0.
- Production solve backward errors: `1.36e-16` for `H_mr`, `3.97e-17` for
  `g_m`. The independent NumPy solve gives the same scale.
- New gradient: finite, norm `40.7850465`, max absolute component `37.39894098`.
- The production in-place Schur symmetrization left max asymmetry
  `1.1175870895e-8`, just above its fixed `1e-8` gate. The raw matrix
  asymmetry Frobenius norm was `6.3275346904e-8`.
- The exact same reconstructed raw Schur matrix, symmetrized out-of-place,
  had max asymmetry 0 and passed the unchanged validator. Its minimum
  eigenvalue under independent symmetric spectral analysis was 0, not
  negative. Production itself short-circuited on the symmetry gate before
  evaluating eigenvalues.

The raw trace CSV still contains the initial diagnostic heuristic label
`M2_TOUCHING_FACTORS`: that first implementation incorrectly applied the
absolute `1e-8` symmetry gate to a large intermediate Hessian. Its normalized
symmetry defect was only `4.47e-17`, and its relative negative ratio was
`1.36e-16`. The capsule-based replay of the actual M7 production expression
resolves the true stage as:

`FIRST_BAD_STAGE = M7_SYMMETRIZED_SCHUR`.

The captured CSV is preserved verbatim. The corrected adjudication is also
recorded in the keyed sidecar `FIRST_BAD_STAGE_ADJUDICATION.csv`; it neither
overwrites the raw trace nor implies that the corrected classifier was rerun
on the real replay.

## Other requested hypotheses

- H1, accumulated tolerated prior indefiniteness: **not supported**.
- H2, tx90 absolute LDLT gate misclassification: **not supported at tx90**;
  its pivots were comfortably above the absolute gate and all were positive.
  Historical relative pivot ratios did become small (minimum
  `6.2486e-10`), but no prior elimination used jitter or reported a rank loss.
- H3, nullspace/range inconsistency amplified by solve/jitter: **not
  supported**. `H_mm` is full rank under the primary and sensitivity
  thresholds; both range residuals were zero; no jitter was used.
- H4, negative Schur eigenvalue rejected only by scale: **not the failure
  mechanism**. The Schur spectrum was nonnegative; the rejection was the
  elementwise symmetry gate.
- M8, nonfinite new gradient: **not supported**.

Independent generalized-Schur comparison: the production-correction Schur
matrix reconstructed from the saved solve versus Moore-Penrose oracle had
relative Frobenius matrix error `1.2413e-10`
(absolute `4.1494e-6`) and relative gradient error `3.4027e-19`
(absolute norm `1.3878e-17`). Direct solve-based Schur relative matrix error
was `1.3812e-11`. The oracle is forensic only; production was not changed to
use a pseudoinverse.

## Transactionality and limits

`MULTI_REMOVAL_TRANSACTIONALITY = NOT_EXERCISED` for the failing enforcement:
tx90 failed on attempt 1, so there was no earlier removal in that same
enforcement to partially commit. The before/after state stamps, prior hash,
and factor counts are identical. This does not claim batch rollback for a
later-attempt failure.

The one P3-100 replay used no GT and no accuracy metric. It ended at the
expected marginalization failure; P3-200, complete Corridor01, and Floor01
were not run. The `26.35 s` runtime and `106648 KiB` maximum RSS include
forensic diagnostics and are not performance benchmarks.

`READY_FOR_FORMAL_EXPERIMENT = NO`.

See `TX90_STAGE_ANALYSIS.md`, `MARGINALIZATION_CHRONOLOGY.md`,
`TX90_GENERALIZED_SCHUR.md`, `REAL_REPLAY_IDENTITY.md`, and
`BUILD_AND_CTEST_RESULTS.txt` for details and reproducibility data.
