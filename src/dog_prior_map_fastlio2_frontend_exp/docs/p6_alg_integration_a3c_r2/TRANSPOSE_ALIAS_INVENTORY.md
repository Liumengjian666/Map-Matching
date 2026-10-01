# P6 package transpose-alias inventory

Scope: package `src/` and `include/`; original line numbers below refer to
START_SHA `d97c3cd8e2faeb6a047e93a789692b898429fcc9`. All transpose occurrences
were inspected, including multiline, pointer, field, block and return forms.
The Python assignment scanner is an aid, **not** complete C++ alias analysis.

28 self-transpose symmetry assignments reviewed: 20 confirmed unsafe changed,
8 already safe retained. Two critical changes now use an independent temporary
returned by `evaluateSymmetricInformation(A)`, whose entire expression is
evaluated before assignment. The other 18 use whole-expression `.eval()`.
No `noalias()` was introduced. One unsafe test oracle outside src/include was
also repaired without changing its tolerance; it is not counted among the 20.

For compactness, `S(A)` below denotes the exact original RHS
`0.5 * (A + A.transpose())`; pointer notation denotes
`0.5 * (*A + A->transpose())`. Every changed row was originally
`destination = S(destination)` without evaluation. Production YES means
reachable package estimation code, not necessarily exercised by NO-VISION.

| File / original line | Function / destination / original expression | Production path | Alias risk | Changed | Reason |
|---|---|---|---|---|---|
| src/fixed_lag_window.cpp:507 | linearizeSelected: `*hessian = S(*hessian)` | YES | YES | YES | Independent evaluated temporary |
| src/window_marginalization.cpp:398 | marginalizeOldest: `new_information = S(new_information)` | YES | YES | YES | Independent evaluated temporary; tx90 M7 |
| src/window_marginalization.cpp:453 | marginalizeOldest jitter-sensitivity reference: `alternate_information = S(alternate_information)` | YES, existing jitter diagnostic branch | YES | YES | `.eval()`; production jitter value/decision unchanged |
| src/window_visual_factor.cpp:104 | linearizeSelectedVisualFactor: `*covariance = S(*covariance)` | YES, visual factor enabled | YES | YES | `.eval()` |
| src/window_imu_factor.cpp:237 | integrateBoundedImuSequence: `output->covariance = S(output->covariance)` | YES | YES | YES | `.eval()`; propagation formula unchanged |
| src/measurement_noise_model.cpp:61 | propagatePhysicalLidarCovariance: `result.residual_covariance = S(result.residual_covariance)` | YES | YES | YES | `.eval()`; A_exact unchanged |
| src/dual_reliability.cpp:122 | cappedResponseCovariance: `*output = S(*output)` | YES | YES | YES | `.eval()`; existing eigen cap unchanged |
| src/dual_reliability.cpp:344 | assessLocalRisk: `equalized = S(equalized)` | YES | YES | YES | `.eval()`; equalization unchanged |
| src/dual_reliability.cpp:659 | makePoseMeasurementNoise: `result.covariance = S(result.covariance)` | YES | YES | YES | `.eval()`; adaptive R formula unchanged |
| src/dual_reliability.cpp:877 | assessVisualWeakSubspaceInformation: `result.visual_information = S(result.visual_information)` | YES, visual enabled | YES | YES | `.eval()`; visual information definition unchanged |
| src/reliability_metrics.cpp:70 | pseudoInversePSD3: `*inverse = S(*inverse)` | YES | YES | YES | `.eval()`; supported-subspace threshold unchanged |
| src/reliability_metrics.cpp:95 | schurDecouple: `*rotation = S(*rotation)` | YES | YES | YES | `.eval()`; Schur definition unchanged |
| src/reliability_metrics.cpp:96 | schurDecouple: `*translation = S(*translation)` | YES | YES | YES | `.eval()` |
| src/reliability_metrics.cpp:119 | schurDecouple reconstructed rotation: `*rotation = S(*rotation)` | YES | YES | YES | `.eval()`; existing reconstruction unchanged |
| src/reliability_metrics.cpp:120 | schurDecouple reconstructed translation: `*translation = S(*translation)` | YES | YES | YES | `.eval()` |
| src/reliability_metrics.cpp:201 | transformPclScoreHessianToNormalizedMapTangent: `result.hessian_physical = S(result.hessian_physical)` | NO in V3; historical curvature interface | YES | YES | `.eval()`; Euler/physical transform unchanged |
| src/reliability_metrics.cpp:208 | same: `result.normalized_negative_score_curvature = S(result.normalized_negative_score_curvature)` | NO in V3; historical interface | YES | YES | `.eval()`; sign and length scale unchanged |
| src/reliability_metrics.cpp:382 | analyzeGeometricObservability: `information = S(information)` | YES | YES | YES | `.eval()`; weight normalization unchanged |
| src/reliability_metrics.cpp:386 | same: `normalized_information = S(normalized_information)` | YES | YES | YES | `.eval()`; scale unchanged |
| src/reliability_metrics.cpp:417 | same after existing PSD reconstruction: `normalized_information = S(normalized_information)` | YES | YES | YES | `.eval()` only; existing cwiseMax untouched |
| src/fastlio2_frontend_ikfom.cpp:737 | projected update: `posterior = (S(posterior)).eval()` | YES, legacy/pre-handoff boundary | NO | NO | Already evaluated |
| src/fastlio2_frontend_ikfom.cpp:909 | projected update: `posterior = (S(posterior)).eval()` | YES, legacy/pre-handoff boundary | NO | NO | Already evaluated |
| src/fastlio2_frontend_ikfom.cpp:1123 | bridge: `output->information15 = 0.5*(... + ...transpose()).eval()` | YES, initialization | NO | NO | Evaluated parenthesized sum before assignment |
| src/fixed_lag_window.cpp:1177 | sparse marginal output: `output->map_pose_covariance6 = 0.5*(... + ...transpose()).eval()` | YES | NO | NO | Already evaluated sum |
| src/fixed_lag_window.cpp:1236 | dense marginal test oracle: `hessian = 0.5*(hessian + hessian.transpose()).eval()` | NO, test-only | NO | NO | Already evaluated sum |
| src/window_lidar_factor.cpp:146 | freezeLidarProjection: `selected_covariance = (S(selected_covariance)).eval()` | YES | NO | NO | Already safe; frozen projection unchanged |
| src/window_linear_system.cpp:285 | blockLinearizedSystem: `block.second = (S(block.second)).eval()` | YES | NO | NO | Already safe; retained unchanged |
| include/.../fixed_lag_production.hpp:89 | selected NIS: `innovation = 0.5*(innovation + innovation.transpose()).eval()` | YES | NO | NO | Already evaluated sum |

Additional manually inspected non-self/other forms:

- `window_linear_system.cpp::dense()` return expression is whole-expression
  evaluated; `sparse()` diagonal blocks initialize independent `Matrix15d`.
- `dual_reliability.cpp` projected visual-information block assignment uses
  independent `projected`; no destination transpose alias.
- `measurement_noise_model.cpp:85` takes an external input and writes an
  output field. Current caller uses distinct objects; no confirmed active
  self-alias. API would theoretically allow shared storage, which is not
  certified by this limited audit; no unrelated API redesign was made.
- Independent initializations (`symmetric`, `hmm_solve`, covariance/curvature
  blocks), residual transpose products, and congruence products were inspected.
  Matrix-product evaluation and independent destination storage do not have
  the confirmed in-place symmetry-average bug. Existing safe `.eval()` products
  were retained, including IKFoM posterior reset.
- A1-R1 `schurOldest()` test oracle now evaluates its symmetric result before
  assignment. The R1 capsule's legacy unsafe expression remains explicitly
  forensic-only, never an estimation path.

Ideal-math equivalence for every changed row: for an immutable pre-assignment
matrix A, both intend `(A_ij + A_ji)/2`, with the same diagonal and dimensions.
Only eager evaluation prevents later coefficients from reading already-written
values. No reliability definition, eigen direction, rank threshold, length
scale, adaptive covariance, noise, damping, Schur blocks, pivot or validator
parameter was edited. No new eigen clamp/regularization was added; pre-existing
reliability reconstruction/caps remain unchanged. The prior has no clamp.

Scanner after repair: 46 symmetry-related assignments (including independent
initializations and outer products), 26 remaining textual self-assignments all
evaluated, plus the 2 critical assignments moved to the independent helper.

Fresh-context read-only review found no remaining confirmed unsafe production
symmetry-average expression and prompted the independent A1 oracle repair and
the pose-only rank-5 nullspace test strengthening. External review is left to
the user's final handoff; no additional approval pause is required.
