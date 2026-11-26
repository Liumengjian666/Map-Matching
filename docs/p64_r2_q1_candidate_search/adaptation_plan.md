# Candidate search adaptation after Q1

Only A, the original Park planar grid with documented assumptions, is implemented for the qualification. B–E below are planning. A future adaptive strategy must preserve the full R2 ≥90% development recall target, lose ≤2 percentage points against the best equally budgeted direct grid, and expose precision, latency, mode diversity, and uncertainty suitable for geometric comparison.

| Strategy | Cost and coverage risk | First bounded validation |
|---|---|---|
| A · Park planar grid | 21×21×11=4,851 poses at native resolution and fixed height, roll and pitch. Large tilt or drift may exclude the true basin | Separate continuous fixed-plane feasibility, lattice feasibility, local peak existence and ranking. Preserve fixed sample results |
| B · Direct 6-DOF | If 3 translation axes have 21 samples and 3 angles have 11, 12,326,391 poses, 2,541× A. Equal-budget 4⁶=4,096 sparse poses may miss narrow basins | Use a fixed equal-budget sparse grid plus exact center, at most one bounded higher-resolution diagnostic with approval |
| C · IMU covariance ellipsoid | Complexity remains O(GNh), G is tested poses, N source count, h neighbor limit. Overconfident estimates after loss can remove true hypotheses | Use covariance in a documented SE(3) perturbation frame, hard search bounds and a reserved deterministic anchor grid. Measure missed labels and boundary mass |
| D · Weak-direction subspace | Sampling grid exponent reduces with subspace dimension. The Hessian describes local curvature, while nonlocal rival modes can point outside its weak span | Before limiting search, compare candidate separation directions with scaled weak eigenvectors across frozen frames. Preserve several orthogonal anchors |
| E · Prior and coarse-to-fine modes | Coarse Gc plus summed fine Gj; coarse resolution may erase a narrow mode. A motion prior may favor the wrong basin after drift | Keep several diverse nonlinear peaks and full-score comparisons on identical maps. Count native fixed evaluations and full fits separately |

Do not enlarge the qualification grid after inspecting rankings. Cases out of range, including non-searched attitude or height, go to A/B/C coverage qualification. Cases with valid discordant candidates are reserved for fixed-candidate discrimination research after acceptance. R3 requires per-candidate uncertainty and a conservative projection rule, with no uncontrolled alignment between the proposed local views.
