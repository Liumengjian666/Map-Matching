# P62-R15 coupled subspace refinement: frozen mathematical contract

Status: frozen before R15 frame processing. One accidental inspection of three rows from the *prior R14B summary* `REFERENCE_ERROR.csv` occurred before R15 candidate freeze; it exposed only previously archived reference-relative error summaries, not raw registered poses or any R15 output. This crossover is disclosed and those values were not used to choose frames, steps, directions, or outcomes. No underlying reference pose was read for R15 before candidate-output freeze.

Numerical-method amendment: the initial synthetic implementation mapped PCL's Euler-chart analytic gradient into right/body coordinates, but pre-run synthetic directional checks showed roughly 5–7% mismatch against direct score differences. Before any R15 frame inputs/results were processed, that route was rejected in favor of central differences of the exact PCL scalar score directly in right Lie coordinates. A subsequent synthetic precision sweep found the `h_H=1e-3` gradient-Hessian stencil exceeded the frozen 5% antisymmetry gate narrowly (5.02%); `h_H` was therefore changed to `8e-3`, with independent scalar second-difference steps set to `4e-3, 8e-3, 16e-3`. The sweep was synthetic-only; no real-frame poses/scores/reference values were inspected to choose these steps. These are the final pre-run steps and gates.

## Claim and falsification gate

Claim under test: for a frozen nominal pose `T0`, causal prediction `TP`, training Source, and the exact PCL 1.10 target Gaussian grid, the normalized local NDT score model can define a stable strong/weak decomposition and a bounded strong response to the prediction projected into at most two weak directions. The implementation does not replace or re-run NDT and does not inherit NDT convergence status.

Falsification: do not proceed to real-frame refinement if the PCL score-equivalent formula cannot be reproduced, right/body directional derivatives fail their predeclared finite-difference tolerance, the R6 geometric-information conversion fails its independent check, or the local strong solve is nonfinite/non-positive/ill-conditioned. A failed frame remains recorded as not evaluable; thresholds are not tuned on frame results.

## Coordinates and exact PCL objective

`T_A_B` maps coordinates from frame B to A. `T0` and `TP` are `T_M_L`. Perturbations are right/body perturbations:

`T(z) = T0 Exp(S z)`,

where `z` is dimensionless, ordered translation then rotation, and
`S = diag(0.30, 0.30, 0.30, 3*pi/180, 3*pi/180, 3*pi/180)` maps it to an SE(3) tangent `(rho [m], phi [rad])`. `Exp`/`Log` are the full SE(3) maps from the frozen R11 implementation; in particular, `rho` is mapped through the SO(3) left Jacobian inside `Exp`.

The configured parameters are resolution `0.8 m` and outlier ratio `0.55`, but all computations use the actual `float` values stored by the configured PCL 1.10 object (`resolution_`, `gauss_d1_`, and `gauss_d2_`), never a separately re-created double approximation. For explanation, PCL computes those constants from its stored values as follows:

```
c1 = 10 (1-o)
c2 = o / r^3
d3 = -log(c2)
d1 = -log(c1+c2) - d3
d2 = -2 log(( -log(c1 exp(-1/2)+c2) - d3 ) / d1)
```

For each source point `x_i` transformed with the same float transform and operation sequence used by PCL to `y_i(T)`, PCL radius-searches Gaussian leaves `N_i(T)` within its stored `resolution_`. With residual `e_ik=y_i-mu_k` and inverse covariance `Q_k`, its score is

`Score(T) = sum_i sum_{k in N_i(T)} -d1 exp(-d2 e_ik^T Q_k e_ik / 2)`.

R15 minimizes the exact negative normalized score
`F_NDT(z) = -Score(T(z))/N_train`, the exact negative normalized score of the configured PCL 1.10 object; it is not nearest-neighbor fitness. Radius-search associations are pose-dependent and hard-truncated, so the objective is piecewise smooth. At each nominal pose, record an ordered association signature of Gaussian-leaf identities returned for every training point. Every predeclared finite-difference sample for the gradient and Hessian must preserve this signature. If a point's leaf set changes anywhere in a stencil, mark that frame `LOCAL_ASSOCIATION_SET_CHANGED`; report no local derivatives or refinement response for it. Dynamic exact scores for proposed poses can still be recorded, but cannot be interpreted as derivatives on the nominal local branch.

The local derivatives are computed directly in the declared right/body Lie-algebra coordinates, independent of PCL's Euler gradient chart. The gradient is a deterministic central difference of the exact negative normalized PCL score, `g_i(z)=[F_NDT(z+h_g e_i)-F_NDT(z-h_g e_i)]/(2h_g)`, with fixed dimensionless `h_g=5e-4`. The Hessian approximation is the central difference of this gradient with fixed outer step `h_H=8e-3`, `H_raw[:,i]=[g(h_H e_i)-g(-h_H e_i)]/(2h_H)`. PCL's protected `computeDerivatives()` scalar score is used as an independent score-parity cross-check; its Euler-chart analytic gradient is not mixed into the right-tangent model. Before symmetrization, report `||H_raw-H_raw^T||_F/max(1,||H_raw||_F)`. Check `g^T v` against the exact scalar central directional difference at the independent fixed step `h_d=1e-3` for six coordinate and four predeclared mixed directions `(e_i+e_j)/sqrt(2)` for `(i,j)=(0,3),(1,4),(2,5),(0,1)`. Independently compare `v^T H_raw v` with the central second difference of exact scalar `F_NDT` along those same ten directions at steps `4e-3,8e-3,16e-3`. Every objective sample in these stencils must preserve the same PCL leaf-association signature. These are explicitly numerical local derivatives, not PCL's optimizer Hessian and not the R6 `geometry()` matrix.

The reported local quadratic is
`J(q)=F0 + gq^T q + 0.5 q^T Hq q`, with `q=U^T z`. Its predicted changes are kept separate from exact PCL-score changes evaluated at the resulting poses.

## Geometric information and strong/weak decomposition

The PSD geometric-information matrix is the Gaussian-weighted Gauss-Newton matrix, not an exact NDT Hessian:

`G_z = (1/N_train) sum_(i,k) w_ik J_(y,z)^T Q_k J_(y,z)`,
`w_ik = (-d1) d2 exp(-d2 e_ik^T Q_k e_ik/2) >= 0`,
`J_(y,z) = R0 [ I, -[x_i]_x ] S` at `z=0`.

All leaves, transformed query coordinates, and residual weights use the real PCL Gaussian grid and the same float `PointXYZ` transform operation as the PCL score at the nominal pose. The right/body Jacobian remains the differential of the declared continuous SE(3) chart. The implementation independently computes this matrix from the R6 `geometry()` output after the exact coordinate conversion below, and compares it to the direct right/body calculation. R6 `geometry()` uses double pose arithmetic before casting query points to float; the pre-run cross-check tolerance is therefore `1e-7` (the generic synthetic case measured `3.24e-8`), while the algebraic frame/scale conversion remains exact for identical point/weight inputs. R6 `geometry()` is explicitly not labeled the optimizer Hessian. The PCL radius and Gaussian constants use the actual stored float values; the nominal association signature accompanies this matrix.

R6 geometry uses left/map perturbation coordinates and `J_legacy=[0.8 I, -[R0 x]_x]`. Let `D=diag(0.8 I, I)` and `A=diag(R0,R0) S`. The exact scale/frame conversion for that stated Jacobian is
`G_z = A^T D^{-T} G_legacy D^{-1} A`.
This identity is checked numerically against the direct `G_z` construction. Eigenvalues and eigenvectors of the normalized `G_z` are recorded; eigenvectors are orthonormal. The frozen R9 diagnostic ratio `lambda_i/lambda_max <= 0.01` is preserved. At most the two smallest qualifying directions are used as prediction-guided weak coordinates. The orthogonal complement is named the `solved complement`, not the strong subspace; additional qualifying weak directions there are reported as `unmodeled weak directions`, not as strongly constrained.

## Prediction projection, fixed regularization, and trust region

`xi_pred = Log(T0^-1 TP)` and `z_pred=S^-1 xi_pred`. For the selected weak eigenvectors `Uw`, `dw_raw(alpha)=alpha Uw^T z_pred`, with fixed `alpha in {0.5,1.0}`. Each `dw_raw` is uniformly clipped, before any objective/validation comparison, to the fixed unit Euclidean trust ball. Thus `||Uw dw||<=1`, implying physical translation norm `<=0.30 m` and physical rotation norm `<=3 degrees`. A correction that is nonfinite or has normalized norm `<=1e-10` is recorded as unavailable.

The local model blocks are formed from the same symmetrized Hessian model `H_model=(H_raw+H_raw^T)/2` used to compute reported quadratic changes, all in the same orthonormal eigenbasis:
`gq=U^T gz`, `Hq=U^T Hz U`.
For the same fixed `dw`, `WEAK_ONLY` sets `ds=0`. `COUPLED` solves

`(Hss + 0.01 I) ds_raw = -(gs + Hsw dw)`

with Eigen LDLT (no explicit inverse); `lambda=0.01` is fixed in the dimensionless, negative-normalized-score objective before real-input results. Before regularization, raw `Hss` must be finite, symmetric within the measured tolerance, positive definite, and have condition number `<=1e8`; zero or negative curvature and raw ill-conditioning reject the pair. The regularized `Hss+lambda I` must independently be finite, positive definite, and have condition number `<=1e8`. This prevents regularization from masking a singular raw block. `ds_raw` is scaled only by the deterministic maximum factor in `[0,1]` that keeps `||(ds,dw)||_2<=1`; `dw` remains exactly identical to the paired WEAK_ONLY candidate. If `||dw||>1` or strong solve fails, that paired refinement is unavailable.

The strong solve uses only eigenvectors with `lambda_i/lambda_max > 0.01`. If more than two directions qualify as weak, only the two lowest are prediction-guided and the remaining weak coordinates are explicitly frozen at zero; they are not silently treated as strongly constrained. If the strong subspace is empty, the WEAK_ONLY ablation remains available and COUPLED is marked unavailable. Before symmetrizing, the projected raw `Hss` antisymmetry is recorded and must be `<=0.05`; then `Hss`, `Hsw`, and `Hww` are all projected from the same `H_model`, avoiding inconsistent cross blocks. The raw and regularized strong block each independently pass finite, positive-definite, and condition-number gates.

The coupled response is a conditional local-model response, not a new NDT optimization. It is rejected as a formal R12 observation because it has not completed an NDT align; `converged` is never copied from `T0`. R12/V2 held-out losses, physical differences, and common-weight `D>0.02` are reported as diagnostics against the raw nominal anchor and other candidates; no refinement is accepted and no EKF feedback occurs in R15.

## Fixed comparisons and output distinctions

For every evaluable frame and alpha, compare the same `T0`, `TP`, map, Training Source, Validation Source, weak basis, and `dw`:

- `NOMINAL_NDT`: frozen `T0`;
- `WEAK_ONLY`: `T0 Exp(S Uw dw)`;
- `COUPLED`: `T0 Exp(S U (ds+dw))`.

Record separately: local quadratic predicted delta; exact training negative-score delta; R9/R12 Coverage V2 held-out block losses and `D`; translation/rotation pose change; formal R12 eligibility (false for post-NDT candidates); reference-relative Newer College error only after all candidate and selector outputs are frozen. No NDT align, baseline replay, trajectory replay, or EKF is performed.

Predeclared numeric checks: direct score parity against the same PCL object's `computeDerivatives()` score absolute error `<=1e-9`; geometric matrix conversion relative Frobenius error `<=1e-7` to account for the fixed R6-double versus PCL-float point transform difference; basis orthogonality error `<=1e-10`; directional derivative relative error `<=2e-2` when the central directional magnitude exceeds `1e-8`; raw Hessian antisymmetry relative error `<=5e-2`; every independent second-difference check must have relative error `<=1e-1` at the central step and no more than a factor-of-two spread between adjacent step sizes; raw and regularized strong blocks pass their separate positive-definite/condition gates and the unregularized raw `Hss` antisymmetry is `<=5e-2`; trust-ball norm `<=1+1e-10`. The finite-difference steps and thresholds are frozen before reading R15 frame poses/scores. A failed check is preserved and prevents the affected frame from refinement; it cannot trigger threshold retuning.
