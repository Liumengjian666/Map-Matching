# P9-R1A: oracle terminal and strong-profile closure

This offline experiment preserves the exact PCL 1.10 optimization energy and the R1/P9 product chart. It does not define a posterior NLL, basin probability, measurement covariance, or EKF policy.

With radius-search target support G_i(T), the native score is S(T)=sum_i sum_{c in G_i(T)}[-d1 exp(-d2 e_ic^T C_c^-1 e_ic/2)], subject to PCL's finite/[0,1] exponential-guard check; E(T)=-S(T), mean energy=E/N over the unchanged prepared source count. For this frozen .8m/.55 outlier contract, d1=-1.646558519810 and d2=.528248766985. Dynamic support can change this energy non-smoothly. Covariance determinants/normalization do not turn this PCL score into a calibrated posterior NLL.

## Frozen inputs and oracle membership

The 22 P9-R1 major cluster IDs are frozen before this experiment. Primary clustering is reproduced using `p5_i1_cluster_modes.py` at `9945c4f5c3d7759104de108a594bcaf2553fd78c`, SHA256 `13dd0a7fe9fe87506f1f3728a81a6c81faa5afbbd0789cc4e8fc4cc89fa38805`. It uses deterministic agglomerative complete-link with translation <=0.2 m AND rotation <=2 degrees, with minimum seed-index tie breaking. `summarize_cluster()` selects `max(raw_ndt_score_sum)` and copies that candidate's `final_pose_matrix16`; the representative is therefore a real best-score terminal, not an averaged pose. Exact membership and representative identity are checked against `candidates.csv` and `mode_clusters.csv`.

Canonical terminal means the real member with highest saved raw objective score. Each receives one standard frozen-parameter NDT refinement. Successful refinement remaining within 0.2 m AND 2 degrees of that canonical terminal defines a closed target; departure is separately marked `ORACLE_CLUSTER_NOT_STATIONARY`, and failed/limited refinements cannot be closed targets. Complete-link admission against every original member is also reported, because threshold proximity to the representative alone does not reconstruct agglomerative membership. A successful radius-preserving refine does not certify dynamic-energy stationarity.

## Product chart

`eta=[delta_t_map/0.8m, delta_theta_map]`, `t=t0+0.8 eta_t`, `R=Exp(eta_theta)R0`. `mapChartDisplacement()` is used to compute `delta_b`, then `u_b=W^T delta_b`, `v_b=S^T delta_b`. This is additive map translation and map-spatial left rotation about the LiDAR origin. SE(3) log is used only for pose-distance diagnostics, not weak coordinates. W and S remain fixed at the archived nominal terminal T0.

The original 22 IDs remain the oracle denominator. GROUP A contains closed targets with weak-span projection >=0.8 and a projected weak displacement inside the original coordinate bounds and the combined <=2 m / <=15 degree region. Targets that depart their archived cluster after canonical refinement are explicitly reported rather than silently removed from the oracle contract.

## Solver parameters frozen before results

All solvers hold u fixed and use the exact dynamic energy E/N for acceptance. They share the source, target Gaussian grid, pose chart, and fixed W/S. No per-frame parameter tuning is permitted.

- ONE_STEP_NEWTON: exact P9-R1 `profileAt()`, one iteration, step norm <=0.10, strong coordinate norm <=0.25, line search [1,0.5]. Its max-iteration flag means its one-step budget was used, not that a converged inner minimization failed.
- ITERATIVE_PROJECTED_NEWTON: maximum 20 iterations; initial trust radius 0.10, minimum 1e-5, maximum 0.50; eigenvalue damping floor 1e-4 times local Hessian scale; line-search alpha [1,1/2,...,1/128]. Only actual dynamic-energy reduction greater than 1e-10 is accepted. Predicted/actual reduction adjusts the trust radius (halve below ratio .25, double above .75 when the boundary step was used). Projected branch-gradient tolerance 1e-5, step tolerance 1e-6, relative energy-change tolerance 1e-9. No strong-coordinate norm cap. A small branch gradient is not claimed to certify a smooth dynamic-support stationary point.
- DERIVATIVE_FREE_PATTERN: budget <=100 dynamic value-only evaluations including the initial value; initial coordinate step 0.10, minimum 0.001, successful-sweep growth 1.2 up to 0.25, unsuccessful-sweep shrink 0.5; deterministic +/- coordinate polling followed by one pattern move. Accept only actual energy reduction >1e-10; no strong-coordinate norm cap. Budget termination is reported separately from step convergence.

Each GROUP A target uses five combinations: one-step/zero-v, Newton/zero-v, derivative-free/zero-v, Newton/oracle-v, derivative-free/oracle-v. Oracle-v is diagnostic only. Each endpoint receives at most one full NDT refinement (resolution .8 m, step .08, epsilon 1e-5, max 80, outlier ratio .55). Recovery requires that endpoint to be within .2 m AND 2 degrees of the closed canonical target; before-refinement distance is preserved.

The new Newton pullback differentiates the fixed global exponential chart into the current native Euler branch, including its second derivatives. It removes the old .45-radian distance-to-base-Euler check (uses a 4-radian branch unwrap guard), retains the reconstruction/condition-number checks, and does not rotate W. Endpoint projected gradients use the exponential's left Jacobian to convert local-at-endpoint derivatives back to the fixed global product chart. Trace gradients are explicitly pre-step values. Accepted dynamic energy is strictly monotone, but a budget-terminated solve is not `min_v` certified. Directional FD sidecars compare frozen and dynamic support independently, with steps .001/.0005; these additional diagnostic evaluations are not hidden inside the 100-call pattern budget.

Every accepted poll, pattern move or Newton step is recorded, not just sweep endpoints; accepted_move indices run from0(initial) to the recorded accepted-step count. ONE_STEP_NEWTON's initial and single accepted endpoint are reconstructed by the wrapper from existing diagnostics, preserving the exact old solver and its >1e-12 acceptance rule. New solvers use >1e-10. Maximum accepted support change is the maximum change from the preceding actual accepted state, never an unmeasured default zero. Regression validation verifies this completeness and its endpoint/energy/support consistency for all35 runs.

## Boundary and conditional grid diagnostics

The original grid is reconstructed exactly: 17 coordinates for k=1 and 9x9 for k=2, with combined weak physical-limit clipping. Distance to the coordinate box and to the clipped physical boundary is reported in original grid-step units. A projected basin whose nearby grid node lacks any required neighbor would be excluded by the original complete-neighborhood rule; that is a discrete-grid diagnostic, not an assertion about the continuous objective.

Adaptive local-grid diagnostics are authorized only if at least 70% of GROUP A targets recover with a single zero-v strong solver. The gate is evaluated for each solver separately, not by pooling their successes. If triggered, a frozen choice of the best zero-v solver evaluates oracle-conditioned local 3x3 neighborhoods at original spacing, 1/2, 1/4, 1/8, with no search-range expansion. Results estimate sampling/capture widths and remain offline oracle-conditioned diagnostics.
