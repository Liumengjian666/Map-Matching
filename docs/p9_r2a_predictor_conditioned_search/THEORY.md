# R2A: Predictor-conditioned weak-subspace search

## Frozen question and scope

Test whether keeping the real IMU predictor's complementary coordinates enables low-budget competing-basin discovery. DUAL-U remains within-basin directional observability plus nonlocal ambiguity. This experiment evaluates one candidate discovery mechanism on the frozen 32-frame Floor01 cohort: nine major-competitor frames and 23 no-major controls. Labels, canonical basin poses and GT never enter proposal construction or selection. R2 terminal-dispersion statistics are not reused as evidence.

## Predictor and chart

The frozen `cohort_frozen.csv.initial_pose_xyz_q_xyzw` must equal the saved `clusters/manifest_current_baseline.csv.predicted_pose_xyz_q_xyzw`. The original runner forms `fromMatrix(rightPerturb(frame.initial_pose, seed))`; seed122 has six zero perturbations. Compare its archived start against the explicit predictor in that original Matrix4f carrier with predeclared translation tolerance 1e-5 m and rotation tolerance 1e-4 degree. Preserve the archived seed122 pose as T_pred; the first proposal uses that exact carrier.

All displacements use the existing P9 map product chart at nominal NDT terminal T0:

`eta(T)=[(t-t0)/0.8m; Log_spatial(R R0^T)]`.

Read the ascending-curvature eigenvectors from the frozen U_obs output; W2=(q1,q2), STRONG2=(q5,q6). No basis is recomputed. For each archived start j=0..262, compute eta_j and eta_pred from actual poses, then

`eta_cond,j = eta_pred + Q Q^T (eta_j-eta_pred)`.

Thus `(I-Q Q^T) eta_cond,j = (I-Q Q^T) eta_pred`. Projection is never renormalized. Every archived pose and every conditioned proposal must round-trip through the existing chart within the same geometry tolerances. Complement preservation is checked algebraically to 1e-12 before the float pose conversion. The old H2 formula `Q Q^T eta_j` is a historical reference, not the conditioned formula.

## Pool and budgets frozen before results

Use only the 263 archived base starts for all 32 frames; exclude targeted extensions. Three RANDOM2 bases per frame reuse the already frozen R2 PCG64 bases (seed `20261007 + 1000*cohort_index + rep`, rep=0,1,2). They are reproduced from the RNG and archived before NDT.

Select in 2D coordinates `Q^T(eta_j-eta_pred)`: first seed122, then maximize minimum squared Euclidean distance to the selected set. Ties within `1e-12*max(1,largest_distance_squared)` choose the lowest seed index. Retain duplicates and charge each call. Freeze all 16 selected indices before alignment; report B=4,8,12,16 and the complete nested prefix 1..16 for normalized trapezoidal AUC against log2(B).

Each selected proposal is aligned once. Maximum new calls are 32*16*(WEAK+STRONG+3 RANDOM)=2560. NDT is unchanged PCL1.10, resolution0.8, step0.08, epsilon1e-5, max80, outlier0.55, frozen source/map/preprocessing. Record every finite result and convergence/iteration-limit separately. FULL6D is reused from H2 with no new alignments. Scores are the exact frozen PCL sum objective in its archived normalized quaternion/Matrix4f carrier; lower is better, with no probabilistic interpretation.

## Recovery and comparisons

Use H2's frozen 22 canonical archive terminals and admission: translation<=0.2m AND rotation<=2deg, only PCL converged returns admitted. Preserve H2's overlap handling: enumerate eligible IDs, assign at most one ID by normalized squared translation/rotation distance and deterministic cluster-ID tie-break. Report multiply eligible cases. This is archive-ID recovery, not a proof of distinct dynamic minima.

Primary recall averages the nine frames equally; secondary micro recall counts recovered IDs/22. RANDOM2 reports each replicate and median/min/max over the three replicate macro curves, without unioning all three at budget B. Its physical total validation cost is 3B calls/frame.

Compute conditioned AUC over the full prefix1..16. Historical H2 FULL6D/WEAK2 are recomputed on the same 1..16 interval using the already frozen 100 matched permutations; keep their original 1..263 AUC separately to avoid comparing incompatible intervals. Compare conditioned per-frame recall with H2 WEAK2's historical permutation median at the tested budget. Also report an additional same-index old-WEAK/FULL control by looking up the conditioned-WEAK selected seed indices in the completed H2 results, without any new alignments; this exposes the difference between conditioning and changed selection order.

Mechanism targets are the six H2 final misses: 2226/P05,2350/P01,2350/P05,2722/P05,3341/P02,3341/P03. Only after all proposals/results are fixed, audit predictor complement coordinates against archived successful FULL6D seeds, reporting distances to their componentwise median and nearest member, plus empirical ranges. Never feed this diagnostic into proposals.

## Controls and objective selection

For 23 no-major frames, cluster only the new returned terminals plus T0 for descriptive proliferation. Use deterministic complete-link pose distance (translation0.2m AND geodesic rotation2deg), ordered by nominal then probe rank; do not recluster the oracle. Report all-finite and converged-only counts, objective improvement, best-candidate displacement, iteration-limit rate. No-major is not a claim that every alternative is spurious.

For each method/frame/B, select the minimum raw objective among T0 and all finite B returns, retaining nominal on an exact score tie. Archive the choice before loading GT. Post-hoc scoring alone reuses the existing P5 fixed first-pose reference alignment and official LiDAR/IMU transform; check the nominal error against the existing same-objective posthoc archive. No new trajectory alignment is fitted. Report translation and rotation error changes separately; use dimensionless diagnostic `sqrt((e_t/0.8)^2+e_r_rad^2)` only for the requested improved/same/worse summary, with numerical comparison tolerance1e-6. GT never changes a selected terminal or a scientific discovery gate.

## Decision gate

Choose the smallest B in4/8/12/16 satisfying: weak macro recall>=.70; weak1..16 AUC>strong and median random AUC; weak per-frame recall>=historical old-WEAK median for>=7/9 frames; recover>=2/3 primary IDs; achieve .70 at<=16 calls vs historical FULL6D median32. At B<=8 mark low-budget support. If no supported budget, report the requested failure category with all gates exposed; never extend the search budget.

For a failed support gate, 'conditioning helps but insufficient' requires positive weak-minus-old comparison on both common-range macro AUC and B16 macro recall, while recall remains<.70. 'Generic conditional search only' is used if weak reaches .70 with the primary IDs but fails observability-specific AUC superiority. Otherwise report predictor-conditioned weak search not supported. These operational boundaries are frozen before results; borderline cases are shown explicitly and not tuned after observation.

Report calls, iterations, per-probe and per-frame time for each budget. If scientific success costs about one second/frame at B16, record that it is not yet competitive online. Keep Release binary/input/source/RNG/selection/result hashes. No support continuation, support features, GT-guided search, transported W, posterior, covariance fusion or EKF integration.
