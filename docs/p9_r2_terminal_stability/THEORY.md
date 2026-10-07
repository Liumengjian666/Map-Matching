# P9-R2: Low-Budget Terminal Stability Evidence

## Question and scope

For a frozen scan/map pair, do a small number of deterministic perturb-and-reoptimize probes produce a more dispersed set of NDT terminal poses on the frozen major-competitor frames than on the frozen no-major frames? If so, is the effect specifically stronger when the perturbations lie in the locally weakest observability subspace?

This is an offline optimizer-sensitivity experiment. Its frame statistic is not a posterior, basin probability, covariance, or proof of a dynamic multi-attractor. Frozen major/no-major labels are loaded only after the proposal manifest and all NDT results are fixed. No canonical competing-basin pose, basin ID, GT, trajectory error, support feature, or EKF state is used.

## Frozen inputs and nominal frame

The cohort is the 32-row frozen Floor01 cohort in the R1 archive. For each frame, the proposal source is exactly the archived base seed pool with `seed_index=0..262`; any targeted extension rows (`seed_index>=263`) are excluded. The nominal pose `T0` and ordered U_obs eigensystem are read from that frame's frozen `dual_u.csv` row. The official frozen map, source clouds, preprocessing provenance, and archive hashes are checked before execution.

For every archived start pose `T_start,j`, the proposal chart is the P9 map product chart:

`eta_j = [(t_start-t0)/0.8m, Log_spatial(R_start R0^T)]`.

The archived pose is round-tripped through the same P9 `mapChartDisplacement()` / `poseAtEta()` implementation. The fixed pre-run acceptance limits are 1e-5 m translation and 1e-4 degree rotation.

## Proposal pools and deterministic selection

Let `Q` be one orthonormal 6x2 basis. The projected proposal is `eta_Q,j = Q Q^T eta_j`, and its two-dimensional selection coordinate is `c_Q,j=Q^T eta_j`. Projection is not renormalized. The three compared proposal types are:

- `WEAK2`: the first two ascending-curvature U_obs eigenvectors;
- `STRONG2`: the last two ascending-curvature U_obs eigenvectors;
- `RANDOM2`: three independently frozen Haar 2D bases per frame.

Each 2D pool is ordered by deterministic farthest-point selection using Euclidean distance in `c_Q`: begin with the archived seed nearest the origin; then repeatedly select the seed whose minimum squared distance to the selected set is largest. Exact/numerical ties within `1e-12*max(1,|largest squared distance|)` go to the lowest seed index. No projected proposal is removed, and only the first 16 selected proposals are evaluated. Budgets are the nested prefixes B=4, 8, 12, 16.

Random bases use NumPy `Generator(PCG64)`. For cohort-order frame index `i` and replicate `r=0,1,2`, the seed is `20261007 + 1000*i + r`. A 6x2 standard-normal matrix is QR-factorized; each column's sign is fixed so the corresponding diagonal entry of R is nonnegative. The complete basis manifest is archived before any NDT alignment.

## NDT execution

WEAK2 and STRONG2 each use at most 16 calls/frame. RANDOM2 uses three frozen subspaces and at most 16 calls per subspace/frame. A selected probe is aligned once, regardless of convergence; the returned finite terminal is retained, with convergence and iteration-limit status reported separately. All methods use the same frozen map, source, preprocessing, and PCL 1.10 settings: resolution 0.8 m, outlier ratio 0.55, step 0.08, transformation epsilon 1e-5, and maximum 80 iterations. No FULL6D calls are run in R2; FULL6D is not an arm of this terminal-sensitivity experiment.

Raw PCL NDT score is recorded as a secondary diagnostic only. `delta_raw_ndt_score_sum = E(T_terminal)-E(T0)` is never converted to a probability or weight.

## Terminal-instability tensor

For each returned terminal `T_j*`, compute the same map product displacement from T0:

`xi_j = mapChartDisplacement(T0,T_j*)`.

At budget B:

`A_terminal(B) = (1/B) sum_{j=1}^B xi_j xi_j^T`.

The primary scalar is `S_terminal(B)=sqrt(lambda_max(A_terminal(B)))`. Secondary summaries are `trace(A_terminal)`, median and maximum `||xi_j||`, and nominal-return fraction. A return is defined only by the fixed geometric neighborhood translation <=0.2 m AND rotation <=2 degrees from T0; it is not an oracle-basin test. Nonconverged finite terminals remain in this geometric sensitivity statistic because their returned endpoints are part of the actual optimizer response; their status is reported.

## Frame-level evaluation

The independent statistical unit is the frame: 9 frozen major-competitor frames versus the remaining 23 no-major frames. For each method/budget, the report contains the score distributions, ROC AUC, and a one-sided frame-label Monte Carlo permutation p-value with 10,000 PCG64 permutations (seed 20261008, plus-one correction). LOFO classification holds out one frame at a time; its threshold is selected only from the other 31 frames by maximum balanced accuracy, with ties resolved to the highest threshold. All fold predictions and training-only thresholds are archived.

RANDOM2 is reported for all three replicate bases. Its primary method-level comparison also uses the per-frame median of the three scores; replicate AUC min/median/max are retained. Probe selection never uses labels. The H1 weak-concentration values and the major/no-major labels are joined only after all NDT output rows are complete.

The predeclared positive-evidence gate is the smallest B<=16 for which WEAK2 has AUC>=0.80, frame permutation p<0.05, LOFO accuracy>=0.80, and AUC greater than both STRONG2 and the RANDOM2 median-score arm. A qualifying B<=8 is the low-budget result. AUC/LOFO fold ranges and all per-frame records are shown to assess cohort dependence; this small frozen cohort does not support population-wide claims.

## Reproducibility and exclusions

The execution manifest records input/code/PCL-header/binary hashes, RNG seeds, selected probe order, preprocessing, build metadata, and run state. The result artifacts preserve every projected seed pool, round-trip audit, selected probe, NDT return, displacement, tensor, frame statistic, and LOFO fold. This task does not use GT, posterior inference, Laplace weights, support-transition features, continuation, multistart expansion, covariance fusion, or production EKF integration.
