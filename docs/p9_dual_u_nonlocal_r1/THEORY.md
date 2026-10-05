# P9 reduced-subspace nonlocal prototype: mathematical contract

## Scope

This is an offline prototype on frozen same-objective PCL 1.10 NDT evidence. It does not modify the production EKF, run a full trajectory, or use GT. The archived exhaustive search is an optimizer-landscape oracle, not an online method.

## Basin-mixture decomposition

For candidate basins (b), write the scan/map posterior as a mixture

\[
p(x\mid z,M)=\sum_b \pi_b p(x\mid b,z,M),\qquad \sum_b\pi_b=1.
\]

With basin means \(\mu_b\), within-basin covariances \(\Sigma_b\), and \(\bar\mu=\sum_b\pi_b\mu_b\), the total covariance decomposes as

\[
\Sigma_{\rm total}=\underbrace{\sum_b\pi_b\Sigma_b}_{\text{within-basin}}+
\underbrace{\sum_b\pi_b(\mu_b-\bar\mu)(\mu_b-\bar\mu)^T}_{\text{between-basin}}.
\]

The first term is the role intended for local directional observability; the second is the role intended for nonlocal basin ambiguity. The current experiment can discover neither calibrated \(\pi_b\) nor a valid posterior covariance, so it does not instantiate these probability-weighted tensors.

## Reused U_obs chart

The frozen R1 product chart uses

\[
\eta=[\delta t_{map}/L,\;\delta\theta_{map}],\quad L=0.8\;{\rm m},
\]

with additive map-coordinate translation and left/map-spatial rotation about the LiDAR origin:

\[
t(\eta)=t_*+L\eta_t,\qquad R(\eta)=\operatorname{Exp}(\eta_\theta)R_*.
\]

This is not a six-dimensional left SE(3) twist. The archived U_obs eigenvectors are columns of the symmetric curvature eigensystem in this dimensionless chart, ordered by ascending curvature. The reduced search uses the first (k\le2) vectors as \(W\) and the remaining vectors as \(S\). The PCL-native derivative coordinates are \([t_x,t_y,t_z,\mathrm{roll},\mathrm{pitch},\mathrm{yaw}]\) with the PCL (R_xR_yR_z) convention; R1's pullback includes the chart Jacobian and second-derivative term.

Reusable R1 implementation was found on the frozen `research/dual-u-architecture-r1` line:

- `dual_u_architecture.cpp`: `analyzeWithinBasinObservability`, `buildMapProductChartPullback`, `applyMapProductChartIncrement`.
- matching `dual_u_architecture.hpp` declarations.
- R1's frozen-support and dynamic-support diagnostic hooks were not present in the clean P9 baseline and were not cherry-picked. P9 contains only the offline objective/chart evaluator needed here.

R1's real-frame frozen-support FD validation covered tx120, tx838, tx1359, and tx2350. Dynamic PCL support changes are separate from that mathematical branch check and are not interpreted as a smooth global Hessian.

## Exact PCL 1.10 NDT score and evaluation energy

For transformed source point \(x_i(T)\), PCL 1.10 radius-searches target Gaussian cells \(g\) within the configured NDT resolution. Each returned cell contributes

\[
q_{ig}=(x_i(T)-\mu_g)^T\Sigma_g^{-1}(x_i(T)-\mu_g),\qquad
e_{ig}=\exp(-d_2q_{ig}/2),
\]

\[
s_{ig}(T)=-d_1e_{ig},\qquad
S(T)=\sum_{i=1}^{N}\sum_{g\in A_i(T)} s_{ig}(T),\qquad E(T)=-S(T).
\]

The implementation retains PCL's derivative guard \(0\le d_2e_{ig}\le1\); with this experiment's \(\rho=0.55\) and resolution (0.8\) m, \(d_2=0.528248766985\), so the guard is inactive for finite nonnegative Mahalanobis distance. Constants are

\[
c_1=10(1-\rho),\quad c_2=\rho/r^3,\quad d_3=-\ln c_2,
\]

\[
d_1=-\ln(c_1+c_2)-d_3,\quad
d_2=-2\ln\frac{-\ln(c_1e^{-1/2}+c_2)-d_3}{d_1}.
\]

At \(\rho=0.55,r=0.8\) m: \(c_1=4.5\), \(c_2=1.07421875\), \(d_1=-1.646558519810\), and \(d_2=0.528248766985\). PCL maximizes the raw score sum. The offline evaluator reports both \(E=-S\) and \(\bar E=-S/N\); within one frozen frame/map the fixed (N) preserves ranking. PCL's `getTransformationProbability()` is `score / N`, and its own source comments that normalization constants must be changed for global accuracy. `getFitnessScore()` is instead a nearest-neighbor distance statistic.

Important: this is an exact, comparable *PCL optimization energy for the same frozen scan/map/grid*, not a normalized likelihood or posterior NLL. There is no per-Gaussian determinant normalization, no mixture normalization over the changing support set, and no calibrated basin prior. Therefore it does not justify a Laplace evidence or posterior basin probability.

The target Gaussian means/covariances/inverse covariances are PCL `VoxelGridCovariance` cells. PCL 1.10 defaults to at least six points per cell and regularizes the minimum covariance eigenvalue to (0.01\) times the maximum. Every point can contribute multiple returned cells; membership is dynamic `radiusSearch` at the NDT resolution.

Source references from the installed runtime headers:

- `/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp:59-64`: score constants.
- `.../ndt.hpp:199-225`: per-source-point radius search and accumulation over neighboring cells.
- `.../ndt.hpp:352-367`: Gaussian contribution and derivative guard.
- `/usr/include/pcl-1.10/pcl/filters/impl/voxel_grid_covariance.hpp:287-345`: cell mean, covariance, and eigenvalue conditioning.
- `/usr/include/pcl-1.10/pcl/registration/impl/registration.hpp:117-145`: nearest-neighbor `getFitnessScore()`.
- `.../ndt.hpp:170-172`: PCL's warning about transformation-probability normalization.

## Frozen-support derivative check

For each of 39 archived nominal/competitor terminals, the evaluator freezes the center pose's exact radius-search Gaussian-cell identities for each source point. It then evaluates \(E_A(T)\) without changing those identities. Directional finite differences use the weakest and strongest U_obs chart eigenvectors and step sizes \(0.02,0.01,0.005,0.0025,0.001,0.0005\).

The reported normalized discrepancy is \(|a-b|/\max(1,|a|,|b|)\). Across 78 terminal/direction pairs, best-step frozen-support gradient discrepancy was below 0.005 for 78/78 (median (6.19\times10^{-6}), max 0.00298); directional curvature discrepancy was below 0.005 for 78/78 (median (1.91\times10^{-4}), max 0.000771). This validates the local frozen-support derivative implementation for the sampled directions. It does not validate a smooth dynamic-support Hessian: dynamic-support FD met the same tolerance for only 10/78 gradients and 1/78 curvatures.

Archived PCL terminal flags are not a stationarity certificate: PCL sets its convergence flag when the iteration limit is reached as well as on its step stopping conditions. In this sample, chart-gradient norms ranged from `1.26581e-5` to `4.54034` (median `0.132767`, linearly interpolated P90 `1.23605`); only 18/39 were at or below 0.1. This is reported as a stationarity limitation, not hidden by the exact score match.

## Reduced weak-subspace profile search

The intended profile is

\[
\Phi(u)=\min_v E\bigl(\eta=Wu+Sv\bigr).
\]

P9 uses a bounded offline approximation: at each weak coordinate it starts (v=0), performs at most one damped strong-subspace Newton/Gauss-Newton step with line search, then evaluates the exact dynamic-support PCL energy. A 1D profile uses 17 grid coordinates; a 2D profile uses a 9-by-9 grid with points outside the physical fallback limits omitted. Interior discrete local minima receive one full frozen-parameter PCL NDT refinement. Terminal clustering is complete-link with 0.2 m translation and 2 degree geodesic-rotation cutoffs; pairwise relations use an SE(3) logarithm.

The frozen archive has no prediction covariance, so the authorized fallback is used and labeled: each weak-axis coordinate is bounded to at most 2 m translation or 15 degrees rotation, with the combined weak displacement subject to the same limits. This is an experimental search bound, not a probabilistic confidence region.

Oracle `major competing basin`: a supported primary archive cluster separated from the nominal terminal by more than 0.2 m or 2 degrees, and within 5% of the best supported archive score per source point. Recovery additionally requires the refined reduced candidate to belong to a non-nominal reduced terminal cluster and fall within 0.2 m / 2 degrees of the oracle cluster. No GT or reference trajectory is involved.

## Probability and U_nonlocal status

Laplace weights require a normalized, comparable basin posterior energy and a valid local support/curvature approximation. The exact PCL score meets only the first *optimization-energy comparability* need, not the posterior-NLL requirement. Dynamic support is also discontinuous and many archived terminals are not stationary. Thus \(\pi_b\), the weighted `U_nonlocal` second moment, `U_within`, and `R_candidate` are deliberately `INDETERMINATE` / not computed. The experiment reports unweighted pose displacements and their weak-space projection only as diagnostics, not as an ambiguity covariance or principal eigenvector.
