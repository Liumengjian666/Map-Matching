# P62-R15B: nominal-association piecewise coupled refinement

## Scope and interpretation

This is a post-NDT diagnostic refinement prototype. It does not call `align()`, does not inherit the frozen NDT convergence flag, and cannot produce a formally accepted observation or EKF update. The 25 R15 frames and their maps, source splits, nominal poses, and causal predictions are reused unchanged.

Frames use `T_A_B` for a transform mapping coordinates from B to A. Both frozen poses are `T_M_L`. The local chart is a right/body perturbation

\[
T(z)=T_0\operatorname{Exp}(S z),\qquad
S=\operatorname{diag}(0.30,0.30,0.30,3\pi/180,3\pi/180,3\pi/180),
\]

where z is dimensionless, its first three coordinates are translation tangent coordinates in metres, and its last three are rotation tangent coordinates in radians. The reused R11 `Exp/Log` is the full SE(3) map: for \(\xi=(\rho,\phi)\), `Exp` uses \(R=\exp([\phi]_\times)\) and \(t=J_l(\phi)\rho\). Thus the translation in an SE(3) exponential is not treated as an independent post-rotation Euclidean displacement.

## Actual PCL 1.10 score and frozen associations

For each Training point \(x_i\), the PCL 1.10 object transforms it at the nominal pose using the same float `PointXYZ` transform as its optimizer. Denote that exact float baseline by

\[
y_i^0=\operatorname{floatTransform}(T_0,x_i).
\]

At \(T_0\), the actual `VoxelGridCovariance` `radiusSearch` (radius equal to the stored 0.8 m resolution) returns Gaussian leaves \(A_i(T_0)\). Their stable voxel-key identities and ordered scoring memberships are recorded. Each leaf k supplies the actual PCL mean \(\mu_k\), inverse covariance \(Q_k\), and the configured PCL constants \(d_1,d_2\). For a residual r, PCL's contribution is

\[
s(r)=-d_1\exp(-d_2 r^TQ_kr/2),
\]

retained only when \(0\le d_2\exp(-d_2r^TQ_kr/2)\le1\), exactly following `updateDerivatives()`' guard. The score is summed over Training points and their returned Gaussian leaves; the scalar minimized here is the exact negative normalized score, \(F=-\mathrm{Score}/N_{train}\), not Euclidean nearest-neighbour fitness.

For \(\Delta=\operatorname{Exp}(Sz)=(R_\Delta,t_\Delta)\), the continuous point on the nominal float baseline is

\[
y_i(z)=y_i^0+R_0\left(R_\Delta x_i+t_\Delta-x_i\right).
\]

The nominal leaf set is frozen, but residuals remain continuous:

\[
F_{fixed}(z)=-\frac{1}{N_{train}}\sum_i\sum_{k\in A_i(T_0)}s\big(y_i(z)-\mu_k\big).
\]

At z=0 the float baseline and the same ordered leaves reproduce the dynamic PCL score. For z≠0 this is a *local branch model*, not the dynamic-association NDT objective: it intentionally does not update \(A_i(T)\). It also preserves PCL's score cutoff and introduces no smoothing weight. Dynamic PCL score and association changes are evaluated separately at each candidate.

## Derivatives and numerical checks

The model uses central differences of this one fixed-association function only. The six-coordinate gradient uses fixed dimensionless step \(h_g=5\times10^{-4}\); the raw Hessian is the central difference of that gradient with \(h_H=8\times10^{-3}\). Its reported model is \(H=(H_{raw}+H_{raw}^T)/2\). An independent check compares \(g^Tv\) to scalar central differences at \(h_d=10^{-3}\) in six axes and four preregistered mixed directions, and compares \(v^TH_{raw}v\) to scalar second differences at steps 0.004, 0.008, and 0.016. Both sides of each derivative check use the same frozen association branch. Dynamic membership changes in those stencils are a separate diagnostic and do not gate these derivatives.

At z=0 the fixed score must agree with dynamic PCL score within \(10^{-12}\) after normalization and with PCL's own `computeDerivatives()` score within \(10^{-9}\) before normalization. Directional gradient relative error must be ≤0.02; raw Hessian antisymmetry ≤0.05; central scalar Hessian relative error ≤0.1 and adjacent step spread ≤2. Nonfinite or failed checks make the frame unmodelled.

## Geometry and spectral-tail probe

The right-scaled geometric matrix remains the R15 Gaussian-weighted Gauss–Newton matrix

\[
G_z=\frac{1}{N_{train}}\sum_{i,k}w_{ik}J_{ik}^TQ_kJ_{ik},\quad
w_{ik}=(-d_1)d_2e^{-d_2r_{ik}^TQ_kr_{ik}/2},\quad
J_{ik}=R_0[I,-[x_i]_\times]S.
\]

It is not PCL's optimizer Hessian and not the fixed-branch Hessian. Eigenvalues are ascending and eigenvectors orthonormal. The formal degeneracy rule stays \(\lambda_i/\lambda_{max}\le0.01\). For this experiment, regardless of whether that rule finds a weak direction, one *exploratory only* direction is the eigenvector of the minimum eigenvalue. This is `SPECTRAL_TAIL_MECHANISM_PROBE`; it is not a formal weak direction and no result from it counts as degeneracy recovery.

The R6 geometry comparison is reported two ways, and neither is silently promoted to a pass. The `same_path_algebraic_conversion_relative_error` diagnostic reuses the same transformed float points, PCL associations, residuals, and Gaussian weights as direct \(G_z\), but evaluates an algebraically equivalent legacy Jacobian factorization before coordinate conversion. It is not bitwise-identical Jacobian arithmetic: the direct path evaluates \(R_0[I,-[x_i]_\times]S\), while the comparison path evaluates \([0.8I,-[R_0x_i]_\times]\) and then converts coordinates. Equality assumes an exactly orthonormal rotation and exact arithmetic. The legacy R6 `geometry()` path also transforms point coordinates in double before converting the query to `PointXYZ`, unlike the direct PCL float transform. Both relative discrepancies are reported at the original \(10^{-7}\) threshold; neither threshold is raised, and the historical R15 result is not relabelled as passed.

## Paired perturbations and coupled response

For \(\xi_{pred}=\operatorname{Log}(T_0^{-1}T_P)\), define \(z_{pred}=S^{-1}\xi_{pred}\). For the single exploratory minimum-spectrum vector u, \(d_w=\alpha u^Tz_{pred}\), \(\alpha\in\{0.5,1.0\}\), uniformly clipped to the fixed unit Euclidean trust ball. Physical displacement consequently remains bounded by 0.30 m translation norm and 3 degrees rotation norm. This deterministic projection is not a covariance model and does not interpret the 0.8 m / 5 degree admission limits as uncertainty.

In the orthonormal eigenbasis, let s denote the five-coordinate complement to the one probe direction w. With the same \(d_w\) for both ablations, `WEAK_ONLY_PROBE` fixes \(d_s=0\); `COUPLED_PROBE` solves

\[
(H_{ss}+0.01 I)d_s=-(g_s+H_{sw}d_w)
\]

by LDLT without explicit inverse. The fixed regularizer is \(\lambda=0.01\); raw and regularized blocks must be finite, positive definite, and conditioned ≤1e8. The unregularized raw block's antisymmetry must be ≤0.05. The deterministic combined unit trust ball can shrink only \(d_s\), preserving the paired \(d_w\). Failed strong solves do not erase an otherwise valid weak-only diagnostic. A resulting pose is always marked `POST_NDT_REFINEMENT_CANDIDATE`, `converged=false`, and formally ineligible.

## Comparisons and evidence

The comparisons keep separate (a) quadratic prediction, (b) fixed-branch objective, (c) dynamic PCL objective and association changes, (d) Coverage V2 held-out losses and common-weight paired advantage. R12's strict \(D>0.02\), formal 0.8 m / 5 degree admission, and overlap ≥0.50 are unchanged; they are diagnostic only for these post-NDT candidates. No candidate is fed to an EKF.
