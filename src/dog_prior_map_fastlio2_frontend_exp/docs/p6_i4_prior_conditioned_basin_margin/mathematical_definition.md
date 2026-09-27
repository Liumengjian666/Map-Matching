# P6-I4 mathematical definition

## Deterministic registration map

For the frozen scan-end source cloud `Z_k`, preprocessed prior map `M`, and
fixed PCL NDT parameters `theta`, define

\[
\mathcal R_k(T_{\mathrm{init}};M,Z_k,\theta)
  = T_{\mathrm{terminal}},
\]

where `T_terminal` is the terminal LiDAR pose returned by one deterministic
PCL NDT alignment from `T_init`. In this implementation, the replay context
stores the IKFoM `map_T_imu` prediction. The registration wrapper applies the
frozen `T_imu_lidar` extrinsic to form the actual `map_T_lidar` NDT seed;
probe results and operational mode comparisons are terminal `map_T_lidar`
poses. The filter state is never updated by a probe.

For one baseline frame, let `T_minus` and `P_state_minus` be the formal
closed-loop IKFoM prediction and covariance immediately before its nominal
NDT update. The nominal representative is

\[
M_0=\mathcal R_k(T_{\mathrm{minus}};M,Z_k,\theta).
\]

## Pose tangent and prior covariance

The physical tangent is the map-frame product tangent

\[
\delta=\begin{bmatrix}\delta\phi_{map}\\\delta p_{map}\end{bmatrix}
\in\mathbb R^6,
\quad
R_{seed}=\operatorname{Exp}(\widehat{\delta\phi_{map}})R_{pred},
\quad
p_{seed}=p_{pred}+\delta p_{map}.
\]

This is `SO(3) x R^3`, not an `SE(3)` exponential with a coupled `V(delta_phi)`
translation. The 6x6 pose covariance is projected from the full IKFoM
prediction covariance using `J_pose`, rather than formed from independently
diagonalized rotation/translation blocks. `J_pose` has nonzero columns only for
the pose rotation and position state components, so the resulting marginal
retains the rotation-position cross covariance; velocity, bias, gravity, and
extrinsic-to-pose cross terms do not directly appear in the 6x6 pose marginal:

\[
P_{pose}^{-}=J_{pose}P_{state}^{-}J_{pose}^{T},\qquad
P_{pose}^{-}\leftarrow\tfrac12(P_{pose}^{-}+(P_{pose}^{-})^T).
\]

The output coordinate order is
`[dphi_map_x,dphi_map_y,dphi_map_z,dp_map_x,dp_map_y,dp_map_z]`.
The right-error SO(3) state uses `J_R=R_est`; the additive map/world position
state uses `J_p=I`. No covariance block is extracted independently.

Write the self-adjoint eigendecomposition as

\[
P_{pose}^{-}=V\operatorname{diag}(\lambda_i)V^T.
\]

With `lambda_max=max(lambda_i)`, the numerical tolerance is
`1e-10 * max(lambda_max, 1e-18)`. An eigenvalue below the negative tolerance
makes that frame invalid. Only a negative eigenvalue within this roundoff
band is clamped to zero; no positive eigenvalue is regularized or discarded.
The effective rank is the number of positive eigenvalues. On positive support,

\[
P_{pose}^{\dagger}=V_r\operatorname{diag}(\lambda_i^{-1})V_r^T,
\quad L=V_r\operatorname{diag}(\sqrt{\lambda_i}),
\quad d_P(\delta)=\sqrt{\delta^TP_{pose}^{\dagger}\delta}.
\]

For any unit `u` in `R^r`, `delta(alpha,u)=alpha L u` has
`d_P(delta)=alpha`. `alpha` is a prior-metric radius, not a probability or a
calibrated number of standard deviations. All selected contexts in this run
are rank six; a future rank-deficient case needs separate interpretation of
the ideal infimum because the pseudometric has zero-cost nullspace directions.

## Operational nominal-basin acceptance set and margin

For a fixed scan/frame context `k`, let the nominal terminal LiDAR pose be

\[
M_0=\mathcal R_k(T^-;M,Z_k,\theta),
\]

where the nominal registration converges. For a prior perturbation `delta`,
let `T_delta = R_k(T^- boxplus delta; M,Z_k,theta)` denote the probe terminal
LiDAR pose when the probe converges. Define the operational acceptance
indicator relative to the one fixed nominal reference `M_0`:

\[
A_k(\delta;M_0)=
\begin{cases}
1,&\text{if the probe converges, }\|p_\delta-p_0\|_2\le0.20\;\mathrm m,
  \text{ and }\operatorname{angle}(R_0^TR_\delta)\le2.0^\circ,\\
0,&\text{otherwise.}
\end{cases}
\]

Equivalently, for a converged probe this is the product of the three
convergence/translation/rotation indicators; a non-converged probe is rejected.
The inequalities are inclusive. Objective and fitness do not enter acceptance.
The fixed-reference acceptance relation is not a pairwise equivalence relation:
the tolerance rule need not be transitive. No global mode label `kappa` or
equivalence class is induced by it.

The operational nominal-basin set is

\[
\mathcal B_0^{op}=\{\delta\mid A_k(\delta;M_0)=1\}.
\]

The ideal **Prior-Conditioned Operational Nominal-Basin Margin** is

\[
m_B^{op,*}=\inf_{\delta\notin\mathcal B_0^{op}}
       \sqrt{\delta^TP_{pose}^{\dagger}\delta}.
\]

This is a prior-conditioned operational stability margin: the filter-prior-
metric distance to the complement of the fixed nominal terminal-pose
acceptance set. It is an operational proxy for attraction-basin stability,
not a strict dynamical-systems attraction basin or its topology-exact global
boundary. It is not pose error, solution correctness, GT likelihood, objective
quality, failure probability, calibrated uncertainty, or posterior probability.

## GT and frozen cohort provenance

No GT pose/error values were read by the P6-I4 estimator, direction
generation, NDT probing, operational boundary search, retention computation,
or A--F verdict. Frame identities and strata were inherited from the frozen
P5-I2 manifest; therefore this experiment does not claim that the historical
cohort design was prospectively GT-independent. In `preparation_manifest.json`,
`gt_read=false` records only that the P6-I4 preparation script did not itself
open the GT file; it does not mean that the inherited manifest contains no
historical GT-derived metadata.

## Finite estimators used in this experiment

`m_principal` is the **principal operational margin estimator**: it checks both
signs of every positive-support covariance eigenvector. For each ray it visits
`alpha=0.25,0.50,...,3.00`, finds the first sampled accepted-to-rejected
transition under `A_k`, then bisects until the interval width is at most
`0.01`. The reported ray margin is the rejected-side endpoint. If no switch is
found through 3.0, the ray is right-censored (`>3`). This finite grid can miss
narrow switch-and-return regions and does not establish a global nearest
boundary.

`m_dense` is the **dense directional reference for `m_B^{op,*}`**: it takes the
minimum over the same twelve signed principal rays and both signs of 32 extra
fixed whitened-space unit directions (deduplicated). The 32 vectors are
generated before NDT from PCG64(seed `20260928`), standard-normal sampled and
normalized. If a frame has rank less than six, the first `r` components are
retained and renormalized. Both `m_principal` and `m_dense` are
finite-direction approximations to the operational margin, not estimates of a
topology-exact global attraction-basin boundary. Including the principal rays
implies `m_dense <= m_principal` for finite pairs up to numerical tolerance
`0.01`; this is an implementation-consistency property.

The protocol's empirical retention curve uses only the 32 extra direction
families and both signs, which do not overlap the principal-estimator rays:

\[
S(\alpha)=\frac{\#\{\text{extra probes still in the nominal mode}\}}{64},
\quad \alpha\in\{0.5,1,2,3\}.
\]

`S(alpha)` is empirical directional retention, not a Gaussian probability.
Probe cache keys are frame transaction, ray id, sign, and alpha quantized to
`1e-6`; cache lifetime is one frame only.

The retention directions are also included in the dense-reference search.
Therefore Gate E (`m_dense` versus `S(alpha)`) is a **shared-ray internal
semantic-consistency gate**, not held-out, independent, or external validation.
The separate `m_principal` versus `S(alpha)` association does not share
principal-estimator rays with the retention sample, but remains descriptive and
is not a fully independent experiment. These finite-direction limitations do
not change the frozen protocol's numerical gate.
