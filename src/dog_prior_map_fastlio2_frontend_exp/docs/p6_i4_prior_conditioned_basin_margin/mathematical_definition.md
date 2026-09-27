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
translation. The linearized map from the complete IKFoM error state to this
pose tangent is `J_pose`; it retains every orientation/position cross term:

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

## Operational mode and basin

For two converged terminal poses `A` and `B`,

\[
sameMode(A,B)\iff
\|p_A-p_B\|_2\le0.20\;\mathrm m
\quad\land\quad
\operatorname{angle}(R_A^TR_B)\le2.0^\circ.
\]

The inequalities are inclusive. A non-converged result is never classified as
the nominal operational mode. Objective and fitness do not enter this label.
Define the nominal operational basin

\[
\mathcal B_0=\{\delta:\kappa(\mathcal R_k(T_{minus}\boxplus\delta))
                         =\kappa(M_0)\}.
\]

The ideal prior-conditioned operational basin margin is

\[
m_B^*=\inf_{\delta\notin\mathcal B_0}
       \sqrt{\delta^TP_{pose}^{\dagger}\delta}.
\]

This is not pose error, solution correctness, GT likelihood, objective quality,
or posterior probability. It measures how far the nominal attraction mode is
from an operational mode change under the filter-reported prior metric.

## Finite estimators used in this experiment

`PRINCIPAL_MARGIN` checks both signs of every positive-support covariance
eigenvector. For each ray it visits `alpha=0.25,0.50,...,3.00`, finds the first
sampled same-to-different transition, then bisects until the interval width is
at most `0.01`. The reported ray margin is the different-side endpoint. If no
switch is found through 3.0, the ray is right-censored (`>3`). This finite grid
can miss narrow switch-and-return regions and does not establish a global
nearest boundary.

`DENSE_DIRECTIONAL_REFERENCE_MARGIN` takes the minimum over the same twelve
signed principal rays and both signs of 32 extra fixed whitened-space unit
directions (deduplicated). The 32 vectors are generated before NDT from
PCG64(seed `20260928`), standard-normal sampled and normalized. If a frame has
rank less than six, the first `r` components are retained and renormalized.
The result is a finite directional reference, not the exact global margin.
Including the principal rays implies `m_dense <= m_principal` for finite pairs
up to numerical tolerance `0.01`.

The protocol's empirical retention curve uses only the 32 extra direction
families and both signs (so it is independent of the principal-ray estimator):

\[
S(\alpha)=\frac{\#\{\text{extra probes still in the nominal mode}\}}{64},
\quad \alpha\in\{0.5,1,2,3\}.
\]

`S(alpha)` is empirical directional retention, not a Gaussian probability.
Probe cache keys are frame transaction, ray id, sign, and alpha quantized to
`1e-6`; cache lifetime is one frame only.

The retention directions are also included in the dense-reference search.
Thus the prescribed `m_dense` versus `S(alpha)` association is not a held-out,
statistically independent validation: it measures consistency on a shared
fixed extra-ray set. The separate `m_principal` versus `S(alpha)` correlation
does not share principal rays with the retention sample. This dependency is
reported as a limitation and does not change the task's literal numerical
gate.
