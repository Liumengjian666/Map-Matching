# P6-I5B mathematical definition and scope

## Frozen acceptance and response

For every frame the nominal terminal LiDAR pose is the frozen P6-I4 `M0`.
Acceptance is unchanged: the saved probe converged, translation separation
from `M0` is at most 0.20 m, and rotation separation is at most 2 degrees
(with only the frozen I5A 1e-12 degree angular roundoff allowance).

For a terminal pose `(R_terminal, p_terminal)`, the response is

`e(delta) = [phi_map; t_map]`,
`phi_map = Log(R_terminal R0^T)`,
`t_map = p_terminal - p0`.

The coordinate order is `[rotation_map, translation_map]`. `phi_map` is a
map-frame left tangent. Euler subtraction and `Log(R0^T R_terminal)` are not
used. Translation and rotation remain separate physical quantities.

## Empirical principal secant

For each dense frame, the six columns use only P6-I4 principal ray IDs 0..5,
with the original saved signed probes at `alpha=0.25`:

`J_w[:,j] = (e(+0.25 e_j) - e(-0.25 e_j)) / (2 * 0.25)`.

`J_w` is an empirical local terminal-response secant matrix at finite step
size. It is not an NDT Hessian, an analytic registration Jacobian, or a new
`U_nonlocal` estimator. No covariance eigendecomposition or basis-sign
reconstruction is performed. Extra rays never enter `J_w` estimation.

For each whitened principal axis the even residual is
`b_j = (e(+0.25 e_j) + e(-0.25 e_j))/2`. Its rotational norm is reported in
radians and divided by the frozen 2-degree tolerance only as a normalized
descriptive value. Its translational norm is reported in meters and divided
by 0.20 m separately. The raw rotation and translation values are never
added.

## Strict support

A frame is `STRICT_LOCAL_SUPPORT` iff all twelve signed alpha=0.25 training
probes both converged and passed the frozen P6-I4 acceptance. All other frames
are retained as `LOCAL_SUPPORT_CONTAMINATED` stress samples; none are silently
deleted. Transaction 1 is classified by the same rule and is not excluded
based on its startup covariance scale.

## Held-out prediction

For each saved unit extra direction `u` (D01..D32 from the frozen file), each
sign, and fixed `alpha in (0.5, 1, 2, 3)`:

`e_hat(alpha,u) = alpha * J_w * (sign * u)`.

The predicted pose is `R_hat = Exp(phi_hat) R0`, `p_hat = p0 + t_hat`.
Geometric predicted acceptance uses `||t_hat|| <= 0.20 m` and
`||phi_hat|| <= 2 degrees`; this does not predict PCL convergence. If
`||phi_hat|| > pi`, the row is marked `OUT_OF_LOCAL_MODEL_DOMAIN`; raw linear
classification is still reported with that caveat and an in-domain subset is
also provided.

Terminal translation error is `||p_hat-p_terminal||`. Terminal rotation error
is the SO(3) geodesic angle of `R_hat^T R_terminal`. Metrics are reported both
for all actual probe outputs and for converged-only outputs.

Actual acceptance labels are joined from saved `EXTRA_MARGIN` and
`EXTRA_RETENTION` streams with the frozen I5A key `(transaction_id, ray_id,
sign, round-half-up(alpha / 1e-6))`. Duplicate logical keys must agree in
convergence, terminal pose, recorded acceptance, and recorded separations.
The 32 one-based direction IDs D01..D32 map to ray IDs 100..131. The I5A
coverage CSV is independently checked against this reconstruction.

## Local predicted exit radius

For each signed extra direction:

`alpha_local = min(0.20/||J_t u||, (2 degrees)/||J_R u||)`,

with zero denominators mapped to infinity. This is a local linear-model exit
radius, not the P6-I4 measured operational margin. Actual extra-ray first
sampled exits are reconstructed offline from saved `EXTRA_MARGIN` CSV rows
using the frozen 0.25 grid and saved bisection probes. No NDT is rerun. A ray
with no detected transition through alpha=3 is right-censored, not treated as
an exact alpha=3 boundary. No posthoc scale is fitted.

## Synthetic quaternion direction test

The script synthesizes a non-identity `R0`, applies a small map-frame left
rotation by quaternion composition, and verifies `Log(R_terminal R0^T)`
recovers that map-frame perturbation. It separately checks the right/body
coordinate relation and quaternion sign equivalence. Result:
`PASS`, left error
`7.41e-17` rad, right-coordinate relation error
`7.85e-17` rad.

## Interpretation limits

This is same-frame cross-direction held-out validation over a frozen
24-frame sequence. Directions within a frame are correlated and are not
independent scenes. Failure of the local secant prediction does not prove
basin hopping: higher-order smooth nonlinearities, finite iteration effects,
correspondence changes, solver termination, or other terminal-map behavior
remain alternatives. The I5A GT-derived posthoc CSV and all official GT are not
read by this analysis.
