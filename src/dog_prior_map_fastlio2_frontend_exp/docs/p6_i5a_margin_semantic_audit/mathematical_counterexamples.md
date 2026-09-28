# Mathematical counterexamples and local terminal sensitivity

## One true minimum, finite operational margin

Let `F(x)=0.5*x^2`. It is strictly convex and has the unique global minimizer
`x*=0`. Gradient descent with `eta=0.5` obeys `x_(j+1)=0.5*x_j`; after the
fixed budget `K=3`, the terminal map is `R_3(x_init)=x_init/8`. For nominal
seed zero, unit prior variance, and a terminal acceptance tolerance of `0.2`,

`A(delta)=1 iff |delta/8| <= 0.2`, so the ideal operational margin is `1.6`.

The independent numeric test gives alpha `1.599` = **ACCEPTED** and alpha
`1.601` = **REJECTED**. Thus a finite operational margin can exist while every
finite initialization follows the same true attraction basin. The toy treats
the fixed-step terminal output as a successful finite-budget registration and
does not model PCL `hasConverged()`; it is not a Floor01/PCL result.

## Local terminal sensitivity

Define the map-frame product-tangent terminal error
`e_terminal(delta)=[Log(R_terminal R0^T), p_terminal-p0]`. If the terminal
mapping is locally differentiable at the nominal seed, then
`e_terminal(delta)=J_registration delta + O(||delta||^2)`. With a whitened
direction `delta=alpha L u`, partition `J_registration` into rotation and
translation blocks `J_R` and `J_t`; locally,
`e_terminal(alpha) ~= alpha J_registration L u`. The first-order operational
exit radius is

`m_linear(u)=min(0.20/||J_t L u||, (2*pi/180)/||J_R L u||)`,

with a zero denominator interpreted as `+infinity` for that term. This is a
local approximation only: it requires small perturbations, local
differentiability, and fixed nominal convergence behavior. It does not
estimate the real Floor01 registration Jacobian. Residual initialization
sensitivity in a finite-iteration optimizer can therefore cause a finite
operational margin without any optimizer basin switch.

## Covariance scale identity

For a fixed physical acceptance set and full-rank prior `P`, scaling the
covariance to `P'=cP` with `c>0` gives `(P')^-1=(1/c)P^-1`. Therefore

`m_B^op,*(cP) = inf_(delta outside B0op) sqrt(delta^T ((1/c)P^-1) delta)
              = m_B^op,*(P)/sqrt(c)`.

This is an identity for the ideal full-rank quantity. It is not required to
hold pointwise for the existing finite-grid, capped estimator, whose sampled
directions, alpha grid, and censoring can change discretely. No covariance is
rescaled and no NDT is rerun for this audit.
