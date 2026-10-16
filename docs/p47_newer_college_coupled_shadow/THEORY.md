# P47 Hessian chart and Coupled shadow math

## Frozen Local NDT objective

For each sampled P46 frame, the probe reuses the P46 CONTROL Source, the active P45 Tile IDs/Target, and frozen P46 initial and raw-NDT poses. The score kernel is the installed PCL NDT derivative/score implementation exposed by `ObservablePclNdt`; the research tool does not substitute a new registration objective.

PCL's NDT score is maximized. The existing `pullNativeJet()` transforms it to a minimization convention and normalizes by source count:

\[
E(\eta)=-S(\eta)/N,\qquad g=\nabla_\eta E,\qquad H=\nabla_\eta^2E.
\]

The existing P9/R6 pose chart is:

\[
p(\eta)=p_0+0.8\,\eta_t\;\text{meters},\qquad
R(\eta)=\operatorname{Exp}(\eta_r\;\text{radians})R_0.
\]

Thus all six \(\eta\) coordinates are dimensionless. The reported Hessian is not a raw meter/radian mixed-unit matrix. Its eigenvectors are directions in this fixed normalized local pose chart; the score Hessian is per source point.

## Frozen weak/strong split

The existing R6 rule first requires a finite positive-definite local Hessian. It chooses \(k=1\) when \(\lambda_1/\lambda_0\ge2\), otherwise \(k=2\); \(W\) contains the lowest-eigenvalue columns and \(V\) the remaining columns. This is a relative spectral split, not an absolute information threshold and not a claim that the NDT objective is singular.

For the 154 frozen real samples, every Hessian was finite and positive definite, and every split was two-dimensional. The median \(\lambda_{min}/\lambda_{max}\) was `5.097832e-4`; the range was `1.028186e-4` to `9.766273e-4`. This is substantial relative curvature anisotropy, while the smallest eigenvalue stayed positive (`1.84153` minimum in the normalized, per-point objective chart).

The subspace is locally repeatable but not globally fixed. Consecutive startup samples 1–10 had a median largest principal angle of `0.348°` and maximum `2.036°`. The maximum angle in the five preselected tile-switch neighborhoods ranged from `0.994°` to `11.511°`. Across all neighboring stored samples the median was `2.318°`, P95 `12.955°`, and maximum `68.123°`. The 68° change is evidence against treating one world-fixed axis as weak for the entire 180 s sequence.

## Paired arms and the actual strong correction

The single eligible pair is P46 frame 1. It passed the existing innovation trigger (`0.202266 m > 0.12 m`; rotation was `0.843°`, below `3°`) and had a valid two-dimensional Hessian. The same frozen Source, Local Target, nominal pose, initial pose/diagnostic reference, weak basis, and local objective were used in both calls to `runWeakCoupledRefinement()`; only `config.coupled` differed. Both runs produced the same weak candidate. A and the shared nominal are the frozen P46 raw-NDT pose; the parity replay is not substituted for it.

The nominal cross-block norm in the eigenbasis was `1.523e-12`, as expected from diagonalizing the nominal Hessian. In the Coupled arm, existing code recomputes a full joint jet at the displaced weak candidate, then invokes `coupledStrongStep(displaced, k, Zero(k), ...)`. Consequently, the exact system solved by this implementation at that stage is the displaced strong-residual solve

\[
(H_{vv}(\eta_u)+\lambda I)\,\delta v=-g_v(\eta_u),
\]

with no additional weak increment passed to that call. It is **not literally** a one-shot solve using the nominal blocks \(g_v(0)+H_{vu}(0)\delta u\). The displaced jet had a strong/weak block norm `6.924` and strong-gradient norm `23.577`; the LDLT system was well-conditioned enough under the existing guard (`1707.73`), residual `1.51e-16`, damping zero, and the nonzero strong correction was selected after score/objective checks. The implementation therefore exhibits a real nonlinear relinearized strong-residual correction, but this run does not establish that the explicit nominal linear cross-block term alone caused all of \(\delta v\). This distinction must be retained before any feedback integration.

The selected strong correction was `delta_v=[-0.0221453, 0.00415893, 0.00286754, 0.000207841]` in strong-basis coordinates, corresponding to `0.017718 m` translation norm and `0.289083°` rotation norm. It changed the objective from `-2.231117` (Weak-only) to `-2.311239` (Coupled; lower is better), with PCL score increasing from `3139.154` to `3251.324`. The candidate passed the existing rigid-pose, bound, local score, and solver checks. These are one-frame shadow metrics, not a trajectory result.

One further comparison caveat is important: the existing `runWeakCoupledRefinement()` computes its weak update using a Schur complement, `A=H_uu-H_uv H_vv^{-1}H_vu` (plus regularization), and a right-hand side containing `H_uv H_vv^{-1}g_v`. Thus the `Weak-only` arm means **the pose is updated only in the weak subspace**; it is not an optimizer that ignores all weak/strong coupling in its weak-step solve. The two arms share the same computed weak displacement (`delta_u=[0.146750859, 0.027142171]`) and weak candidate. The observed additional C-vs-B effect is the nonzero displaced strong-residual correction, not a comparison of a fully decoupled weak optimizer against a coupled optimizer.

## Why the prime alignment is present

`ObservablePclNdt::nativeJet()` directly calls PCL's protected `computeDerivatives()`. Production calls that backend only after `ndt_.align()` has run. The first offline probe called it before PCL had initialized its target-grid/registration internals and segfaulted in PCL. The final probe now follows production setup order for each sampled Source/Target: `setInputSource()` → one `align()` from the frozen P46 initial guess → verify convergence, iteration count, Fitness and output pose against the frozen P46 nominal → evaluate the jet at the frozen nominal. The parity tolerances were 1 mm, 0.01°, and `1e-5` absolute PCL Fitness; all 155 priming alignments passed. Maximum observed differences were `73.1 µm`, `0.000551°`, and `3.39e-6` Fitness. No replayed pose replaces the frozen P46 nominal.

The final run used 155 such counted `align()` calls: one for each of 154 Hessian samples, plus one fresh state primer for the one paired frame. Jet/value/align counts and timings are in `runtime_breakdown.csv`. P46 itself was not replayed continuously.
