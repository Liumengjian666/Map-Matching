# R1C frozen-support branch contract

Fixed inputs are R1B GROUP A, T0 W/S, k=2 and L=.8m. The product chart is
`t=t0+.8 eta_t; R=Exp(eta_rotation) R0; eta=Wu+Sv`.
The objective is the exact negative PCL optimization score divided by the
unchanged source count, not a posterior NLL.

For fixed point/leaf membership s, E_s uses `frozenScore` only. All derivatives
are central FD with h=.001. A full mixed stencil constructs a symmetric Hessian;
its symmetry is algebraic, so separate directional tests must also be recorded.
No dynamic PCL gradient or Hessian is used in the new solver.

A certificate requires exact per-point support equality (not hash equality),
strong gradient norm <=1e-5, and strictly positive H_vv eigenvalues. Hessian
damping is allowed for an optimization step only, never for SPD certification.

Corrector: at most8 support rounds; each has Newton20, initial trust .10,
maximum trust .50,10 dyadic line-search candidates, frozen decrease >1e-12,
minimum trust1e-6. Repeated exact support after a change is a cycle. Support
equality with failed stationarity is not a cycle and cannot certify a node.

Predictor: `dv/dalpha=-H_vv^{-1} H_vu u_b`. Step .05, maximum .05, minimum
.005; halve failed attempts, no random reset. Failed seeds must not be replaced
by another solver, full NDT, relaxed tolerance, or oracle state.

The existing evaluator transforms points in float. FD/strict stationarity
results must retain this numerical caveat without silently changing h or
evaluator precision. No production path is modified.

## Independent derivative checks and inference boundary

The numerical-helper self-test uses an analytic SPD quadratic with weak/strong
coupling. Real T0/canonical checks use3 deterministic normalized directions for
each of7 basin targets and2 supports:42 independent directional checks. Main
FD partial/mixed stencils and directional check steps are all .001. Diagnostic
gradient agreement requires absolute error<=1e-4 OR relative error<=.02;
curvature agreement requires absolute error<=.02 OR relative error<=.05. These
uniform thresholds were fixed before the formal execution; they do not relax
the branch stationarity norm<=1e-5. Hessian symmetry and Huv/Hvu asymmetry from
a shared mixed stencil are algebraic consistency checks, not an independent
proof of derivative accuracy.

An uncertified root ends that direction. No predictor, random reset, relaxed
gate, or full NDT is permitted to repair it. Accepted-node certification rate
is undefined when accepted nodes=0, not100% or a measured0% accepted-node rate.
Attempted certification fraction is reported separately. Seven basin targets
use only two frozen scan frames; the six tx616 forward roots are identical,
not six independent observations.

The data-dependent multibranch/barrier/local-recapture/event-spawn/full-refine
experiments are gated off when no starting point is certified. Their CSVs have
headers and zero observations. Zero observed escape is not evidence of safe
refinement when no refinement was attempted.

The present numerical implementation failing to certify roots does not prove
the IFT/support-branch model impossible. Float transformed-point quantization,
FD truncation, bounded solver convergence and nonstationary archived terminals
are unresolved contributors. The allowed result label identifies the current
contract as unclosed, not a mathematical disproof or support-only causation.
