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
