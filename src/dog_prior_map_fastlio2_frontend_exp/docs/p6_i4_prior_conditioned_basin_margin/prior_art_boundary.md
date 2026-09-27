# Prior-art boundary and claim discipline

This experiment does not claim that any of the following concepts is new:

- registration uncertainty that depends on initialization;
- propagation of initialization uncertainty through ICP or another nonlinear
  registration routine, including Unscented Transform approaches;
- multiple initial poses or multi-start registration;
- covariance over multiple NDT solutions;
- Hessian-guided seed placement;
- multi-start recovery or selection.

The only recorded `NOVELTY_CANDIDATE` is the proposed reliability coordinate:

> prior-conditioned operational nominal-basin margin, explicitly separated
> from local geometric observability.

The present study tests numerical and operational viability, finite-direction
consistency, and internal directional-retention consistency on one frozen
Floor01 sequence. The object is an operational proxy for attraction-basin
stability, not a strict attraction basin or a topology-exact boundary. It does
not establish novelty, priority, correctness prediction, improved localization,
runtime feasibility, or a complete dual-reliability method. `NOVELTY_UNVERIFIED`
remains the status until a separate literature review and later validation.
