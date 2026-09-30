# Basis relinearizer call audit

The production call graph freezes each active degenerate LiDAR basis in
`buildLidarIterationSnapshot()`. `blockLinearizedSystemWithLidarSnapshot()` and
`objectiveWithLidarSnapshot()` only call the frozen-projection linearizer.
Therefore candidate evaluation contains no basis callback path.

Evidence:

- The synthetic optimizer fixture has a state-counting basis callback. With
  trace enabled and disabled, the callback count increases by exactly one per
  optimizer outer iteration, not once per candidate. The state and optimizer
  outcome match exactly with diagnostics on/off.
- Candidate objective calls leave the counter unchanged.
- The real P3-100 trace contains 582 optimizer rows; every recorded
  `candidate_basis_relinearization_calls` value is zero. This trace field is
  corroborating diagnostics; the independently instrumented synthetic callback
  count is the direct call-count assertion.
- In the captured run, repeated snapshots after rejected candidates had a
  maximum projector difference of zero. Across accepted outer updates, the
  snapshot generation advanced and projectors were allowed to change.

The trace-only retry abort guard was removed during final review. Projector
repeat difference remains observational and cannot change optimizer policy.
