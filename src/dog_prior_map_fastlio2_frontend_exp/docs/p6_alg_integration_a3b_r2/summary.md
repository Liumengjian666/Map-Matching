# PAPER-P6-ALG-INTEGRATION-A3B-R2

## Outcome

The optimizer now uses the required outer-relinearized / inner-frozen LiDAR
subspace contract. Synthetic Release and Debug tests pass. The one P3-100
short real-link run did **not** pass: it advanced beyond tx83 and stopped at
tx90 during marginalization because the generated prior failed its finite-PSD
validation. Per the stop rule, P3-200 was not run. This is not a localization
accuracy result.

## Frozen-subspace implementation

Every outer iteration snapshots each active degenerate factor's basis and
symmetrized projected covariance `B^T R B`, keyed by observation ID and stamp.
The same snapshot supplies `H/g/current surrogate cost` and candidate
acceptance. Candidate evaluation computes only the raw pose residual/Jacobian
under that frozen projection; it does not call the basis relinearizer or
recompute projected covariance. A subsequent outer iteration rebuilds the
projection at the updated state. The retry projector-difference metric is
diagnostic-only; final review removed a trace-only abort condition so tracing
cannot change optimizer control flow.

No damping schedule, step threshold/clip, iteration count, acceptance rule,
NDT/U_obs/U_nonlocal/noise/NIS/window/IMU/Schur definition, visual data, or GT
was changed.

## Tests and real-link result

- Release build: PASS; all 26 CTests PASS.
- Debug targeted tests: 5/5 PASS, including A1-R1 Schur and A3B-R1 forensic
  regressions.
- Synthetic checks cover dynamic-basis mismatch vs frozen FD consistency,
  actual basis callback counts through optimizer iterations, exact projected
  covariance freeze, next-outer projector change and consistency, independent
  full-rank residual/Jacobian/H/g/cost oracle, and trace on/off parity.
- P3-100: 38 scan-end terminals completed, 39th terminal (tx90) stopped at
  `marginalized_prior_not_finite_psd` after the optimizer itself reported an
  accepted update. Tx83 was passed. The run is not a pass because marginal
  prior PSD validation failed and pre-marginalization span at tx90 was
  `2.017077923 s`.
- P3-200: NOT RUN under stop-on-first-failure.
- No GT, ATE/RPE, full 2777-scan replay, or formal accuracy experiment.

The complete R2 audit and run evidence is alongside this summary. The real
failure is preserved without changing its thresholds or retrying the run.

## Git

`START_SHA=749b31e2fd6ca4ffa1d00df3091eb041a9bdc664`.
`CODE_SHA=29f9338685e39b4db9173a11af96f4b71ce1c0a0`.
The final report/artifact commit is `END_SHA` and is reported in the final
handoff after Git verification.

`FROZEN_SUBSPACE_CONTRACT_ENGINEERING_PASS = NO`.
`READY_FOR_FORMAL_EXPERIMENT = NO`.
