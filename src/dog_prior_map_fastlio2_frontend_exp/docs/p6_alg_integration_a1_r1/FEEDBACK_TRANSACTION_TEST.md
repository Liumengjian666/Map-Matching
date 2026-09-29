# Prediction feedback and IKFoM transaction test

## Fixed-lag revision validity

Every successful state/prior/factor insertion increments `window_revision` and
invalidates `prediction_feedback_ready`. A successful accepted update or legal
zero-step convergence sets `optimized_revision = window_revision`. Feedback is
exported only when both revisions match and the latest state is finite.

The regression verifies:

* feedback succeeds immediately after `CONVERGED_WITHOUT_STEP`;
* adding a new state makes the old feedback fail with
  `window_revision_not_optimized`;
* a failed optimizer cannot export feedback;
* a Schur removal immediately following a validated optimization transfers the
  validated revision because the retained state values do not change.

## IKFoM transaction rollback

`setWindowPredictionSeed` backs up the complete IKFoM state, 23x23 covariance,
and timestamp immediately before `change_x/change_P/stamp`. A test-only
compile-time hook forces failure after all three writes. The test compares the
pre/post `FilterSnapshot` field by field and passes only if pose, velocity,
biases, gravity, extrinsic, covariance, and timestamp are identical.

The hook is compiled only into test executables via
`DOG_PRIOR_MAP_ENABLE_TEST_HOOKS`; formal runtime binaries do not contain it.

## Covariance limitation

The fixed-lag state is 15D and IKFoM's state covariance is 23x23. R1 does not
invent a covariance mapping. The interface remains an experimental boundary
and requires a genuine propagated 23x23 covariance. It does not fill identity
blocks, zero unknown cross-covariances, or reuse an unrelated old EKF
covariance. A full posterior mapping remains future integration work.
