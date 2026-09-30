# Single state owner after initialization

`initializeFixedLagProducer` locally constructs FastLio2IkfomFrontend. Its only
state operations are static initialization, optional bounded held-input
prediction to the requested initialization epoch, and read-only initialization
seed extraction. It returns by value and destroys the frontend object.

`runFixedLagProducer` subsequently constructs FixedLagEventAdapter with the
seed and real noise. No IKFoM object exists in its event processing code.
Post-handoff counts for `predict*`, `applyPoseMeasurement*`, `applyProjected*`
and `applyProjectedPositionMeasurement*` are structurally zero.

NDT seeds originate only from `adapter.prepareStateAt`. Covariance originates
only from `adapter.latestMarginalCovariance`. Prior originates only from the
one-time conditional initialization and existing Schur history. There is no
`setWindowPredictionSeed`, read of EKF P, or window/EKF feedback loop.

The read-only Python source audit rejects any post-handoff frontend construction,
prediction/update/feedback or EKF covariance dependency. It also asserts the
actual window prediction/covariance/NIS connections remain present. The real
PCL fixture additionally checks every diagnostic row ends with zero handoff
calls. This is structural ownership evidence, not an instrumentation counter
inserted into a live EKF object.

## Old FULL preservation

Remove only the additive new header imports, experimental main dispatch and
usage line; the remaining entire runner source must equal START_SHA byte for
byte. This is stronger than checking only `runDualReliabilityMode`'s body.

Audit result: PASS.

Original runner SHA256:
`498ec598db3aaec84b74391292d9db7928b2337944be01287621502135abed34`

The shared adapter is intentionally extended for A2C causality/admission
contracts, but formal FULL does not become an adapter-based producer. Existing
runtime updates and all old CTest regressions still pass. No new experimental
mode is silently selected as a default.
