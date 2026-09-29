# A1 source call graph

```
ROS/scan callback
  -> FrontendRuntime::beginScan
       -> FastLio2IkfomFrontend::cloneCandidate
       -> predictImuSequence / predictHeldInputTo
  -> NDT terminal transaction
  -> FrontendRuntime::finishScan
       -> applyPoseMeasurement or checked projected update
       -> commitCandidate (only on successful postconditions)

P6-I1 offline replay
  -> dual reliability decision
       -> assessLocalRisk / Schur U_obs
       -> U_nonlocal terminal-response record
       -> makePoseMeasurementNoise

Experimental A1 path
  -> FixedLagExperimentalController(FULL_FIXED_LAG_EXPERIMENTAL)
       -> FixedLagWindow::addState/add*Factor
       -> optimize
       -> marginalizeIfNeeded
       -> predictionFeedbackSeed
```

`FORMAL_FULL_LEGACY` rejects the experimental controller operations.  No
public ROS callback currently constructs a fixed-lag window.
