# P3-R8 online-observable audit

| Candidate signal | Available from current online pipeline? | GT-independent? | Evidence / limitation |
|---|---|---|---|
| PCL getFitnessScore | Yes; logged by frozen baseline | Yes | Lower is better for the PCL nearest-neighbor score, but it is not calibrated covariance or the exact NDT optimized likelihood. Sequence/frame objective preferences are mixed.
| NDT iterations / convergence | Yes; logged | Yes | All selected counterfactual outputs converged, including far-from-reference Floor endpoints; convergence alone is insufficient.
| Initial-to-raw NDT pose displacement | Derivable online from initial guess and raw pose; present in diagnostic records | Yes | Potential observable, but large/small displacement has not been shown to be a cross-sequence reliability threshold.
| Translation/rotation limiter flags | Yes; logged in full baseline records | Yes | Activity is directly observable. No causal ablation establishes the limiter as the common dominant cause.
| IMU motion and prior-pose velocity | Available in adapter/runtime diagnostics | Yes | Floor translation CV is derived from prior NDT poses; Corridor has similar causal prior-NDT translation deskew. This signal is potentially coupled to estimator history, not an independent truth source.
| Source-cloud / map geometry statistics | Can be computed from current source and fixed map | Yes | No validated shared threshold or mechanism evidence was established in R8.
| Transformation probability / objective samples | Offline fixed-pose probe only for the cited comparisons; not consistently exposed/recorded by the frozen runtime (Floor R7H says unavailable) | Yes | Do not assume this is currently available to a deployed online policy without a separate instrumentation stage.
| Initial-guess reference error | No | No | Requires GT/reference; diagnostic only.
| Oracle init / oracle endpoint distance / oracle retention | No | No | Uses GT to construct or evaluate counterfactual; strictly offline only.
| Perfect-increment error / predictor-vs-GT increment bias | No | No | Requires GT and is not a valid online input.

**P4 gate:** No candidate mechanism is presently supported in the required shared, online-observable, GT-independent form. `P4_ALLOWED = NO`. The next step should be a targeted discriminator, not an algorithm change.
