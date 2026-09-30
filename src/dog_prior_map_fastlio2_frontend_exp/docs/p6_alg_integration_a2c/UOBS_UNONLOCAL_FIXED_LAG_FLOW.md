# Geometric U_obs and window-owned U_nonlocal

The independent producer follows the existing modules:

```
Window prediction -> M0 NDT
  -> geometricObservations -> analyzeGeometricObservability -> assessLocalRisk
Window pre-measurement P15 -> G P15 G^T
  -> mapProductInnovation -> shouldRunNonlocalProbes
  -> analyzePoseCovariance -> +/- principal prior-conditioned seeds
  -> existing runNdtCandidate -> analyzeNonlocalTerminalStability
  -> decideDualReliability / riskWithNonlocalResponse
  -> routed risk + base/adaptive noise -> selected window factor
```

All M0/M+/M- guesses start from predicted map_T_imu and the real fixed extrinsic.
NDT uses resolution 0.8, step 0.08, epsilon 1e-5, max iterations 80. No NDT
implementation, scientific threshold or GT-based gate is replaced.

Probe amplitude is `probe_prior_sigma * sqrt(lambda_max) * v_max` in the
existing map-left pose chart. Both trigger and eigenspectrum consume the window
P_map6. If covariance is unavailable, diagnostics say
`UNONLOCAL_DISABLED_WINDOW_COVARIANCE_UNAVAILABLE`; no old EKF covariance is
available as a substitute.

Algorithm terminal status remains `NOT_PROBED` if no probe was requested.
The no-trigger reason is a separate diagnostic field. Passing a no-trigger
reason into the mature noise model as a failed terminal response is forbidden.
Actual requested probe failures remain explicit invalid-response statuses.

`uobs_valid` reports the geometric analysis validity; selected weak/reliable
dimensions report the final routed risk, which may include nonlocal response.
Support counts are map-support diagnostics, not accuracy measurements.

The PCL fixture computes genuine geometric observations and a +/- probe pair
for each policy. Its periodic transaction 25 exercises nonlocal routing; all
seven event covariances are available. This demonstrates wiring, not a
wrong-basin detector, local-minimum proof or real-data performance conclusion.
