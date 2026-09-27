# P6-I3 Scope: Dual Registration Reliability

## Innovation hierarchy

1. Overall innovation candidate: **DUAL REGISTRATION RELIABILITY** for prior-map registration.
2. Its two conceptually distinct components are:
   - `U_obs`: local observability reliability—whether the currently converged registration mode is geometrically constrained, and along which physical directions.
   - `U_nonlocal`: nonlocal/mode-basin reliability—whether the local mode depends on initialization or competes with a better basin.
3. This phase tests only whether DCReg-style Schur-decoupled physical observability can serve as a candidate `U_obs` estimator for PCL NDT.

## Frozen status entering this phase

- P6-I2's first `U_nonlocal` estimator (objective uplift + mode separation + covariance-guided probe) **FAILED** as a reliability estimator: basin escapes were better in 474 cases and worse in 439, a 0.51917 better ratio.
- P6-I2 recovery experiments still show that nonlocal/capture-basin failures matter; multi-start is evidence of the problem, not a reliability estimator or novelty claim.
- P6-I1's single-start baseline had 21.9361 m translation RMSE, while objective multi-start was about 1 m. This is frozen context, not a result of P6-I3.

## Boundaries

- `U_obs` is the sole subject; `U_nonlocal` remains **OPEN** and its first estimator remains **FAILED**.
- A positive NDT-Schur result cannot be described as completion of Dual Registration Reliability.
- Visual input is off. No DCREG pose replacement, multi-start, COV3, GEO7, EKF change, mitigation, threshold, runtime trigger, or ROS runtime modification is part of this phase.
- Floor01 has no ground-truth degeneracy labels. GT may only add a descriptive absolute pose error after observability analysis.
- Novelty status remains **NOVELTY_UNVERIFIED**.
