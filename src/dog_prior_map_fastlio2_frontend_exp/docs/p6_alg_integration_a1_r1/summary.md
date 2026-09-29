# PAPER-P6-ALG-INTEGRATION-A1-R1

## Result

The directed A1 repair passes all permitted build and synthetic regression
gates. Fixed-lag marginalization now conserves information without counting
retained raw factors twice; optimizer success, feedback readiness, and IKFoM
feedback application are transactional; and observation-ID memory is bounded
without allowing retired measurements to re-enter.

## Implemented corrections

1. `marginalizeOldest()` uses an explicit marginalization-subgraph assembly:
   existing prior plus only IMU/LiDAR/visual factors touching the removed
   state. Retained-only factors stay active and relinearizable.
2. Production diagnostics expose the complete linearized `H,g` so regression
   tests compare algebraic information, not only final positions.
3. A fixed prior uses the Jacobian of its retained local-coordinate map at
   nonzero rotation. This is documented as local Gauss-Newton consistency, not
   nonlinear global exactness.
4. Solve-only jitter influence is measured against the unjittered
   Moore-Penrose Schur reference.
5. Optimizer outcomes are explicit: `ACCEPTED_UPDATE`,
   `CONVERGED_WITHOUT_STEP`, `FAILED_ALL_CANDIDATES`, and
   `INVALID_LINEAR_SYSTEM`. Iteration count alone cannot produce success.
6. Stacked window increments roll back every state if any node update fails.
7. Window and optimized revision counters prevent export of stale prediction
   feedback after any new state or measurement.
8. `setWindowPredictionSeed()` rolls back IKFoM state, covariance, and stamp as
   one transaction after a postcondition failure.
9. Active observation IDs are removed with retired factors, hard-bounded, and
   protected by a historical watermark under a global monotonic-ID contract.
10. The A1 math audit now matches the actual LiDAR residual and covariance
    chart in `window_lidar_factor.cpp`.

## Evidence

* Three-state complete Schur versus new-prior-plus-retained-factor:
  `H error 1.24609e-10`, `g error 7.85276e-13`.
* Conflicting/noisy retained observation: same equality with an explicitly
  nonzero gradient.
* Repeated add/optimize/marginalize: per-cycle `H,g` equality and correct active
  counts.
* Nonzero rotation prior chart: gradient error `0` at printed precision.
* Jitter effect: `lambda=1e-7`, `H delta=1.71782e-7`,
  `g delta=1.07995e-8`.
* Observation lifecycle: 10,000 cycles, active index bounded, historical replay
  rejected.
* Standalone Release CTest: 6/6 PASS.
* FAST-LIO2/DCReg Release CTest: 8/8 PASS.
* Catkin-style experimental ON CTest: 5/5 PASS.
* Default experimental OFF configure/build: PASS.

## Frozen boundaries and limitation

No frozen baseline or formal FULL default behavior was modified. No Floor01,
Corridor01, rosbag, or GT-based experiment ran. The 15D fixed-lag posterior is
not yet mapped into IKFoM's complete 23x23 posterior; the feedback API requires
a genuine 23x23 covariance and remains an experimental boundary.

`READY_FOR_FORMAL_EXPERIMENT = NO`
