# R2 policies and selected pre-measurement NIS

| Alias | Existing policy string | Noise | NIS |
|---|---|---|---|
| P0 | LEGACY_BASE_NO_GATE | makeBasePoseNoise | OFF |
| P1 | ADAPTIVE_NO_GATE | makePoseMeasurementNoise | OFF |
| P2 | BASE_SELECTED_NIS | makeBasePoseNoise | ON |
| P3 | ADAPTIVE_SELECTED_NIS | makePoseMeasurementNoise | ON |

These names are not redefined. Directional selection/routing remains shared
across policies. Covariance semantic remains EMPIRICAL_POSE_RESIDUAL, in
`[position map, rotation right/body]`, matching window residual:

```
r = [ p_measured - p_state ; Log(R_state^T R_measured) ]
r_s = Qr^T r
R_s = Qr^T R Qr
J_s = actual window linearizeLidarFactor at the predicted state
S_s = J_s P_latest15 J_s^T + R_s
NIS = r_s^T solve(S_s, r_s)
```

The P15 snapshot is captured after adding the predicted state/IMU link and
**before** admitting the current LiDAR measurement. Threshold is the existing
`chiSquare99Threshold(rank)`. LLT/finite checks are mandatory. An unavailable
window covariance does not fall back to EKF; selected NIS cannot accept it.

NIS rejection sets `measurement_commit_allowed=false`. The terminal risk and
transaction are consumed for diagnostics/causality, but no LiDAR factor enters
the joint objective. Visual routing sees `LIDAR_MEASUREMENT_REJECTED` and may
admit a valid FULL_TRANSLATION relative-only factor, not pretend a global
LiDAR correction happened.

Tests exercise a large conflicting selected residual (rank 5), verify rejection
does not add a factor, then admit a real rank-3 relative visual factor with the
explicit rejection trigger. The PCL fixture verifies NIS is finite exactly for
P2/P3, all four policy names, and actual geometric LiDAR admission.

These are empirical engineering noise/chi-square gates; statistical calibration
and target nominal rejection rate are not claimed by this test stage.
