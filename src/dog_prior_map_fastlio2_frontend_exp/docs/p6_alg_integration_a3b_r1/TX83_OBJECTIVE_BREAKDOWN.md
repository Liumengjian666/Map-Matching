# TX83 objective breakdown

`ObjectiveBreakdown` evaluates the exact production factor residual/covariance/basis path. Its total is the same `WindowLinearSystem::cost`; the synthetic regression verifies the four component sum against total.

| Component | At optimization start | First candidate | Change |
|---|---:|---:|---:|
| Prior | 3.35450e-8 | 2.04496e-6 | +2.01141e-6 |
| IMU | 0.0469251084890 | 0.0469256958760 | +5.87387e-7 |
| LiDAR (20 active factors) | 5.432657298377 | 5.432654713645 | -2.58473e-6 |
| Visual | 0 | 0 | 0 |
| **Total** | **5.479582440411** | **5.479582454478** | **+1.40669e-8** |

The latest active LiDAR factor cost is `0.0463641751163` (stamp `1517157227358132435`, observation id 93). The maximum individual LiDAR factor cost is `0.6842744129781` (stamp `1517157225542773319`, observation id 39). The exact per-iteration current/candidate breakdown is retained in the optimizer trace; the scalar exact values are in the failure summary.

Although LiDAR component cost decreases, prior and IMU increase slightly more, so the actual full objective rises. Frozen-basis comparison changes that outcome: the same first candidate decreases the frozen-B objective by `1.44852396744e-7`. Thus component accounting alone does not explain away the local-model mismatch; the paired objective/FD evidence in `TX83_BASIS_RELINEARIZATION_TEST.md` is key.
