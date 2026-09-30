# TX83 pre-optimizer capsule

来源：`RUN_P3_100/trajectory.csv.r1_preopt_capsule.csv`，tx83 行在 `optimizeCurrentWindow()` 之前 flush；failure summary 和后续 trace 在异常传播前 flush。

| 指标 | tx83 |
|---|---:|
| timestamp | 1517157227458992315 ns |
| Window nodes / span | 41 / 2.017078876 s |
| predicted IMU position (map) | `(4.842426551, -3.132160178, -0.151738088) m` |
| NDT | converged; fitness 33.897558306; fixed objective 602.332796130; 51 iterations; 94.912966 ms |
| U_obs | valid; `VALID_GEOMETRIC_GAUSS_NEWTON_PROXY`; weak dimension 1; reliable dimension 5 |
| translation eigenvalues / weak ratio | `8.289464, 9.219280, 14.695616` / `0.704128` |
| rotation eigenvalues / weak ratio | `63.368972, 527.561193, 756.311572` / `0.091822` |
| pre-measurement P15 | valid; min/max eigenvalue `1.62285e-5 / 5.90331e-2`; all 15 diagonal entries retained in CSV |
| map pose P6 | min/max eigenvalue `2.29670e-4 / 1.70756e-2` |
| U_nonlocal | probe ran; 2 extra NDT calls; `RECORDED_NO_BASIN_CLASSIFICATION`; ± terminal deltas retained in CSV |
| measurement noise | `ADAPTIVE`; R6 eigenvalues `[0.01, 0.01, 0.01, 0.04, 0.04, 0.04]` |
| selected factor | preview valid; rank 5; full B, raw r6, selected rs, and selected Rs retained in CSV |
| selected NIS | valid `45.0319668967`; threshold `15.086`; rejected |
| factor counts before optimize | IMU 40; LiDAR 20; visual 0 |
| current tx83 LiDAR factor | attempted, not committed; event `SKIPPED_INVALID_SOURCE;LIDAR_MEASUREMENT_REJECTED` |

NDT terminal `map_T_lidar`, prediction pose, weak directions/basis, P15 diagonal, R6, residual vectors and matrices are in the sidecar without being rounded to the summary table above. The tx83 NDT measurement was rejected by selected NIS; it is not in the 20-factor window objective that fails. The failure objective contains prior + IMU + previously active LiDAR factors and no visual term.

No GT was read or used.
