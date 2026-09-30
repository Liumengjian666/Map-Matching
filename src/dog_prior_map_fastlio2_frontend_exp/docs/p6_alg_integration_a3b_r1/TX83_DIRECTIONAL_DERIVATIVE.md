# TX83 directional derivative check

At the failed pre-optimization state, production H/g were assembled once. For each direction the audit compared a centered FD of the production objective and the frozen-LiDAR-basis objective to `2 gᵀd`, over `ε={1e-8,1e-7,1e-6,1e-5,1e-4}`. All 10 rows are in `RUN_P3_100/trajectory.csv.r1_directional_derivative.csv`.

| Direction | `2gᵀd` | FD at ε=1e-5, production | relative error | FD at ε=1e-5, frozen B | relative error |
|---|---:|---:|---:|---:|---:|
| `-g/||g||` | -0.8670066582 | -0.8667422350 | 3.04984e-4 | -0.8670066581 | 5.15207e-11 |
| first production LM step, normalized | -0.1245512539 | +0.0118546580 | 1.09518 | -0.1245512538 | 1.19466e-9 |

For the first-LM direction the production FD has the opposite sign to the local model, while freezing the bases recovers the model derivative to about 1e-9 relative error. The normalized negative-gradient direction shows a smaller but repeatable production mismatch and a near-exact frozen-B match. This is direct evidence of local-model/objective inconsistency associated with relinearizing state-dependent bases; this round deliberately does not implement a `dB/dx` correction.
