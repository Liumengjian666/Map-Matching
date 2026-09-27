# P6-I3 Synthetic Gate (pre-Floor01)

Expected axes SHA-256: `e4310edc4b9e0b597ca15fbf37346c8883fe4810e4ecb5b1eb5a89172403356f` (hash-pinned for this run; this commit alone does not prove preregistration before results).
Perturbations SHA-256: `8d1ce425d3317bba909e38ba65a671d5764b7fc358b72a6a008da99c19856f83` (hash-pinned for this run).

| Hard gate | Result | Measured |
|---|---:|---:|
| Corridor +x median Schur alignment >= 0.90 and >=90% perturbations identified | PASS | See `perturbation_robustness.csv` |
| Tunnel +x median Schur alignment >= 0.90 and >=90% perturbations identified | PASS | See `perturbation_robustness.csv` |
| Tunnel map-frame rotation_x weak subspace median Schur agreement >= 0.90 and >=90% identified | PASS | See `perturbation_robustness.csv` |
| Plane XY weak translation subspace Schur agreement >= 0.90 and >=90% identified | PASS | See `perturbation_robustness.csv` |
| Plane map-frame rotation_z weak rotation subspace Schur agreement >= 0.90 and >=90% identified | PASS | See `perturbation_robustness.csv` |
| Finite estimator output >=99% | PASS | 100.0000% |
| Spatial-Jacobian FD validation <=1e-5 | PASS | max error 6.2e-09; max condition 2.20243 |
| Spatial-coordinate transform validity | PASS | converged synthetic samples rejected: 0 |
| Physical-axis mapping consistency | PASS | checked 844 rows; failures 0 |
| Analyzer mean <=2 ms/frame | FAIL | mean 9.528 ms; P95 14.505 ms; max 15.049 ms |

Overall synthetic hard gate: **FAIL**.
Incremental-value result: **SCHUR_NOT_DISTINCT**.
A distinctness case is recorded only if Schur meets both task-defined 0.90 median and 90% robustness criteria while both RAW6 and BLOCK miss at least one of those same criteria. No additional numeric threshold was added.

This gate is completed before the Floor01 subset run. A failing synthetic gate is preserved as a failure; expected axes and perturbations are not edited.
