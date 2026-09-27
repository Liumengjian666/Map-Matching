# P6-I3 Synthetic Gate (pre-Floor01)

Expected axes SHA-256: `e4310edc4b9e0b597ca15fbf37346c8883fe4810e4ecb5b1eb5a89172403356f` (frozen before analysis).
Perturbations SHA-256: `8d1ce425d3317bba909e38ba65a671d5764b7fc358b72a6a008da99c19856f83`.

| Hard gate | Result | Measured |
|---|---:|---:|
| Corridor +x median Schur alignment >= 0.90 and >=90% perturbations identified | PASS | See `perturbation_robustness.csv` |
| Tunnel +x median Schur alignment >= 0.90 and >=90% perturbations identified | PASS | See `perturbation_robustness.csv` |
| Tunnel roll weak subspace median Schur agreement >= 0.90 and >=90% identified | PASS | See `perturbation_robustness.csv` |
| Plane XY weak translation subspace Schur agreement >= 0.90 and >=90% identified | PASS | See `perturbation_robustness.csv` |
| Plane yaw weak rotation subspace Schur agreement >= 0.90 and >=90% identified | PASS | See `perturbation_robustness.csv` |
| Finite estimator output >=99% | PASS | 100.0000% |
| Analyzer mean <=2 ms/frame | FAIL | mean 10.178 ms; P95 15.148 ms; max 17.119 ms |

Overall synthetic hard gate: **FAIL**.
Incremental-value result: **SCHUR_NOT_DISTINCT**.
A distinctness case is recorded only if Schur meets both task-defined 0.90 median and 90% robustness criteria while both RAW6 and BLOCK miss at least one of those same criteria. No additional numeric threshold was added.

This gate is completed before the Floor01 subset run. A failing synthetic gate is preserved as a failure; expected axes and perturbations are not edited.
