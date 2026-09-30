# Covariance equivalence and scaling sanity

`sparse_marginal_covariance_test` compares independent sparse production and dense test-oracle P15 and P_map6 on: initial prior with nonzero gradient; IMU-only prediction; directional LiDAR; joint IMU + LiDAR + visual cross-state graph; before Schur; and nine consecutive Schur marginalizations (12 appended states, maximum 3 retained nodes). Existing A1-R1 information conservation and A2C covariance adapter tests are preserved.

Additional failure cases include: unconstrained state; positive-diagonal rank-deficient Hessian; tiny balanced Cholesky pivot; and reuse of previously valid covariance output for an unavailable request. No regularization is used to pass these cases.

Required P15/P_map6 relative error ≤1e-9 and normalized backward error ≤1e-10. Observed fixture maxima: P15 8.88821e-15, P_map6 3.3327e-15, backward error 4.55665e-17. The scaling fixtures' maximum P15 error was 6.84464e-14. Exact output is archived in `COVARIANCE_EQUIVALENCE_RESULTS.txt`.

| Nodes | Sparse marginal ms | Dense oracle ms | P15 relative error |
|---:|---:|---:|---:|
| 8 | 0.526189 | 1.17614 | 1.20433e-14 |
| 16 | 1.22447 | 6.50263 | 2.22265e-14 |
| 24 | 1.92114 | 18.1731 | 3.26834e-14 |
| 32 | 2.63067 | 40.5213 | 6.22435e-14 |
| 48 | 4.16642 | 117.091 | 6.84464e-14 |

The measured operation includes factor assembly, balancing, factorization, 15-RHS solve and fixed 15D/6D PSD checks. A2D's frozen 48-node dense result was about 113 ms; this run's same-graph dense oracle was 117.091 ms versus sparse 4.16642 ms. The latter is approximately 28× lower wall time in this single synthetic observation.

These are complexity sanity observations on the development machine, under ordinary concurrent build/test load—not a controlled CPU benchmark, runtime WCET, or formal paper performance claim. No real trajectory was evaluated.
