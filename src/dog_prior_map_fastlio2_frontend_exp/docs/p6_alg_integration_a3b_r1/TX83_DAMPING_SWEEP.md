# TX83 diagnostic-only damping sweep

The 13 values `1e-6` through `1e6` used the same failed-state H/g, production linear-system backend/diagonal scaling and maximum-step clipping. Candidate states were local copies. No sweep candidate was applied to the failed Window. Full rows are in `RUN_P3_100/trajectory.csv.r1_damping_sweep.csv`.

| λ | Applied step norm | Production actual reduction | Frozen-B actual reduction |
|---:|---:|---:|---:|
| 1e-6 | 1.16505e-6 | -1.40669e-8 | +1.44852e-7 |
| 1e-5 | 1.36876e-7 | -1.89476e-9 | +1.46366e-8 |
| 1e-4 | 1.77110e-8 | -7.68150e-11 | +1.48751e-9 |
| 1e-3 | 1.96811e-9 | -1.62181e-12 | +1.51593e-10 |
| 1e-2 | 1.99607e-10 | +7.10543e-15 | +1.52864e-11 |
| 1e-1 | 1.99575e-11 | -8.88178e-16 | +1.52856e-12 |
| 1 | 2.00105e-12 | -1.77636e-15 | +1.49214e-13 |
| 10 | 1.98174e-13 | 0 | +1.50990e-14 |
| 100 | 1.98024e-14 | -2.66454e-15 | 0 |
| 1e3 | 1.98022e-15 | -1.77636e-15 | -1.77636e-15 |
| 1e4 | 1.98022e-16 | -8.88178e-16 | -8.88178e-16 |
| 1e5 | 1.98022e-17 | 0 | 0 |
| 1e6 | 1.98022e-18 | 0 | 0 |

Strictly in double precision, λ=`1e-2` happens to yield a positive reduction of `7.1e-15`; relative to a total cost around 5.48 this is roundoff-scale and not a robust acceptance margin. At larger damping the relinearized objective is equal or differs by only a few ULPs, without reliable positive decrease. Thus the sweep does not support “the optimizer merely needed a substantially larger damping to find a numerically meaningful decreasing candidate.” It instead shows frozen-B descent alongside production-objective disagreement.
