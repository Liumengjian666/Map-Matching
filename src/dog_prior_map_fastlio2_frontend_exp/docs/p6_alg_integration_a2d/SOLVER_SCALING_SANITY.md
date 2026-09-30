# Synthetic solver scaling sanity (not a paper benchmark)

One Release run, same mathematical graph and four-iteration budget, gravity
fixed, noisy LiDAR observations, binary IMU and directional visual factors.
Both optimizers used three iterations at every tested size. Values are ms.

| Nodes | dense assembly | block assembly | dense solve | sparse solve | latest marginal |
|---:|---:|---:|---:|---:|---:|
| 8 | 0.884268 | 0.165176 | 0.153303 | 0.251291 | 1.05929 |
| 16 | 5.08701 | 0.364086 | 0.922472 | 0.572006 | 6.51709 |
| 24 | 13.553 | 0.608952 | 2.05049 | 0.840198 | 15.9135 |
| 32 | 29.5306 | 0.918212 | 5.13345 | 1.14562 | 36.8045 |
| 48 | 91.274 | 1.51257 | 17.7264 | 2.20351 | 113.051 |

Small sparse systems can be slower due to conversion/factorization overhead.
The maximum final-state difference in this run is 3.86139e-16 in the 15D local
coordinate norm. Complete five-row data are in `solver_scaling.csv`.

## Linear-system storage accounting

At 48 nodes: dense H numeric payload 4,147,200 bytes; stored upper blocks plus
gradient 176,760 bytes; compressed sparse H 172,468 bytes. The CSV additionally
records `accounted_sparse_peak_upper_bytes=7,277,068`: a conservative account
including block payload, compressed H, conversion triplets, vectors and a
worst-case dense-fill LDLT numeric/index allowance. This is **not measured
malloc peak or process RSS**; allocator/node overhead and the stored marginal
prior are not included. It must not be presented as an actual RAM benchmark.

Fixed-chart priors may be dense; latest marginal covariance still uses its
own dense assembly and solve. The independent dense reference's prior-chart
matrix products are intentionally retained as a reference. These explain
remaining cost and prevent a claim that the complete estimator is now linear
complexity or proven low-power on real data.

No repetitions/statistical benchmark protocol, CPU pinning, runtime WCET or
real dataset measurement was performed. Offline inputs/catalog/event lists
still scale with dataset length, even though the active Window is bounded.
This is only the authorized synthetic complexity sanity check.
