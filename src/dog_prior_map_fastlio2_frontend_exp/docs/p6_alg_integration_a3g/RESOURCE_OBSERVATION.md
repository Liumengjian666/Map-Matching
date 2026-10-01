# ENGINEERING RESOURCE OBSERVATION — failed prefix only

Wall/user/system: 47.98 / 41.76 / 5.99 s. Max RSS 74448 KiB (72.70 MiB).
No CPU-affinity or runtime optimization. Legacy covariance and marginalization
shadows OFF; lightweight health and last-optimizer trace ON.

| Timing, ms | N | Mean | P50 | P95 | Max |
|---|---:|---:|---:|---:|---:|
| Completed LiDAR terminal event | 314 | 132.550 | 70.638 | 371.835 | 470.080 |
| NDT total per terminal, including probes | 314 | 50.408 | 1.161 | 262.179 | 362.011 |
| Optimizer estimated excluding QR compute | 630 | 13.487 | 6.941 | 29.032 | 32.268 |
| QR removal compute | 592 | 4.697 | 4.708 | 5.251 | 6.533 |
| QR covariance, including failed request | 315 | 38.114 | 39.248 | 44.583 | 48.370 |

Optimizer estimate = elapsed optimizeAndMarginalize call minus sum of its QR
removal compute times. It includes commit/lifecycle/diagnostic overhead and is
not a pure optimizer microbenchmark. Event/runtime legacy fields have limited
output precision; no precision beyond the recorded CSV is asserted.

Covariance stack maximum: 729×615; maximum temporary estimate 13767072 bytes.
Completed-event prior max rows 15, max stored estimate 72120 bytes. This
completed-event counter does not include transient pre-enforcement expansion.
All QR removal ranks 15.

These data cover only the prefix before tx366. They cannot be called full-course
latency, real-time capability, paper performance, or localization accuracy.
