# P10 Corridor01 frozen Control run

## Result

`CONTROL_REPLAY_COMPLETED_TRACKING_RISK_CONFIRMED`

The frozen 40-second prospective slice completed one nominal P2B-style ROS
NDT + EKF replay: 5 seconds of startup followed by the fixed 35-second
evaluation window. This is a new prospective runtime protocol, not a byte- or
field-level recreation of the unavailable historical P2B v1. No Weak-only or
Coupled method was run.

The Codex execution sandbox could not create a local TCP socket
(`PermissionError: EPERM`). The user ran the frozen command in a regular Ubuntu
terminal. Before replay, the actual package source, configuration, map,
extrinsic, input slice, and already-built NDT/EKF binaries were checked against
the frozen hashes in the prior protocol. This archive adds only an offline
posthoc evaluator and compact run evidence; it does not alter the runtime
algorithm, map, initialization, or input.

## Runtime chain and data

- Input: the existing `P10_CORRIDOR01_PROSPECTIVE_CAUSAL_BASELINE_V1` slice;
  396 scans (346 in the evaluation segment), 7,989 IMU messages including the
  real leading boundary sample, and 11,473,279 timed raw points.
- The bag spans 39.962544 s. Scan-start timestamps and raw point offsets are
  preserved. The evaluation origin is 5 s after the first NDT scan timestamp.
- The normalized Corridor01 map and official `T_imu_lidar` calibration are
  unchanged. The sensor-only first-scan map anchor is not GT. Initial velocity
  and IMU biases are disclosed zero-mean priors, not estimates; the gravity
  convention remains inferred rather than officially closed.
- The existing split ROS chain propagated the real Epson IMU, deskewed each
  cloud, aligned it to the prior map, converted the LiDAR observation to the
  IMU state, and applied it through OOSM replay before subsequent propagation.

## Runtime integrity

- 396/396 output scans were deskew-published with `covered_and_deskewed`;
  input/output point counts matched for every scan.
- Current-scan NDT leakage was zero and no post-scan-end IMU was used in all
  396 deskew records. Maximum state-history gap was 10 ms; deskew processing
  mean/P95/max was 13.605/18.144/22.789 ms.
- 396/396 NDT records reported `has_converged=1`; mean/P95/max iteration count
  was 1.303/2/16. This is optimizer convergence only, not proof of correct
  localization. Mean/P95/max recorded NDT fitness was 2.580/7.274/88.788.
  Source inspection confirms `raw_ndt` is saved before `limitNdtStep`, while
  `R_`/`p_` and `/dog_livo/ndt_odom` use the resulting `used_result`; 28 scans
  were step-limited.
- 396/396 delayed measurements were applied by OOSM. Mean/P95/max measurement
  lag was 108.416/110.686/125.229 ms; maximum timestamp alignment error was
  4.981 ms. The EKF lineage ends with 396 NDT updates and 7,989 IMU samples;
  the bag contains 7,988 high-rate odometry publications because the final
  internal IMU/state event is not itself an additional published sample.
- Reproducible NDT timing samples, full per-frame/end-to-end latency, and peak
  RSS are not available in the committed archive. A transient ROS-log timing
  sample was observed during execution but was not retained as an auditable
  artifact, so no timing statistic is claimed here.

## Posthoc evaluation contract

All runtime outputs were frozen and SHA256-recorded before GT evaluation. The
Corridor01 GT is the official-lineage IMU pose sequence. Published poses are
LiDAR poses, so the frozen calibration is used to recover the IMU pose before
comparison. The exact map-to-GT absolute transform is not officially closed;
therefore these are relative-drift metrics, not absolute map-frame ATE.

The primary alignment is one fixed full-pose SE(3) left transform estimated
from the first 3 s of the 35 s evaluation interval using the corrected
high-rate trajectory. The same transform is applied to the corrected
trajectory and the `/dog_livo/ndt_odom` topic output. This topic publishes the
pose after `limitNdtStep`; it is not the raw PCL optimizer transform. Raw
optimizer poses are separately recorded in `ndt_diagnostics.csv` as
`raw_ndt_*` and are not evaluated in the table below. The first 3 s are
reported as the fit prefix; the primary holdout metric is the remaining 3–35
s. A position-only Kabsch alignment is diagnostic only: its position
cross-covariance singular values are `[1026.218, 7.917, 0.001677]`, and its
fitted rotation differs by 5.034 degrees from the full-pose fit, consistent
with strong corridor conditioning sensitivity.

## Results

| Stream / interval | Samples | Translation RMSE / P95 / max (m) | Rotation RMSE / P95 / max (deg) |
|---|---:|---:|---:|
| Corrected high-rate, fit 0–3 s | 600 | 0.084 / 0.197 / 0.217 | 1.333 / 2.437 / 4.198 |
| Corrected high-rate, evaluation 0–10 s | 1,999 | 0.462 / 0.778 / 1.118 | 1.995 / 2.925 / 5.150 |
| Corrected high-rate, holdout 3–10 s | 1,399 | 0.550 / 0.811 / 1.118 | 2.219 / 3.006 / 5.150 |
| Corrected high-rate, primary 3–35 s | 6,388 | 1.885 / 5.004 / 5.563 | 3.033 / 3.693 / 16.375 |
| Corrected high-rate, all 0–35 s | 6,988 | 1.802 / 4.972 / 5.563 | 2.927 / 3.650 / 16.375 |
| Step-limited NDT topic pose, primary 3–35 s | 316 | 1.863 / 4.905 / 5.226 | 3.030 / 3.537 / 15.629 |
| Step-limited NDT topic pose, all 0–35 s | 346 | 1.780 / 4.884 / 5.226 | 2.918 / 3.494 / 15.629 |

Using the predeclared 5-second persistence rule on corrected high-rate
translation error, the first sustained `>0.5 m` crossing is +6.410 s from the
evaluation origin (about +11.410 s from the input-slice start). Sustained
`>1 m` and `>2 m` crossings are +26.738 s and +29.778 s from the evaluation
origin. The different historical P2B time origin/protocol prevents claiming
an exact reproduction of its onset values.

## Interpretation and limits

The real runtime and feedback chain executed end-to-end, but the long-corridor
tracking risk is confirmed: the corrected trajectory reaches sustained
sub-meter-threshold drift early and exceeds 5 m P95 error in the primary
holdout. All PCL NDT calls converged, illustrating why convergence alone is
not a localization-success metric. The result does not establish an advantage
or disadvantage for Coupled NDT because no alternative method was run. The
NDT-topic errors above are for the step-limited published pose, not the raw
PCL optimizer pose.

The per-sample posthoc errors and frozen runtime CSVs are included for review.
The 7.7 MB output bag and 3.5 MB full EKF lineage remain at the local persistent
experiment path recorded in `artifact_hashes.json`; they are not committed.
