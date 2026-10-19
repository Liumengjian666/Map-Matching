# P61-R8 — full return CONTROL baseline

## Result

**FULL_RETURN_CONTROL_COMPLETED_WITH_FAILURES** (engineering classification).

Exactly one formal run processed all **2441/2441** admitted scans, 2986–5426.
Deskew, one normal NDT align, reported convergence, and OOSM APPLIED each occurred
2441 times. No time/numerical abort, manual restart, reset, GT input, extra align,
Coupled jet/value or candidate feedback occurred. All 2441 source hashes differ.
Every APPLIED row has a unique same-frame evidence chain. The next frame's
production callback has the preceding NDT update count, 0..2440.

This is **not a stable/accuracy pass**. Significant geometric mismatch occurred
while the optimizer and fusion still reported success. WITH_FAILURES here denotes
observed matching-quality failure/risk, **not** hasConverged=false or a GT-certified
wrong pose/basin. No independent localization error or RMSE was calculated.

## Identity and isolation

- START: `aa8eadbff8c4238095105e2219669ff3eccbe12f`.
- Actual R7 C++ algorithm CODE: `17922f2b63dae1f2af34f353287728d50a312f73`.
- R8 input/replay/recorder CODE: `1367e1d9587d1f90a47bf6f44405239a7b65e4f6`.
- Branch: `research/p61-r8-geode-full-return-control-baseline`.
- Workspace: `/home/jian/livox_ws/p61_r8_geode_full_return_ws`.
- Full raw inputs/output: `/home/jian/livox_ws/p61_r8_geode_runs/`.
- No C++ file changed. The R7 existing executables were reused byte-for-byte,
  verified both on disk and via the running `/proc/PID/exe`. Fresh processes,
  private ROS Master 11399, new output directory; not R7's final state.
- R7 archive differs from R7 actual CODE only in launch/recording/docs, not C++.
  R8 intentionally uses the archived latched-map launcher, not a claim that it
  participated in R7. All new analysis/archive scripts are post-run tools.
- Final archive also contains a post-run recorder evidence-wait guard identified
  by CLI review. It **did not run** in this experiment; actual recorder source is
  frozen at1367e1d above. No C++ math or recorded output changed. See CLI_REVIEW.md.
- No push executed; remote state must be checked independently at delivery.

## Frozen input

Full test contract: 1706671587829840123–1706671831999708652 ns.
Actual first header: **1706671587900364160** ns.
Actual final full-point end: **1706671831999706132** ns.
Actual scan span: **244.099341972 s**. Boundaries2985/5427 were excluded whole;
no point cropping, offset shifts, GT or artificial timestamp repairs.

The original bag contains Livox `/livox/lidar` CustomMsg and external Xsens
`/imu/data`. Only these two sources were consumed. The new bag adapts real points
to PointCloud2, retaining exact uint32 offset_time ns and original header ns;
adapter byte SHA and original serialized SHA are recorded for every scan.
The raw `/livox/imu` is never used.

Input bag includes **24419** real Xsens messages; replay delivered **24418** through
the last needed end bracket. The extra retained tail message was not delivered.
24418 real high-rate publications, 2441 scan-end predictions, 2441 NDT-used poses,
and 2441 corrected poses were recorded. Input bag is1.4GiB; no external disk access.
About55GiB internal free space remained, above the20GiB floor.

SHA256 (all frozen assets also rechecked after run):

| Asset | SHA256 |
|---|---|
| Original bag | `162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80ddf59d0b22091` |
| R5 map,321053 points | `4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4` |
| R6 initialization | `9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d` |
| R6 original23D covariance | `55ac110fd02620c494e916fdb575bd514b734245b26d09764fc7de1116670f39` |
| R2 scan ledger | `21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb` |
| R8 complete input bag | `a138b5ceb0e8ee4a2e5203e70b3943596811ff15d7a45495909934d3a1494331` |
| R8 input manifest | `4444883c441f0c3c08650bf629c05812d34b1a069c7102d565053a47b976c76d` |
| R7 numerical config | `9d2e32ebf5bd89230d46b7af7b5d0cf9115cbe72d600634899337eab09eba0d3` |
| Fullprecision initialization YAML | `084a1f4b04265c82b8fcf9e519bc35708692e5b409eb9c0fcb551955310ac6fa` |
| Actual NDT binary | `89fdc9da728660df9958276f7fc61c216842a866c885a316f8ac0537b329c06b` |
| Actual EKF binary | `40942380de766d75022c5f09475680f6288a0124364160b7241f52ac375bd0f9` |

## Configuration and timing

NDT0.8m/.08step/.001epsilon/40iterations/.55outlier; source0.25m voxel,
0.5–80m,1400points; target0.15m twice. Map321053→target318377 points.
Actual `target_cells_` leaf is `(0.800000012,0.800000012,0.800000012)` m,
the float representation of0.8m, not merely getResolution().
No parameter change: fixed step limiter0.5m/5deg, velocity blend0.6, original
IMU propagation/OOSM fix and fixed15D heuristic restart prior. The R6 23D posterior
was **not** imported. R6 position, orientation, velocity, biases, gravity and
extrinsic retain full precision. See inherited R7 state/time contract and configs.

Deskew and prediction reference true scan end. The last IMU at/before end is used
for the end fragment; a later arrived IMU may be replayed only after correction.
2441 records show `post_scan_end_imu_used=0`. Every correction's native state time
equals its NDT measurement time exactly; maximum native-double epoch conversion
error against exact input ns is119.185425ns. This representation error is not a
millisecond-early update. Every OOSM replays one actual tail sample; all2441 return
to their original current watermark. No rejected/OOSM-incomplete/timeout terminal.

All20 inherited R7 scans have exactly equal source hashes, NDT initial/raw/used
pose cells, iterations and convergence. Fitness differences are zero. No second
R8 smoke or nominal replay was run to obtain parity.

The external fusion is the existing fixed-ratio pose correction + velocity blend
and IMU rollback/replay, **not** a newly claimed standard Kalman measurement update.
`EKF=disabled` in the NDT-node startup refers to its optional internal initial-guess
feature, not absence of the separate actual EKF/OOSM subscriber and correction.

## Observed trajectory and anomalies

Corrected path length over2440 adjacent intervals: **912.572663m**.
Initial L position `(328.540732545,71.786484104,3.036005748)` m.
Final corrected L position `(354.413726696,78.029505193,3.039793721)` m;
quaternion xyzw `(-0.006227675363,-0.030793962409,0.995619295561,-0.088063421724)`.
Max adjacent corrected translation **0.516287m**, rotation **5.097789deg**.
These are actual trajectory increments, not absolute errors. Step limiting is
relative to the preceding NDT used pose, not the current prediction. Physical
motion and replay therefore must not be mistaken for raw optimizer correction.
Maximum prediction→actually-used NDT difference is0.974275m (scan5172) and
12.164822deg (scan5164). These differ from the raw optimizer correction below;
the limiter is not a0.5m/5deg bound relative to the current IMU prediction.

| Diagnostic | Actual evidence |
|---|---|
| Max raw prediction→NDT correction | scan5172:2.915002m /41.043233deg |
| Translation/rotation limiter |186 /32 frames |
| Iteration limit reached |16 frames (still native converged=true) |
| Worst Fitness | scan5171:393.557751,40 iterations |
| Worst frame used-pose NN median |9.156142m at5171 |
| Worst frame used0.5m correspondence |4.071429% at5171 |
| More than half used points beyond8m |5170–5174,5 frames |
| Under half used points within0.5m |5039–5042 and5159–5229,75 frames |

Severe5170–5174 interval: **1706671806300144910–1706671806799748070 ns**.
Broader5159–5229 low0.5m correspondence interval:
**1706671805199903249–1706671812299679066 ns**, about7.10s.
At5171/5172 both NDT and OOSM still report successful processing, with limiter
active. This is direct evidence that convergence/acceptance alone did not certify
geometric agreement. It is **not** proof of a distinct wrong basin or true error.

Important: the geometry CSV's2441 `PASS` strings are written by the existing
diagnostic path as literal status labels after source admission. They do **not**
constitute2441 independently checked adequate-map-coverage decisions; see
`dog_prior_map_ndt_node.cpp` geometry CSV writer. We do not use them as such.

The8m/0.5m flags above are descriptive summaries of already recorded diagnostic
distances, not new online gates or a revised scientific degeneration threshold.
No data was filtered based on these flags. Full tables and all raw failures remain.
There was no software lock-loss terminal; there **was** significant matching-quality
risk. Actual physical lock correctness and an independent failure cause remain
uncertified without a legal independent reference and coverage analysis. No jets
or Hessian were added to this frozen runtime; no Hessian-based failure claim.

## Resource and visual evidence

| Timing ms | Mean | P95 | Max |
|---|---:|---:|---:|
| Deskew |11.755634|16.168517|19.346163|
| NDT align |11.533955|18.519168|85.901504|
| NDT callback |19.637428|27.195747|108.312533|
| Fusion update + replay |0.278128|0.364295|0.711269|
| Sum deskew + NDT callback + fusion |31.671190|40.166267|123.435157|
| Publisher cloud→ACK |39.609873|48.967165|154.976317|

The inherited NDT `localization_time_ms` timer ends before pose publication and
the final geometry diagnostic; its "callback" label is not whole-callback wall
duration. The sum row adds separately instrumented stages, not a measured
end-to-end duration. Cloud→ACK is the actual controller-observed end-to-end wall
latency including required recorded evidence; ROS processes may overlap.

2441 same-frame matched timing samples. Logged cold map/preprocessing/Gaussian
build87.955206ms. Replay215.908068s includes input reading, real IMU publication,
readiness per frame and passive recording. Sensor span244.099342s. ACK-backpressure
throughput is not sustained10Hz validation; maxima exceed100ms.

Separate process HWM MiB / sampled CPU seconds:
NDT106.390625 /54.45; EKF19.027344 /38.71; publisher-recorder327.507813 /71.56;
RViz220.714844 /31.75; map visualizer41.433594 /0.21.
Each has18041 samples (~10ms target; actual sampler includes scheduling overhead).
HWM includes its startup life; CPU deltas cover sampled run, not full startup.
Do not add HWM as unique physical memory. The research publisher/recorder is not
a100MiB deployment claim; GUI/map visualization costs are separate.

Real screenshots at77,1004,1882 and2441 completed frames were taken from the actual
RViz PID. No static replacement, repeated inputs or restarted estimator for GUI.
`rviz_live_*.json` binds wall time and original scan progress. Actual paths contain
all2441 outputs, not merely production's bounded high-rate display history.

## Limits and next

All A–E input, execution, time and preservation evidence is present; **position
accuracy, physical lock correctness, native tunnel degeneracy and independent
cross-pass generalization are not established**. This remains a same-recording
time-isolated nonGT-map experiment, not independent cross-traversal certification.
GT was not read at any stage. No ATE/RMSE, correct-basin label or covariance
calibration is claimed. Local review and optional CLI review have different
scope/timing; see reviews.md and the final CLI receipt if approved.

Unique NEXT recommendation only: ask research control to authorize a bounded,
read-only analysis of the frozen5159–5229 anomaly's map coverage, time/coordinate
and matching consistency. Do not rerun the full sequence or enable new algorithms.
All R8 ROS processes are normally stopped; no next stage automatically started.
