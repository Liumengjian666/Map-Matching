# P61-R7: real continuous twenty-frame baseline

**FINAL_RESULT: BASELINE_20_FRAME_READY**

Twenty real test scans passed scan-end IMU deskew, CONTROL NDT and the existing
15D estimator's external-observation/OOSM path. The corrected state genuinely
drives the next scan's prediction/deskew. No GT, Coupled, Recovery, candidate
feedback, mapping or full-return playback. Only **20 actual align calls**.
All task-owned ROS processes were stopped after live RViz acceptance.

## Git and run identity

- Repository: https://github.com/Liumengjian666/Map-Matching
- Branch: research/p61-r7-geode-continuous-baseline
- START_SHA: c649c9fa1743aada3301ddb4f75bce54eaf08532
- CODE_SHA actually run: 17922f2b63dae1f2af34f353287728d50a312f73
- END_SHA: the archival commit containing this report; the complete actual
  final SHA is in the execution receipt and final handoff, not fabricated
  inside its own commit content.
- REMOTE_HEAD for R7: absent when checked; PUSH_EXECUTED=false.
- R6 remote parent independently confirmed as START_SHA. No R6 push repeated.
- Internal final receipt: `/home/jian/livox_ws/p61_r7_geode_runs/final_git_receipt.json`.

Binary SHA256:

| Actual executable | SHA256 |
|---|---|
| NDT | 89fdc9da728660df9958276f7fc61c216842a866c885a316f8ac0537b329c06b |
| EKF | 40942380de766d75022c5f09475680f6288a0124364160b7241f52ac375bd0f9 |

Actual GCC9.4/Noetic/PCL1.10 build used CMake3.16, `-j1`, effective flags
`-O2 -g -Wall -g0 -O3 -DNDEBUG`; research tools default OFF. Frozen binaries
are internal in `p61_r7_geode_runs/frozen/`. No large binary/bag/PCD is in Git.

## GATE A — frozen assets and input

| Asset | Identity |
|---|---|
| R5 map | 321053 points; SHA 4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4 |
| R6 initialization | SHA 9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d |
| R6 original covariance | SHA 55ac110fd02620c494e916fdb575bd514b734245b26d09764fc7de1116670f39 |
| Original bag | SHA 162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091 |
| R2 scan ledger | SHA 21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb |
| Extracted20 bag | SHA cde17297bd8953ce8ec4c4cc0bfadf569df1877c0c2e62a27abfde3f344969d1 |

All three frozen R5/R6 asset hashes were rechecked unchanged **after** running.
Only internal disk was accessed. Original scans2986..3005 were fixed before
matching. Scan2985 did not enter initialization or playback. Raw sensor window:
1706671587900364160..1706671589899545236 ns, **1.999181076s**. Adapter raw XYZ,
reflectivity and uint32 offset ns are retained; original header stamps are
unchanged. Xsens209 measurements available, **208 actually sent** through the
last needed bracket; the extra final sample was not published. No integrated
Livox IMU or camera input. IMU intervals passed the .02s deskew coverage gate.

Initialization at1706671587829840123ns used the complete precision inherited
JSON. PositionL=(328.540732544529,71.7864841037506,3.0360057482115215)m.
The state/covariance distinction is explicit in `state_and_time_contract.md`:
the native23 covariance was **not** imported into15 dimensions. Existing fixed
startup stds .5m/.5m/s/5deg/.05 biases are a heuristic restart prior, not a
posterior uncertainty continuation. p,v,R,biases and gravity come from R6.

## GATE B — real continuous frontend

| Item | Actual result |
|---|---:|
| Raw/deskewed scans | 20/20 |
| Raw points per scan | 24000 |
| Final NDT source points per scan | 1400 |
| Different source hashes | 20 |
| Actual normal align calls | 20 |
| Converged / failed | 20 / 0 |
| Gaussian grid leaf, all axes | float32(.8)=0.800000012m |
| Target after unchanged .15m twice | 318377 points |
| Iterations | 4,2,2, then seventeen1s |
| Fitness minimum / maximum | .0202201447 / .0214218969 |
| Translation/rotation step limits | 0 / 0 |
| Coupled jet/value/extra align/feedback | 0 / 0 / 0 / 0 |

Runtime log proves actual target_cells_ size, not only getResolution(). The
opt-in adapter sets resolution before the global target. Optimizer remains
the existing ObservablePclNdt/PCL CONTROL kernel: .8m,.08step,.001epsilon,
max40,outlier.55,source .25m,range .5..80m. No parameter search.
All20 initial guesses are the actual same-reference scan-end IMU prediction
converted to LiDAR, including the first; no stale t0 pose is relabeled as end.

## GATE C — genuine fusion/time feedback

NDT T_M_L is converted to T_M_I using inverse(T_I_L) by the existing observation
path. **20/20 OOSM APPLIED, zero rejection**. The actual measurement correction
state stamp equals the measurement's native double stamp on all20 frames.
Original integer reference ns remain in receipts; maximum exact native-double
representation difference is **118.024292ns**, not a physical timing error.
No post-scan-end IMU was used to construct deskew poses (20 diagnostics false).

The one permitted time repair is documented in PROTOCOL and reviews: legacy
rollback applied the end measurement at the preceding IMU snapshot. R7 advances
that fragment causally with the real held head using the existing propagation
kernel, corrects at end, and replays only the remaining real IMU interval.
It is default-off and not claimed numerically equivalent to the old unsplit
interval. No filter/optimizer formula was rewritten. Existing NDT observation
fusion is a fixed-ratio pose correction, velocity blend .6 and covariance
damping within the IMU-propagated15D baseline, not native FAST-LIO IKFoM.

Each following scan was sent only after its predecessor's actual measurement
receipt plus newly received corrected pose. Saved TCPROS peer-readiness proves
the real prediction/deskew/NDT/EKF chain, not only publishers' registration.

Final corrected LiDAR pose at **1706671589905755281ns** (current IMU state after
OOSM replay, later than last NDT measurement):

```
p = [337.2430623171154, 74.01876572122809, 3.379309134879685] m
q_xyzw = [-0.06132774488957222, -0.02287543814726658,
           0.9913837173824708, -0.11346341677053262]
```

Corrected20-frame path length over19 intervals: **8.223951751m**.
Including the actual initialization-to-first-corrected segment .804340857m,
the total becomes **9.028292608m**; these are different time ranges.
Max adjacent translation **.460055496m**; max adjacent rotation **1.338060046deg**.
These .1s increments alone are not evidence of a nonphysical jump. No observed
processing interruption or loss of lock in this bounded window. Largest raw
NDT-vs-prediction correction: **.203803847m / 1.115885896deg**, on the first scan.
No absolute accuracy or global basin correctness is asserted.

All20 raw aligned sources had .5m nearest-map-point correspondence **100%**;
max P90 distance .196253408m. First prediction overlap98.5714%. These are
**estimated-pose-conditioned geometry**, not independent true coverage or GT.
No additional Hessian jets were enabled in this runtime; this task supplies
no new severe-degeneracy or wrong-basin certification.

## GATE D — resource and actual live RViz

| Measurement ms | Mean | P95 | Max |
|---|---:|---:|---:|
| Deskew | 12.635596 | 16.385463 | 18.148524 |
| NDT align, all20 | 14.759618 | 19.231643 | 72.054743 |
| NDT align excluding first19 | 11.744085 | 16.314899 | 16.451480 |
| NDT processing callback | 23.879683 | 28.858011 | 82.156276 |
| EKF observation/update + replay | .197538 | .257061 | .345764 |
| Sum of three processing stages | 36.712817 | 42.642193 | 91.751350 |
| Publisher→fusion ACK wall latency | 43.802822 | 52.554130 | 99.150321 |

Sum of callbacks is not a single monolithic callback. ACK wall includes IPC,
diagnostics and polling; it is distinct from algorithm CPU time. Cold map
load/preprocess/Gaussian build92.754072ms; first align72.054743ms.
Actual20-frame replay wall **1.758212899s**, excluding manual GUI setup.

| Process | Sampled peak RSS MiB | VmHWM MiB |
|---|---:|---:|
| NDT | 106.824219 | 106.824219 |
| EKF | 17.710938 | 18.324219 |
| Python bounded publisher | 68.210938 | 68.210938 |
| RViz during replay | 177.363281 | 177.363281 |

10ms /proc sampler retained156 samples/process, actual PIDs and CPU ticks.
Ticks at start→end: NDT22→74,EKF5→37,publisher144→173,RViz116→150;
USER_HZ=100 on this machine. Do not sum process VmHWM as unique physical RAM.
NN KD-tree/diagnostics are enabled; this full-node measurement differs from
R6's raw standalone probe. No headless performance run was added.
**NOT_REALTIME_VALIDATED**: short ACK playback is not a sustained10Hz test.

Actual ROS/RViz ran throughout the real20-frame input. The nonlatched complete
map's one-shot publish preceded RViz subscription; no map message was retained.
After the20-frame freeze, only that visualization publisher was restarted with
latch=true, no new input/align/fusion. The launch now includes latch=true.
Final live scene map topic width321053 was independently received. RViz then
reported VmHWM214.796875MiB; separate map-view publisher41.921875MiB. These are
post-map GUI figures, not substituted into replay-stage performance.
Both actual screenshots are retained, including the initial missing-map phase.
No R6 static screenshot was reused.

## Recording caveat, verification and limits

rospy rewrites transport header.seq on publish. Frozen raw receipt column named
scan_index consequently contains1..20; original timestamps/hashes and the
pre-frozen ledger bind them unambiguously to2986..3005 in `frame_results.csv`.
Raw files were not rewritten. The archival reproducer fixes this log label
only, after the run; CODE_SHA remains the actual original run identity.
The other post-run changes are visualization latch, a safe anonymous default
output directory/existing creation wrapper, active-output-directory discovery
for the recorder, and derived reporting. They were not used to rerun NDT.

Passed existing OOSM planner, deskew, IMU interval, frame/twist C++ tests;
new C++ time-boundary regression; four Python identity/readiness/ACK tests;
build and launch parsing; all20 output/time/feedback assertions; unchanged
asset SHA checks. An authorized CLI second opinion ran exactly once; findings
and local fixes are in reviews.md, with full log internal.

No GT was read. No rotation/translation RMSE exists for this smoke. Same-record
time-isolated map/test is not independent cross-traversal evaluation. No20-frame
success should be promoted to full-return stability, unique basin, uncertainty
calibration, realtime, or algorithmic novelty. No new failure interval is
certified here; preserve these actual logs as the engineering baseline.

**Unique NEXT:** controller review of the scan-end time repair and fixed15D
initialization policy, then decide whether to authorize one frozen full-return
CONTROL baseline. Do not execute it automatically.
