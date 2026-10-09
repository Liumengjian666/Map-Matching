# P10 Corridor01 real scan-to-map benchmark

FINAL_RESULT = `CORRIDOR01_BENCHMARK_EXECUTED_COUPLED_ADVANTAGE_NOT_ESTABLISHED`

NEXT = `RESTORE_CORRIDOR01_CAUSAL_NOMINAL_TRACKING_CONTRACT`

## Outcome

Executed real PCL NDT and unchanged R6 Weak-only/Coupled refinement on every
fixed eligible Corridor01 scan, TX52–2777 (2726 scans per arm). There were 8178
full PCL align calls, no extra aligns, one map/backend object, no GT, no raw
extraction and no bootstrap replay. Input hash gate passed 15/15. Exact prepared
source count/hash/map equality held on 2726/2726 scans; all 8178 recorded
executed poses were finite rigid transforms. This is a **standalone causal
scan-to-map diagnostic**, not full IKFoM localization or an accuracy PASS.

The initial 300 scans improve optimization success, but this does not persist
across the full sequence. All three arms eventually lose useful nominal
matching and continue with the explicitly logged gyro/CV prediction fallback.
Coupled does not outperform Weak-only in full-sequence strict success or
longest failure run. Absolute position/rotation errors are NOT_AVAILABLE.

## Actual Git provenance and startup

User-reported remote/start: `ea12a9edd757bc08bebd83f75405c28982704b13`.
Actual writable branch start: `d9b8b2f8c6d64618969f91853f412dd2cda4024a`, an
ancestor-verified descendant preserving the preceding unpushed P10-R7 result.
No history reset or old archive overwrite.
CODE_SHA = `477465475803f8cab6b518887ee5fce55cc09c38`.
Branch: `research/p9-r4-heldout-visual-evidence`.
Writable clone: `/tmp/dog_loc_paper_r4_ws.Fq21k2`.
Final result commit is reported by the delivery receipt/chat rather than a
self-referential SHA inside its own tree. PUSH_EXECUTED = NO. A bounded remote
query failed; the session does not assert current remote state independently.

Protocol: `SENSOR_ONLY_PRIOR_SEEDED_SCAN_TO_MAP_DIAGNOSTIC`.
An archived sensor-only first-five-second initializer estimates the **first
scan pose**, and the existing normalized map applies that transform's inverse.
Identity is therefore a stale historical first-scan prior, not the correct
current TX52 pose. Historical prior pose time: 1517157219.188980000 s.
Conservative availability time: 1517157224.188980000 s. First actual matching
interval: [1517157224.231677055,1517157224.332516266] s, TX52. These times are
not conflated. No official `# s 67` timestamp/direction assumption was made.
Single-GT initialization = NOT_RUN.

TX52 has raw28836 / prepared1400 points, source FNV1202280274059814369,
actual Corridor01 target226164 points. All arms: SUCCESS, 21 iterations,
raw score1236.2917610387328, all-point fitness8.064398925814748 m².
Strict convergence and finite fitness do not certify correct global position.

## Fixed 300-frame and full comparisons

| Metric | Nominal | Weak-only | Coupled |
|---|---:|---:|---:|
| TX52–351 strict success | 291/300 | 299/300 | 300/300 |
| TX52–2777 strict success | 818/2726 | 885/2726 | 863/2726 |
| Full strict success rate | 30.0073% | 32.4652% | 31.6581% |
| Rejected/failure rows | 1908 | 1841 | 1863 |
| Iteration-limit rows | 38 | 20 | 21 |
| Zero-iteration passthrough rows | 1870 | 1821 | 1842 |
| Failure episodes | 29 | 11 | 14 |
| Failure-to-success transitions | 28 | 10 | 13 |
| Longest consecutive failure | 1872 | 1828 | 1845 |
| Last strict success TX | 905 | 949 | 932 |
| Executed local feedback | 0 | 133 | 161 |
| >0.5 m OR >10 deg step diagnostic | 2324 | 1997 | 2058 |

All arms have PCL `hasConverged=true` on 2726 rows, but the original strict
classifier rejects iteration80 and iteration0. Reporting those flags as 100%
localization success would be false. True wrong-match/recovery counts cannot
be obtained from these convergence proxies.

Coupled succeeds on 63 scans where the independent Nominal arm fails, but also
fails on 18 where Nominal succeeds. Histories have diverged; these are **not**
63 certified recoveries from the same nominal initial pose. The complete
`frame_comparison.csv` and all trajectories preserve both sides.

Persistent failure begins at TX906/950/933 respectively; sustained raw-score0,
iteration0 begins at TX908/957/936. At the final scan, the propagated positions
are approximately [242.794,-422.814,-2349.097],
[-1696.919,5632.136,-2250.280], and [-1617.319,-451.327,1732.138] m.
This large map-frame extrapolation is retained, not cropped or reset.
It is a clear nominal-tracking risk, not a substitute for GT RMSE.

## Real contribution of conditional coupling

B has 160 attempts: k1=43, k2=117; C has 173: k1=47, k2=126. Valid attempted
Hessians/curvatures are in `frames.csv`; unmeasured control curvatures are not
interpreted as zero physical curvature. Event invocation counts B209/C233;
their recorded valid Anchor counts are B766/C736. The control arm does not
invoke R6; its 211 over-threshold innovations are separately reported.

Within C, 172 legal weak points receive a paired conditional comparison.
Intermediate strong selection occurs79 times; two final pose-bound rejects
(TX740 and TX805) leave only **77 actual accepted strong feedbacks**.
Mean accepted strong translation0.001545231 m, P950.004603916 m,
max0.019395466 m; rotation mean0.078998355 deg, max0.357071965 deg.
Mean paired raw-score improvement over its same-input weak point is
0.679159529. This confirms nonzero executed coupling and real objective gain,
not absolute accuracy. Remaining feedback uses the legal weak point.

Weak-only extra jets160/value164; Coupled jets345/value318; extra align0.
Ordinary/control rows7736 have zero extra jet/value/align. The original
R6 mathematical kernels, Anchor lifecycle, quality rules and limits were
not changed; no directional covariance module is used.

## Complete processing costs and limits

Each arm's complete cost charges the **full common** streamed input/rotational
deskew cost, plus its prediction/preprocessing/registration/refinement. It is
not just kernel timing and is not divided by three. Startup map load and
benchmark output I/O are separately included in combined experiment wall time.

| Full cost (ms) | Nominal | Weak-only | Coupled |
|---|---:|---:|---:|
| mean | 325.175711 | 37.905455 | 31.901694 |
| P95 | 492.007360 | 83.976061 | 139.223453 |
| max | 535.639299 | 304.546896 | 332.735027 |
| CPU mean | 324.785394 | 37.714543 | 31.720172 |
| PCL align-only mean | 17.624866 | 14.498716 | 16.140400 |

First300 complete mean/P95: A60.027246/249.135911 ms,
B44.889129/137.361649 ms, C49.378081/152.856095 ms.
Shared-process peak RSS52.019531 MiB. Combined three-arm wall1055.646508 s.

Do not interpret the full-sequence lower B/C means as reliable real-time
localization: most later frames are failed zero-iteration matches. A also has
303.593602 ms/frame of unsegmented non-align/non-deskew processing versus
B19.158852 and C11.176725 ms. Actual source includes post-align
`getFitnessScore()` nearest-neighbor diagnostics outside the align timer;
this run did not separately profile it, so the entire residual cost is **not**
attributed to that one function. The full observed costs remain reported.

## Primary limitation and stop decision

The approximate startup plus gyro/CV-only state recurrence is not a restored
full causal inertial baseline. After matching loses map support, the existing
R6 path requires an effective nominal terminal and returns
`INVALID_OR_INEFFECTIVE_NOMINAL`, so neither weak nor coupled refinement runs.
Small bounded post-nominal corrections do not provide reacquisition once the
prediction escapes the map. The exact initiating cause cannot be uniquely
assigned to initialization, unestimated biases, missing translation deskew or
ambiguous local geometry from this one diagnostic.

Available map/GT evidence remains non-unique Level2 consistency. GT_LOADED=NO;
all translation/rotation RMSE/P95/max, true wrong matches and true location
recoveries are NOT_AVAILABLE. No full IKFoM trajectory exists for this run.

Engineering startup/comparison goal is met. Sustained tracking and Coupled
accuracy advantage are **not established**; Coupled is worse than Weak-only
on full strict-success count and longest failure run. Do not tune Anchor,
covariance or admission again from these outputs. End this task; the sole
recommendation is to restore the Corridor01 causal nominal-tracking contract
before making another localization-accuracy claim.
