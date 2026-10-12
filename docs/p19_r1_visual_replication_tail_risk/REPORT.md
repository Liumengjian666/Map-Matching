# P19-R1: Visual replay replicated; tail stall characterized

TASK: P19-R1-VISUAL-REPLAY-REPLICATION-AND-TAIL-RISK

FINAL_RESULT: **VISUAL_REPLICATED_TAIL_RISK_CHARACTERIZED**

REPLICATION: PASS. TAIL_RISK_ANALYSIS: PASS. One new full ROS replay only.
The 33.88827163291619% aggregate RMSE benefit and 10.233786719056546%
translation-maximum regression both repeat. This is not an all-metric
safety claim, independent generalization, or proof of a new algorithm.

## Git, scope and frozen input

Repository: Liumengjian666/Map-Matching. Branch:
`research/p9-r4-heldout-visual-evidence`. Workspace:
`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`.
START_SHA: `5df7f8c729dd2ff3d53ffe44f25d183963be87fb`.
CODE_SHA: `5707e87a0c6a2e855f47735f256714736afa83b2` (analysis only).
The final archive commit is END_SHA supplied in the chat handoff.
Remote observed before final commit: START_SHA. PUSH_EXECUTED: NO.
No runtime source/configuration or binary was edited/rebuilt. The dirty main
workspace and stable robot workspace were not modified. Earlier archives
and local caches remain intact; no bag or PCD is committed.

New functions are `compare.compare()` (full physical-output parity),
`analyze.analyze()` (fixed-frame peak/window errors), `lineage.inspect()`
(sensor-keyed history/velocity bookkeeping), `runtime.summarize()` and
`archive.archive()`. Ten focused tests pass. Two independent read-only
reviews checked comparison semantics and tail coordinates/times; no external
CLI was invoked. Review fixes concerned analysis only: exact nanosecond IDs,
finite/ordered state checks, QUEUED/APPLIED deadline pairing, corrected-event
lookup by last accepted observation, and explicit cross-method pairing.

| Frozen asset | SHA256 |
|---|---|
| SSD map, 4058751 bytes | 103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f |
| V2 bag | 7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e |
| Raw visual.csv | 10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79 |
| P18 SSD YAML | dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220 |
| NDT executable | ba65e3e516a982c4c0083b866fc8e95c2dede4003fdc0f42cbf1a489da931cfb |
| EKF executable | fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a |

Map remained the P18 SSD recovery copy, not an external-disk read. Actual
ROS physical parameters compare equal; only output locations, run UUID and
roslaunch process URIs differ. CONTROL, camera off, visual on, scan-start,
fixed lag 0.15 s, rate 0.25, no geometry hook. Runtime child paths/hashes
were verified by the unchanged P17/P18 runner before replay.

## Full replication, not metric-only similarity

| Output | P18 | P19 |
|---|---:|---:|
| NDT / reported converged | 396 / 396 | 396 / 396 |
| OOSM APPLIED / corrected publications | 396 / 396 | 396 / 396 |
| High-rate EKF states | 8004 | 8004 |
| Visual applications | 169 | 169 |
| Late / nonfinite | 0 / 0 | 0 / 0 |

All timestamps, source counts/hashes, deskew physical results, Initial/Raw/
Used poses, iteration/convergence flags, visual application/reason, fixed-lag
release/replay and corrected/high-rate states match. Positions and velocity
components are exactly equal in serialized records. The quaternion physical
comparison produces at most 3.2023193490458842e-15 deg self-composition
roundoff, far below the unchanged 1e-8 deg contract. No first discrepancy.
Translation tolerance remains 1e-9 m; timestamps 1e-12 s; fitness relative
tolerance 1e-12. Callback wall clocks and legal pre-deadline arrival snapshots
are not physical sensor-time outputs. QUEUED/APPLIED IDs and deadlines are
validated one-to-one; neither NaN nor late arrivals can pass. Extra strict
raw-delta/history comparisons also match; tolerances were not relaxed.

Pre-GT output freeze was recorded before raw GT was opened. The original
receipt is unchanged. Pre-GT analysis-source snapshots preserve the original
hashes; after-GT analysis-only fixes are explicit. The derived
`evaluation_replay_receipt.json` resolves those identical source snapshots
for reproducibility, without modifying any input/output hash or alignment.
`source_inventory.json` records raw-cache and LF-normalized archive hashes;
three trailing spaces on empty ROS launch parameters are removed only from
the archived log. The raw runtime log is unchanged and retains its hash.
All replay outputs, including outputs.bag, remain in the local cache; only
lightweight CSV/logs and its original hash are committed.

## Fixed evaluation

Original Control V2 0–3 s full-pose SE(3) is reused, not refitted. Evaluation
origin: 1517157224.188979. Primary: 3–35 s. These are **PREFIX-ALIGNED
RELATIVE DRIFT**, not absolute map-frame ATE. NDT LiDAR poses are converted
to IMU origin with frozen official extrinsics; lineage already is IMU.
Raw GT SHA: 3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03.
Extrinsics SHA: 59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d.
P19 and P18 all metric dictionaries, windows and same-history initial pairs
are exactly equal, not merely rounded-close.

| Layer / samples | Control translation RMSE / P95 / max m | P18 = P19 Visual |
|---|---:|---:|
| Initial / 316 | 1.859456724589818 / 4.923619659691138 / 5.253669097416195 | 1.192362408616573 / 2.367812327003957 / 5.7636152102569795 |
| Raw NDT / 316 | 1.8593147190712502 / 4.926219098849796 / 5.218202069359483 | 1.19198954153816 / 2.32691970754862 / 5.784378607182949 |
| Final Used / 316 | 1.857613554762687 / 4.895696329506463 / 5.217287818603588 | 1.1915254861794888 / 2.3271224248353564 / 5.784378607182949 |
| EKF / 6396 | 1.8878773199475662 / 5.01103904419252 / 5.667156999562539 | 1.2481083256675167 / 2.552406529827499 / 6.247121759931853 |

| Layer | Control rotation RMSE / P95 / max deg | P18 = P19 Visual |
|---|---:|---:|
| Initial | 3.0349318365382127 / 3.666385051665659 / 16.070196253297006 | 2.789434210060467 / 3.5169154086603247 / 14.546204608148408 |
| Raw | 2.7164558286462053 / 3.4565224794979397 / 15.645633451651502 | 2.4720131292617054 / 3.464336051625642 / 10.344414696633349 |
| Used | 3.023444139557699 / 3.528819951122136 / 15.645633451651502 | 2.7839009391950387 / 3.496866526295762 / 15.4726379145803 |
| EKF | 3.0262463004912794 / 3.705964593218566 / 16.393782193573646 | 2.775292497875063 / 3.6163535235988036 / 14.930006827113552 |

## Exact peaks and their source observations

Control maximum: **1517157256.246928, +32.05794906616211 s**,
5.667156999562539 m / 16.32253690909127 deg. At that same time Visual:
0.7915591427458251 m / 8.148397804204633 deg.

Visual maximum: **1517157259.186752, +34.99777317047119 s**,
6.247121759931853 m / 2.1832605694421856 deg. At that same time Control:
4.859820525932878 m / 2.591615426339948 deg.

Map-frame error at Visual maximum is [-6.213738395982901,
-0.5332884352968756, -0.36275180837700516] m. Its offline motion-tangent
component is -6.144907643018379 m; orthogonal magnitude 1.1254511727252547 m.
Control at its own maximum has map error [5.54924514930444,
-0.10064213312928771, -1.1456080881570188] m and tangent +5.229508031286789 m.
Thus late Visual lags; peak Control overshoots. Neither tangent is proven
to be the NDT Hessian weak eigendirection. Vector calculation is
`e_map=p_est-R_fit.T*(p_GT-t_fit)`; norm agrees with the frozen evaluator.

Visual peak's latest accepted scan is frame **396**, scan-start
1517157259.026242 (+34.837263107299805 s). Its stages are:

| Stage | Actual time | Error m / deg | IMU-origin map position m |
|---|---:|---:|---|
| Initial | 1517157259.026242 | 5.7636152102569795 / 1.4106388376830266 | [81.879823158, -4.575118076, -2.950406100] |
| Raw | same scan time | 5.784378607182949 / 1.406638308248718 | [81.861240948, -4.597083919, -2.954726574] |
| Used | same scan time | same as Raw | same as Raw |
| Corrected | 1517157259.176752 | 6.218028735010681 / 2.1272232695892157 | [81.813056172, -4.609216822, -2.959750893] |
| High-rate peak | 1517157259.186752 | 6.247121759931853 / 2.1832605694421856 | [81.810111817, -4.610624166, -2.959915822] |

The original Raw LiDAR position is [81.94216918945312,-4.571823596954346,
-2.923847198486328], distinct from its converted IMU origin. Raw optimizer
reports converged, iteration=1, fitness=0.047033075832294866; translation
and rotation step limit both false; OOSM APPLIED, fixed-lag replay successful.
Previous Used, current guess, Raw and replay state are in the per-scan ledger.

Latest visual application: frame279, 1517157247.226354
(+23.03737497329712 s). Age at maximum: **11.960398197174072 s**.
No direct visual application at either peak or anywhere in26–35 s.
Used→Corrected spans0.150510 s. Independent review measured GT displacement
0.390387 m during it, while estimated map x changes-0.048185 m. Therefore
the0.433650 m difference in errors is not an isolated EKF degradation effect.

## Full fixed tail distribution

Every row uses all corresponding high-rate samples. Counts >1/>2/>5 m
are strict; bins are [start,end). Values below are rounded only for display;
full precision is in tail_window_summary.csv.

| s / samples each | Control RMSE/P95/max m | Visual RMSE/P95/max m | C counts >1/>2/>5 | V counts >1/>2/>5 |
|---|---:|---:|---:|---:|
| 26–27 /199 | .913928/1.070031/1.087772 | 1.027408/1.226532/1.248482 |44/0/0|99/0/0|
| 27–28 /200 | 1.196592/1.271047/1.295314 | 1.245026/1.293401/1.300668 |200/0/0|200/0/0|
| 28–29 /200 | 1.443516/1.550077/1.568616 | 1.348310/1.417419/1.424707 |200/0/0|200/0/0|
| 29–30 /200 | 1.846317/2.135093/2.174337 | 1.429410/1.456555/1.461446 |200/36/0|200/0/0|
| 30–31 /200 | 2.710061/3.147061/3.288301 | 1.185549/1.334898/1.355959 |200/200/0|191/0/0|
| 31–32 /200 | 3.954909/5.254117/5.461563 | .938412/1.137504/1.226857 |200/200/26|36/0/0|
| 32–33 /200 | 5.041844/5.496518/5.667157 | 1.443780/1.950503/2.005547 |200/200/107|168/1/0|
| 33–34 /200 | 5.182985/5.285291/5.293429 | 2.739606/3.429317/3.547531 |200/200/196|200/200/0|
| 34–35 /200 | 4.830196/4.963148/4.987983 | 4.896085/6.071543/6.247122 |200/200/0|200/200/85|

30–35 s /1000states: Control4.4407220126279725/5.276477461946436/
5.667156999562539 m; Visual2.6776010183180254/5.531117266706065/
6.247121759931853 m. Counts C1000/1000/329, V795/401/85. Lower window
RMSE and fewer >5m samples coexist with worse tail quantile/maximum.
32–35 s: ControlRMSE5.020436 versus Visual3.344727 m, but VisualP95
5.819384 versus Control5.282273 m. Do not discard this counterexample.

## Observed propagation, and what is not proved

First Control/Visual Initial divergence: scan5,1517157219.592419,
evaluation-4.596560 s,0.015874214489234355 m, direct vision used. Raw/Used
differences0.006685658421498248/0.006685175164173717 m. First high-rate
separation at1517157219.744208 (-4.444771 s):0.0128293675148096 m.
The next scan6 already has different deskewed-source hashes. These are
real changed feedback histories before evaluation, not scheduling drift.
Initial separation>.5m starts+29.592909 s; >1m at+29.996288 s. High-rate
separation>.5m at+29.543133 s and >1m at+30.043069 s. Sensor-time releases
remain identical; state values need not because measurements differ.

32–35 s contains29 scans per method, zero translation step limits. Visual
has19 one-iteration solves and3 rotation limits; Control22 and4. Visual's
mean Raw-minus-Guess translation is0.051413 m, yet its mean NDT-derived
map-x velocity is-0.106015 m/s while offline GT is+1.866275 m/s.
At scans+32.013366→+34.837263, Visual Raw IMU-map x moves-0.306601 m,
whereas Control moves+5.214511 m. The point is a stuck/reversed measurement
trajectory while actual motion continues, not a final isolated6m jump.
Visual first exceeds2m at+32.997885 and5m at+34.577837 s.

Maximum high-rate position jump: C1.127812134m, V1.097917292m overall;
32–35s C1.127812134m, V0.857832930m (both tail maxima at+32.062941s).
All >2m diagnostic counts are0. There are local OOSM changes, but they
do not constitute the6.247m peak: the tail error grows across many states.

Actual source mechanisms, unchanged this round:

```cpp
// dog_prior_map_ndt_node.cpp::handleCloudLocked()
initial_guess = previous_pose_ * delta_pose_; // translation from NDT history
// optional valid vision changes translation only; IMU rotation retained
ndt_.align(aligned, initial_guess.cast<float>());
delta_pose_ = previous_pose_.inverse() * used_result;
previous_pose_ = used_result;

// fusion/ndt_observation.cpp::processNdtObservationLocked()
odom_velocity = (p_target - last_ndt_observation_p_map_) / dt;
v_ = (1.0 - blend) * v_ + blend * odom_velocity; // frozen blend=.6
// then existing real IMU replay to the fixed sensor-time endpoint
```

At final scan, Visual NDT-derived vx=-0.421072165m/s, pre-blend vx=-0.240032763,
computed blended vx=-0.348656404, release vx=-0.298739406; GT at scan is
+2.562836056m/s. This demonstrates the velocity channel actually carries
stalled/wrong-sign measurement motion. NDT initial translation does not
directly read EKF position; EKF can affect subsequent deskew/source instead.
Thus logs support **history-dependent NDT stall plus velocity propagation**,
not that velocity_blend uniquely caused the bad NDT basin, nor that vision
itself is incorrect, nor geometrical degeneracy without a Hessian test.

## Specific questions A–D

A: In6–12s's35 applied scans, same-history pre-hook initialRMSE
0.676393056→visual0.649684054→Raw0.672307020m. Raw increases error on31/35;
25/35 solve inone iteration. This supports limited retention of immediate
initial benefit; actual1200-state EKFRMSE .648813408→.646515666 improves
only0.354145%. No single-frame unique attribution is claimed.

B: No vision after+23.037375s. Prior vision already changed pose histories,
NDT increments, and next deskewed inputs. Therefore26–35s differences are
downstream causal propagation, not late direct vision or common-source
paired optimizer evidence.

C: Visual spends much of30–33s nearer the reference, reducing many errors,
then its near-stationary measurement sequence loses continuing forward
motion. That concentrates large errors in the final seconds. RMSE down
and max up are compatible; both replicated exactly.

D: On121 primary applied scans, pre/post-hookRMSE .531094003→.519163772,
Raw .528731772m. Raw improves initial on32 and worsens on89. No guarantee
that native NDT preserves a visually better initial guess; no new remedy
was developed here.

## Measured resource scope

P19 mean/P95/max ms: NDT align10.578260922/15.356556/62.466587;
deskew13.459852591/18.03493425/20.462092; NDT callback to published pose
14.027343856/19.207239151/65.529823303; paired deskew+callback stage sum
27.487196447/34.264547499/84.851469303.
Wall169.709919526s (.25x replay including orchestration); CPU NDT7.96s,
EKF9.48s. VmHWM NDT78.01953125MiB, EKF17.9765625MiB. Independent peak sum
95.99609375MiB double-counts shared pages, not whole-system unique RAM.
No per-EKF-callback latency measured. No new resource anomaly established.
Offline visual extraction's historical74.499/84.215ms excludescameraIO/MEI;
this CSV replay does not measure an online frontend or certify10Hz deployment.

## Unique NEXT, proposal only

One bounded causal **Visual velocity-feedback ablation**, .6 (replicated
baseline) versus0, with all other input/init/map/NDT/vision/OOSM parameters
frozen. Hypothesis: the NDT-derived wrong-sign velocity channel sustains
tail error through EKF propagation/deskew. Minimum change: independent
experiment YAML's `ndt_observation.velocity_blend` only; inspect existing
`fusion/ndt_observation.cpp::processNdtObservationLocked()` bookkeeping.
No new algorithm, filter, parameter grid or visual threshold. This is a
mechanism experiment for the decision AI to authorize, not an originality claim.

One0-blend40s run against existing .6 trajectory, same fixed alignment.
Metrics:3–35 and32–35 Initial/Raw/Used/EKF RMSE/P95/max, NDT-derived and
release motion, source changes, convergence, OOSM/nonfinite/jumps andruntime.
If Raw still stalls and tail error persists, reject velocity feedback as a
sufficient remedy; report the direct NDT-increment prediction loop as an
unseparated factor. Stop on input/time/state contract failure, late/OOSM
failure or nonfinite. No second parameter try. **Not executed in P19.**
