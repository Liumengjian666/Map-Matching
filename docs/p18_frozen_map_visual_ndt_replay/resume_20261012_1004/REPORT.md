# P18 resumed: frozen map recovered and visual causal replay completed

TASK: P18-FROZEN-MAP-RECOVERY-AND-VISUAL-NDT-REPLAY

FINAL_RESULT: **VISUAL_CAUSAL_BENEFIT_OBSERVED**

EXPERIMENT_VALID: YES. H1/H2/H3 primary translation RMSE improved in the
single controlled-switch replay. **Worst-case translation did not improve:
EKF maximum rose 10.23%.** This is observed aggregate replay benefit, not an
all-metric safety certificate, statistical significance, independent
generalization, a framewise causal-chain proof, or a new algorithm claim.

## Git and actual changes

Repository Liumengjian666/Map-Matching; branch
`research/p9-r4-heldout-visual-evidence`; actual research workspace
`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`.
Original P18 START_SHA: `3b3d53da8fe755c5f3c24dd8c99008579625b7a0`.
Resume START_SHA: `e43b07006c52a08eba0aeb7c0e651e1813d5c844`, confirmed remote
at resume. CODE_SHA frozen before GT:
`8250bcabd2dddf265b8c712e3495fd09cabc0c44`.
The final archive commit supplies END_SHA in the execution handoff.
PUSH_EXECUTED by execution AI: NO; a local commit is not remote delivery.

Runtime edit: `scripts/p17_visual/run.py` adds optional `--dataset-config`,
preserving the default P10 config, rejecting missing files, recording config
SHA and checking actual ROS map path before play. Independent
`config/p18_corridor01_ssd_map.yaml` changes only `map.pcd_fallback_path`.
New scripts audit prefix completeness/application, evaluate the frozen
comparison, summarize costs and archive outputs. A nonidentity float-matrix
case was added to the existing visual adapter test. No NDT/EKF/OOSM, camera
frontend, R6, Anchor, noise, initialization, step-limit or velocity-blend math
changed. Stable robot and dirty main workspaces were not edited.

## Storage recovery and frozen input

At 10:04:25 HIKVISION was newly enumerated on usb 2-2 as a SuperSpeed device;
actual mount `/dev/sda2`, `/media/jian/HIKVISION`, fuseblk rw. No new resets or
I/O errors were observed after that connection. This establishes successful
target-file recovery, not whole-drive health. One bounded 20s copy succeeded
to a new SSD directory, with no overwrite or retry:

```
/home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p18_map_recovery/assets/recovery.oEmedI/corridor01_map_normalized.pcd
size: 4058751 bytes
SHA256: 103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f
```

The earlier missing-device receipt and P17/P17R failures remain intact.
Raw source PCD was neither regenerated nor altered. Frozen V2 bag SHA
`7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e` and raw
visual CSV SHA `10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79`
passed. No camera extraction, parameter search or alternative input was used.
All three actual ROS snapshots agree in physical parameter groups; only
vision is false/true/true. They load the verified SSD map, not the external
map. Config SHA is `dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220`.

## Real replay gates

Chain: frozen raw timed LiDAR + IMU, existing scan-start EKF deskew, shared
PCL NDT, unchanged step limiting, fixed-lag OOSM and actual EKF feedback.
Independent masters were used at .25x playback, fixed sensor lag .150s,
CONTROL mode and camera fusion off. The full Vision run is a genuine ROS
feedback trajectory, not substituted historical CSV poses.

| Run | NDT / converged | OOSM APPLIED | High-rate / corrected | Actual vision |
|---|---:|---:|---:|---:|
| Disabled short, 18.5s bag duration | 183 / 183 | 182 | 3698 / 182 | 0 |
| Enabled short, same interval | 183 / 183 | 182 | 3698 / 182 | 106 |
| Enabled full, 40s input | 396 / 396 | 396 | 8004 / 396 | 169 |

Short Control's first 180 complete transactions and all state output rows
match frozen Control: zero differences, including source counts/hashes,
guess/raw/used poses, deskew, OOSM release/replay and EKF fields. The final
short NDT transaction is QUEUED outside the truncated lag boundary; it is
not silently dropped from the ledger. Full Control/Visual fixed-lag release
nodes and replay counts match on all 396 observations. Recorded bag topic
counts agree with the ledgers. Late observations and nonfinite states: zero.

Full visual apply calls 395, accepted 169, rejected 226 (all invalid visual
measurements); the first scan has no previous pose and does not call apply.
No valid pair was bypassed by an interface failure. Rotation change versus
the same-run IMU guess is zero. Mean/max translation change versus that guess
is 0.028868/0.278816 m. No extra complete NDT align was called: 396 total.

## Fixed offline evaluation

All actual outputs were frozen before copying/opening official GT; GT and
extrinsics SHA checks passed. The original Control V2 0-3s full-pose SE(3)
transform is reused unchanged. Paired sample timestamps are identical, with
no intersection-based reweighting. Origin is 1517157224.188979. LiDAR guess,
raw and used poses are converted to IMU origins; high-rate lineage already
contains IMU state. These are **PREFIX-ALIGNED RELATIVE DRIFT**, not absolute
map-frame ATE. Primary interval is 3-35s, outside the original fit interval.

| Component | Samples | Control translation RMSE/P95/max m | Visual translation RMSE/P95/max m | RMSE improvement |
|---|---:|---:|---:|---:|
| Initial guess | 316 | 1.859457 / 4.923620 / 5.253669 | 1.192362 / 2.367812 / 5.763615 | 35.88% |
| Raw PCL NDT | 316 | 1.859315 / 4.926219 / 5.218202 | 1.191990 / 2.326920 / 5.784379 | 35.89% |
| Step-limited used | 316 | 1.857614 / 4.895696 / 5.217288 | 1.191525 / 2.327122 / 5.784379 | 35.86% |
| Executed EKF high-rate trajectory | 6396 | 1.887877 / 5.011039 / 5.667157 | 1.248108 / 2.552407 / 6.247122 | 33.89% |

| Component | Control rotation RMSE/P95/max deg | Visual rotation RMSE/P95/max deg |
|---|---:|---:|
| Initial guess | 3.034932 / 3.666385 / 16.070196 | 2.789434 / 3.516915 / 14.546205 |
| Raw PCL NDT | 2.716456 / 3.456522 / 15.645633 | 2.472013 / 3.464336 / 10.344415 |
| Step-limited used | 3.023444 / 3.528820 / 15.645633 | 2.783901 / 3.496867 / 15.472638 |
| Executed EKF high-rate trajectory | 3.026246 / 3.705965 / 16.393782 | 2.775292 / 3.616354 / 14.930007 |

EKF translation delta RMSE/P95/max: **-0.639769 / -2.458633 / +0.579965 m**.
Relative improvements: **33.89% / 49.06% / -10.23%**. Its rotation changes
are -0.250954 / -0.089611 / -1.463775 deg (8.29% / 2.42% / 8.93% lower).
All other absolute/percentage changes are in `accuracy_comparison.csv`.
Every primary translation maximum worsened; Raw NDT rotation P95 rose 0.23%.

## Risk windows and direct versus downstream evidence

| Evaluation window | High-rate samples | Direct visual applications / scans | Control EKF translation RMSE m | Visual RMSE m |
|---|---:|---:|---:|---:|
| 6-12s | 1200 | 35 / 59 | 0.648813 | 0.646516 |
| 26-30s | 799 | 0 / 40 | 1.393310 | 1.271772 |
| 30-35s | 1000 | 0 / 48 | 4.440722 | 2.677601 |
| 3-35s | 6396 | 121 / 316 | 1.887877 | 1.248108 |

The first-10s EKF RMSE is 0.468620 versus 0.465187 m. The 6-12s RMSE change
is only 0.35%, not a large immediate improvement. Late windows have **no
direct vision**. Their differences are downstream causal history effects;
they do not prove late visual observations or isolated framewise causation.
In 30-35s, P95 worsens 5.276477 -> 5.531117 m and maximum reaches 6.247122 m.
Thus the no-vision tail still drifts, despite its lower window RMSE.

On the 121 primary applied frames, same-Visual-history before/after initial
translation RMSE is **0.531094 -> 0.519164 m** (2.25% lower), with identical
rotation error. This uses the already logged pre-hook IMU rotation guess;
it is not a counterfactual NDT replay. Across separate trajectories, applied
frame Raw NDT RMSE is 0.538791 -> 0.528732 m; on the 195 no-direct-vision
frames it is 2.328533 -> 1.459116 m. All 195 have earlier visual history.
Within Visual's applied frames, NDT changes initial 0.519164 to raw 0.528732
RMSE: the native optimizer does not consistently retain the initial benefit.

For high-rate states, grouping is by Visual's **latest accepted NDT
measurement**, not by a fictitious camera observation at every IMU tick.
The identical time partition is applied to Control as a comparison label;
Control itself has no visual history. The latest-measurement-visual partition
has 2440 primary states, RMSE 0.550536 -> 0.539283 m. The no-direct-visual but
earlier-visual-history partition has 3956, RMSE 2.361231 -> 1.529447 m.
There are no primary Visual states before any visual history. These are
conditional trajectory comparisons, not isolated update contributions.

Persistent threshold requires 5s continuous crossing, unchanged from P10.
Control 0.5/1/2m onset: 6.449565 / 26.783325 / 29.823069 s.
Visual 0.5m onset: 6.419565 s; 1/2m persistent crossing is absent within this
finite window. Its first 2m crossing is 32.997885 s, only ~2s before the end;
absence of a 5s crossing is **not proof of avoided drift or long-term stability**.

## Safety, optimizer status and resource costs

Maximum adjacent high-rate state jump Control/Visual: 1.127812/1.097917 m,
6.280469/6.954350 deg. Jumps exceeding the existing 2m/20deg correction-scale
diagnostic: 0/0. The threshold is a descriptive count, not a new admission
rule or proof that all jumps are harmless. Used-scan jump is bounded near
0.5m/5deg; rounding details are retained in the audit. Step limits: 28/29.
Mean/max NDT iterations: 1.303/16 versus 1.356/16. Mean PCL all-point fitness:
2.580236 versus 2.618754; its small worsening is not a GT correctness test.
396/396 convergence in both methods means optimizer status, not 100%
localization accuracy.

Full Visual measured mean/P95/max:

- NDT align: **10.489 / 15.685 / 68.342 ms**.
- NDT callback to publish: **13.921 / 20.379 / 72.469 ms**.
- Deskew: **13.613 / 18.519 / 21.847 ms**.
- Deskew + callback stage-sum: **27.535 / 35.060 / 83.354 ms**.
- Replay orchestration wall time at .25x: **169.400 s**.
- NDT/EKF cumulative process CPU: **7.89 / 9.52 s**; not EKF callback latency.
- NDT/EKF VmHWM peak RSS: **78.008 / 18.008 MiB**. Sum of independent peaks
  is 96.016 MiB, double-counting shared pages; not unique whole-system RAM.

Separate visual-interface latency and EKF callback latency were not measured.
The callback timer includes the CSV lookup. Camera IO/rectification/online
frontend, roscore and bag processes are not included in the deployment claim.
Historical P17 visual-stage mean/P95 74.499/84.215 ms excludes camera IO and
MEI rectification; old 194.5 MiB was a sampled RSS, not peak. No complete
online visual 10Hz / 100MiB system certification follows from these figures.

## Closure and only next recommendation

H1, H2, H3 aggregate primary RMSE benefit observed. The initial same-history
effect is much smaller than total trajectory difference; direct evidence is
kept separate from feedback history effects. P18 closes the storage and real
visual-feedback experiment gap, but does not establish originality, repeatable
gain across runs, independent-dataset generalization, or improved worst case.
No more experiment or tuning was performed after these results.

NEXT: **REPLICATE_FIXED_VISUAL_REPLAY_AND_VERIFY_TAIL_RISK**.
Recommend one independently repeated frozen comparison and explicit tail-risk
check before deciding formal visual-EKF integration. Keep the feature default
off; do not automatically introduce new fusion mathematics or tune parameters.
