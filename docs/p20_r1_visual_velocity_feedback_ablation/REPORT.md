# P20-R1: removing explicit NDT velocity mixing regresses localization

TASK: P20-R1-VISUAL-VELOCITY-FEEDBACK-CAUSAL-ABLATION

FINAL_RESULT: **BLEND_ZERO_REGRESSION**

EXPERIMENT_CONTRACT_PASS; VELOCITY_INTERVENTION_CONFIRMED. Exactly one new
full beta=0 ROS replay; beta=.6 reuses P19, not rerun. The intervention
changes the expected velocity channel and downstream deskew/matching, but
does not improve overall accuracy: EKF RMSE +89.77887670252667%, P95
+119.33268109841855%; max -1.3949282413727206%. All three predeclared
engineering screen conditions fail. Keep the small max benefit and resumed
NDT forward motion as counterexamples to an overly simple negative account.

## Git and actual changes

Repository Liumengjian666/Map-Matching; branch
research/p9-r4-heldout-visual-evidence; actual workspace
/home/jian/livox_ws/p10_corridor01_oosm_r1_ws.
START_SHA: 389243ae84cedb9a75964c8d43584cf60e233961, confirmed remote at start.
CODE_SHA: 18c3e2cbe60087f9855880584cec43fe934e73d6.
Runtime source/binary version remains P19. This CODE commit contains the
independent configuration and analysis/supervision code, not a new filter
or NDT build. Final archive END_SHA and verified remote status are provided
in the chat handoff. Archive Git status fields are recorded before push,
not a claim that remote delivery had already occurred at generation time.

New `p20_corridor01_visual_blend0.yaml` differs from original P18 YAML only
at `ndt_observation.velocity_blend`, .6→0. Structured difference count=1.
Original P18 config unchanged. New scripts under scripts/p20_velocity_ablation:
run_guarded.py, audit.py, mechanism.py, evaluate.py, archive.py, test_gate.py.
Eight focused tests pass. No runtime C++ source, P17 runner, NDT/EKF/OOSM,
deskew, visual extraction, step limit, initialization or noise is changed.
Stable robot and dirty main workspaces untouched. No PCD/bag committed.

## Frozen input and actual parameter gate

| Asset | Actual SHA256 |
|---|---|
| SSD map,4058751 bytes | 103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f |
| V2 bag | 7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e |
| visual.csv | 10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79 |
| NDT executable | ba65e3e516a982c4c0083b866fc8e95c2dede4003fdc0f42cbf1a489da931cfb |
| EKF executable | fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a |
| Original P18 YAML | dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220 |
| P20 beta=0 YAML | 9a253dc5bf58e2dc4c3495077a9549f4d67d376a6a4fa23aef74c5be7d2e4e18 |

TCP bind and process-residual checks passed. Existing run.py performs its
usual launch; independent read-only guard checks the actual ROS root
parameter tree before playback, then again after completion. Physical
parameters differ only in beta; output locations/ROS process IDs are
normalized explicitly. Actual beta=0.0. SSD map, Visual on, camera off,
CONTROL, scan-start, lag150ms, .25x playback, geometry off. No external map
read. The complete raw runtime cache and output bag remain local.

## Completeness and controlled schedule

Both runs:396 NDT/converged,396 OOSM APPLIED/corrected,8004 high-rate states,
169 visual applications; late0, nonfinite0, extra complete align0. Recorded
output bag counts match all ledgers. Input/sensor-time arrays and fixed-lag
release/replay endpoints match; expected downstream sources/poses differ.
Deskew396 published, no current-scan NDT leakage or post-scan-end IMU use.

P20 iterations mean1.371212121212121,max18,338 one-iteration frames;
translation limited4,rotation limited28,combined28. Iteration ceiling remains
40, not18. P19 combined step-limit29; no step rule was bypassed.
Read-only review led to analysis-only safeguards for deskew time arrays,
finite fitness and lineage observation association. Mutated-input tests
are rejected; all actual records pass. No second ROS run was needed.

47 artifacts/inputs/runtime-source files were hashed in
P20_PRE_GT_OUTPUT_FREEZE before raw GT opened. Receipt identity is P20, not
a relabeled P18 run. The earlier frozen completeness/mechanism output is
unchanged; final hardening checks are separate cache outputs. Raw source
and archived CSV hashes/row equality are recorded. Archived launch.log
trims trailing blanks only; original raw log/hash remains intact.

## Direct intervention and propagation order (no GT)

Existing source:

```cpp
// processNdtObservationLocked(): position correction and log first
applyPoseCorrection(dp, dtheta);
writeEkfPredictionLineage("NDT_CORRECTION_APPLIED", ...);
if (dt > 1e-3 && dt < 1.0) {
  odom_velocity = (p_target-last_ndt_observation_p_map_)/dt;
  v_ = (1.0-blend)*v_ + blend*odom_velocity;
}
// then real IMU replay to the fixed release node
```

`applyPoseCorrection()` changes p/R, not v. Beta0 removes only this explicit
NDT-derived velocity mix, not position observations or all NDT feedback.
NDT initial translation still uses previous_pose*delta_pose, and delta_pose
is updated from the latest step-limited used pose. EKF can affect subsequent
deskew/source, not directly that translation predictor.

First eligible mix: observation2,1517157219.289839, dt=.1008601188659668s;
rollback state1517157219.289216; release1517157219.444208.
All57 pre-intervention physical-state checks and first2 NDT source/pose
checks pass. Vectors in map m/s:

| Value | beta=.6 | beta=0 |
|---|---|---|
| Same NDT difference velocity | [.500171203,-.124639044,.013367376] | same |
| Same pre-mix velocity | [.129427652,-.103283131,-.010327948] | same |
| Computed explicit post-mix | [.351873782,-.116096679,.003889247] | [.129427652,-.103283131,-.010327948] |
| Observed post-replay velocity | [.498629311,-.299544746,.037048056] | [.276183181,-.286731199,.022830862] |

Post-replay velocity difference magnitude=.22326799259699645m/s; predicted
mix-difference residual=7.110250420355456e-17m/s. Explicit post-mix values
are reconstructed from recorded inputs/source formula, not an invented
standalone measurement. Release velocity includes actual IMU propagation.
First high-rate/corrected position difference=.03460472260270938m at the
same release; rotation difference0deg.

Next scan3,reference1517157219.390699, end1517157219.4915383:
deskew mean displacement .132998908→.130609972m; P95 .369004957→.360545034m.
Source hash aed74fa320333a9f→54572539bb314373. Raw pose first differs
.018968897930436348m/.053841741343069606deg. Scan4 Initial first differs
.03790892663225462m/.05384175175820294deg at1517157219.491559.

Scan3 start precedes release2, but its scan end and actual processing follow
release2: no causal paradox. Subsequent OOSM rollback may also label an old
state timestamp after rewriting history. Association/processing nodes,
not a historical timestamp alone, determine whether a record is pre-mix.
This distinction is tested. No unexplained earlier state difference found.

## Fixed GT evaluation, four layers

Reuse original Control V2 0–3s full-pose transform, origin1517157224.188979;
no refit. Primary3–35s. PREFIX-ALIGNED RELATIVE DRIFT, not absolute map ATE.
LiDAR stages convert to IMU using frozen extrinsics; EKF already IMU.
Same arrays for all methods; no intersection/sample selection. Original
GT interpolation, including its existing near-neighbor quaternion handling,
is unchanged. Independent review reproduced all24,102 samples and window
statistics. Baseline Control/P19 metrics match their frozen archives.

Translation RMSE/P95/max m; changes are beta0 minus beta.6:

| Layer / n | beta=.6 | beta=0 | Absolute change | Percent change |
|---|---|---|---|---|
| Initial /316 |1.192362409/2.367812327/5.763615210|1.917651665/4.688549460/4.967457863|+.725289257/+2.320737133/-.796157347|+60.8279/+98.0119/-13.8135%|
| Raw /316 |1.191989542/2.326919708/5.784378607|1.921650380/4.682471932/4.960597971|+.729660838/+2.355552224/-.823780637|+61.2137/+101.2305/-14.2415%|
| Used /316 |1.191525486/2.327122425/5.784378607|1.916219274/4.646690726/4.960139477|+.724693788/+2.319568302/-.824239130|+60.8207/+99.6754/-14.2494%|
| EKF /6396 |1.248108326/2.552406530/6.247121760|2.368645960/5.598261674/6.159978894|+1.120537635/+3.045855145/-.087142866|+89.7789/+119.3327/-1.3949%|

Rotation RMSE/P95/max deg:

| Layer | beta=.6 | beta=0 | Absolute change | Percent change |
|---|---|---|---|---|
| Initial |2.789434210/3.516915409/14.546204608|2.839343238/3.562744720/14.922214187|+.049909028/+.045829311/+.376009579|+1.7892/+1.3031/+2.5849%|
| Raw |2.472013129/3.464336052/10.344414697|2.500069431/3.426767615/12.408604671|+.028056302/-.037568437/+2.064189974|+1.1350/-1.0844/+19.9546%|
| Used |2.783900939/3.496866526/15.472637915|2.834957140/3.505211886/15.253358825|+.051056201/+.008345359/-.219279090|+1.8340/+.2387/-1.4172%|
| EKF |2.775292498/3.616353524/14.930006827|2.827528743/3.629718975/14.667098506|+.052236245/+.013365451/-.262908321|+1.8822/+.3696/-1.7609%|

Original no-vision Control V2 EKF reference: translation
1.8878773199475662/5.01103904419252/5.667156999562539m;
rotation3.0262463004912794/3.705964593218566/16.393782193573646deg.
It is not the causal beta.6 control group. Full-precision values/deltas
for all4 layers and5 windows are preserved in accuracy_comparison.csv.

## Tail distribution and persistent drift

| Window / n | beta=.6 RMSE/P95/max m | beta0 RMSE/P95/max m | >1/>2/>5m counts .6 | counts0 |
|---|---|---|---|---|
|26–30/799|1.271772143/1.448458789/1.461446328|2.923906542/4.144946151/4.350433456|699/0/0|799/601/0|
|30–35/1000|2.677601018/5.531117267/6.247121760|5.148254082/5.981480485/6.159978894|795/401/85|1000/1000/535|
|32–35/600|3.344726852/5.819383745/6.247121760|5.496578978/6.026162849/6.159978894|568/401/85|600/600/487|
|34–35/200|4.896085059/6.071542701/6.247121760|4.943206468/5.554928086/5.719073029|200/200/85|200/200/87|
|3–35/6396|1.248108326/2.552406530/6.247121760|2.368645960/5.598261674/6.159978894|1514/401/85|2856/1601/535|

32–35 RMSE/P95 worsen while max slightly decreases.34–35 P95/max improve,
but RMSE slightly worsens; do not discard those partial benefits. Unchanged
5s persistence: beta.6 has no confirmed1/2m persistent crossing before35s,
but its late2m crossing is truncated, not proof of avoiding drift. Beta0
persistent1m starts+25.8283331394s (9.169440s span),2m+27.2282691002s
(7.769504s span). These are worse sustained tail outcomes.

Beta0 maximum at1517157257.456864, +33.2678852081s:
6.159978894229623m/1.330780804deg; beta.6 same time2.4345460788429207m.
At beta.6 maximum1517157259.186752(+34.997773s), beta.6=6.247121760m,
beta0=4.078801699m. Peak shifted, not simply removed. Beta0 peak map error
[5.953078499,-1.422628842,-.694495171]m: forward overshoot rather than
the baseline's final backward lag.

Latest beta0 peak observation378 at1517157257.210884(+33.021905s):
Initial4.963904130→Raw4.960597971→Used4.960139477m at scan time;
Corrected5.696972457m at1517157257.361856; high-rate6.159978894m at peak.
It reports converged, one iteration, fitness2.874774812, no translation
limit, rotation limit=true, OOSM APPLIED. Each layer is evaluated at its
actual timestamp, so differences are not isolated same-time EKF effects.
Latest applied visual initial-guess scan remains1517157247.226354
(+23.037375s), age10.230510s; none directly applied in26–35s.

## Motion and velocity: partial recovery is not localization recovery

32–35s29 scan cohorts, map X m/s:

| Diagnostic | beta=.6 | beta0 | Proper reference |
|---|---:|---:|---:|
| NDT Used difference |-.106015215|1.374953966|GT same scan intervals1.846673754|
| Initial sequence difference |-.139388661|1.391974667|same intervals|
| Raw sequence difference |-.109641814|1.374953966|same intervals|
| EKF release velocity |.023746139|6.442045303|GT at actual release1.928070380|
| Release vx error RMSE |1.975741693|4.515316043|actual-release matched GT|

Historical GT center-at-scan diagnostic is1.866275m/s for both, retained
separately from interval/reference-release velocities. Do not compare
scan GT to a150ms-later release as if simultaneous.
Raw-minus-Initial mean translation (IMU origin) .051375831→.027940514m.
One-iteration counts19→24/29; translation limits0/0,rotation limits3/3;
mean fitness2.441650121→2.157493144; all reported converged. Beta0 Raw
resumes forward motion but remains in a wrong relative position and many
one-iteration solves persist. Its EKF velocity overshoots real motion badly.
Closing the explicit mix is not a sufficient repair, nor equivalent to
turning off all NDT feedback. No acceleration-bias/gravity unique cause is
established by this single intervention.

## Stability and resources

Maximum high-rate jumps: position1.097917292m in both;32–35s .857832930→
.618186458m. Rotation6.954350246→5.593200009deg. >2m and >20deg counts0,
nonfinite0, OOSM failures0. Thus there is a substantial accuracy regression
without a new gross discontinuity/nonfinite failure.

| Stage ms mean/P95/max | P19 beta=.6 | P20 beta0 |
|---|---|---|
|NDT align|10.578261/15.356556/62.466587|10.563517/14.916196/65.240392|
|Deskew|13.459853/18.034934/20.462092|13.273126/18.143952/21.297342|
|NDT callback|14.027344/19.207239/65.529823|13.911157/18.615067/69.287300|
|Deskew+callback sum|27.487196/34.264547/84.851469|27.184283/33.388228/83.162486|

CPU NDT7.96→7.95s, EKF9.48→9.40s; replay wall169.709920→169.599962s.
VmHWM NDT78.019531→77.722656MiB, EKF17.976563→17.738281MiB. No extra
NDT align or new map; no performance optimization. Independent peak sums
double-count shared pages, not whole-system unique RAM. No online camera
frontend/per-EKF-callback latency measurement; no complete10Hz claim.

## Scientific interpretation and unique NEXT (not executed)

H1 confirmed by the first same-state velocity mix/release. H2 downstream
state, deskew, source, initial and raw changes observed. The proposed
"delete this channel to fix the tail" is rejected for this frozen scenario:
global error and persistence worsen even though NDT forward motion resumes.
This does not prove beta=.6 optimal, identify the sole bad-basin cause, or
establish an original algorithm. It does show the channel is an important
closed-loop participant with both useful and harmful possible consequences.

Engineering thresholds: max6.159979>Control5.667157;
RMSE2.368646>allowed1.310514; P955.598262>allowed2.680027. All fail.
Do not classify A from the one reduced maximum, or B as if Raw still had
the same backward stall. Case C, BLEND_ZERO_REGRESSION, preserves both signs.

Unique NEXT proposal: retain beta=.6 and all frozen vision/filter settings;
test only the no-visual NDT initial-translation source, historical NDT
increment versus existing causal EKF prediction at the correct scan time.
Hypothesis: direct reuse of a bad NDT increment helps maintain a wrong
matching basin; breaking that direct predictor edge might alter it without
removing useful velocity correction. Minimal implementation would add an
explicit default-off initial-translation-source mode in
dog_prior_map_ndt_node.cpp::handleCloudLocked(), reusing existing EKF pose
history/time gating and unchanged IMU rotation. No extra align or covariance
change. Legal causal prediction is a prerequisite; do not use future state.
One bounded40s run versus P19 baseline, unchanged GT transform; evaluate
Initial/Raw/Used/EKF and tail/performance. If the prediction is unavailable,
fall back exactly; stop for temporal/finite/OOSM failure. If no accuracy
gain, preserve negative evidence without tuning. This is a mechanism-source
ablation, not originality, and is not guaranteed beneficial because EKF
itself already contains NDT history. Decision AI must authorize it; **not
executed in P20**. No beta grid, next algorithm or additional ROS run.
