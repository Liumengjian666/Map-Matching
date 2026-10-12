# P21-R1: causal EKF translation is available, but replacement regresses localization

TASK: P21-R1-CAUSAL-EKF-TRANSLATION-PRIOR-ABLATION

FINAL_RESULT: **EKF_TRANSLATION_PRIOR_REGRESSION**.

EXPERIMENT_CONTRACT_PASS; DEFAULT_OFF_PARITY_PASS;
CAUSAL_PREDICTION_CONTRACT_PASS; SHORT_EKF_FALLBACK_PASS.
One default-off18.5s run, one enabled18.5s run, then exactly one new full
enabled40s run. No full baseline rerun or parameter tuning. Legal prediction
availability does not establish prediction accuracy. P21 primary EKF
RMSE/P95/max=9.944471988850724/23.55549859180675/27.709303867817365m,
versus P19=1.2481083256675167/2.552406529827499/6.247121759931853m.
RMSE/P95/max worsen696.7635/822.8741/343.5531%. Do not deploy this mode.

## Identity and code scope

Repository Liumengjian666/Map-Matching; branch
research/p9-r4-heldout-visual-evidence; actual workspace
/home/jian/livox_ws/p10_corridor01_oosm_r1_ws.
START_SHA=98c08420267d9a6dd969e850f3a5f9e640c47cb5.
Runtime interface commit=6dd9ed8088bc6524950be98ebaac5570b928d02d.
CODE_SHA=91e6ecb615e4207349735249396fabb430ddde7d, before raw GT opened.
END_SHA/verified remote/push status are in the final chat receipt, not a
self-referential commit field. Archive push fields are generation-time state.

New default-off lidar_update/experimental_ekf_translation_fallback_enable.
Only added this flag in the P21 copy of P18 YAML; beta remains.6.
NDT source changes: constructor, ekfHighRateCallback(MessageEvent), pending
readiness, handleCloudLocked after visual apply, and diagnostic fields.
New core/ekf_translation_prior.hpp performs selection/translation assignment,
not prediction mathematics. Existing ekfPredictionAtOrBeforeLocked and
predictMapTLidarWithBodyTwist are reused. Only the NDT target and focused
tests rebuilt. EKF/NDT optimization/OOSM/visual/deskew/step-limit mathematics
unchanged. No Coupled/Anchor path enabled. Original P17 runner unchanged.
Stable robot and dirty main workspaces untouched; no bag/PCD committed.

Twelve C++ selection cases pass, existing causal prediction test passes,
five Python contract tests pass (including static single-align check).
Initial multi-target make requested a newly generated target before the old
Makefile invocation knew it; subsequent explicit targets built and ran.
Two analysis errors were corrected without another ROS run: the short
contract's183rd unreleased observation is an expected shutdown endpoint,
and Python ROS Time uses to_nsec(). Short endpoint acceptance is bounded to
one final observation whose deadline is unreached; full run permits none.

## Frozen assets and actual ROS contract

| Asset | SHA256 |
|---|---|
| SSD map4058751 bytes |103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f|
| V2 bag |7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e|
| visual.csv |10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79|
| original P18 YAML |dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220|
| P21 YAML |39895248d850095e5f8c60966f627831177bcceccee055faa9ade2f64bff3376|
| new NDT executable |edfb55928c12baa2193f2847b21d88c119df74af4d34631216886709cb85bfdc|
| unchanged EKF executable |fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a|

CONTROL, visual on, camera/geometry off, start reference, fixed lag150ms,
rate.25. Actual parameters and subscription checked before playback.
Default-off has no extra EKF subscription. Enabled constructor requires
the frozen ekf_imu_fastlio body-twist and fixed-lag configuration. Only the
new flag differs physically; output paths/process identities are separate.

## Causal coordinates and selection

Frozen publisher /dog_prior_map_ekf publishes current high-rate state after
IMU propagation/fixed-lag correction. It does not publish rollback states
on this topic; corrected output is separate. Map pose is map_T_lidar;
twist is child-LiDAR frame, including the real IMU/LiDAR lever arm. Frames
camera_init/cmu_rc2_velodyne. Fixed-lag accepted observation m satisfies
m+.15<=sample time; current scan cannot enter a prior sample<=scan-start.

Enabled callback validates actual publisher, frames, finite normalized pose
and twist, and strictly increasing stamps BEFORE history/watermark. Invalid,
duplicate/reversed stream or queue overflow stops, never bypasses readiness.
Existing pending callbacks wait for accepted watermark>=target without
blocking the mutex. Latest received sample<=target is selected; integer-ns
age<=20ms. No relaxed age, latest-arrival shortcut, or future sample use.

```cpp
// Existing prediction, not new EKF mathematics:
prediction = ekfPredictionAtOrBeforeLocked(scan_start_ns);
// Only after failed visual apply, valid previous pose and IMU rotation:
initial_guess.block<3,1>(0,3) = prediction.map_T_lidar.block<3,1>(0,3);
// Rotation unchanged; exactly one original ndt_.align().
```

Visual always has precedence. Missing/stale/invalid prediction leaves the
old initial pose unchanged. First scan keeps original initialization.
226 real queries/predictions/applications,0 rejected; full age min/mean/P95/
max=.02003/2.509287/4.757265/4.981013ms. Future-used and unknown-provenance0.
Receipt wall/callback ROS/receipt ROS times are separate. Source+lineage
verification and independent numerical reproduction are documented in
INDEPENDENT_REVIEW.md. No new runtime predictor was added by that review.

## Gate results and actual source counts

| Run | NDT | APPLIED/corrected | high-rate | visual | EKF translation |
|---|---:|---:|---:|---:|---:|
|default-off18.5s|183|182|3698|106|0|
|enabled18.5s|183|182|3698|106|76|
|enabled full|396|396|8004|169|226|

First180 transactions default-off match P19 Initial/Raw/Used, deskew,
OOSM/fixed release and EKF physical states. Maximum position difference0m;
orientation comparison residual3.182107307712431e-15deg;0 failures. Frozen
1e-9m/1e-8deg limits unchanged. Short age mean/P95/max2.525675/4.801588/
4.978025ms; no future-use/nonfinite/late. Full schedule/timestamp arrays
match P19.396 optimizer-reported converged; this is not true matching accuracy.

Full sources:169 visual,226 EKF,1 original first-frame initialization.
Actual historical-NDT-increment fallback=0. The log dispatch reason
NDT_HISTORY_FALLBACK includes that first scan; initial_guess_source=current_pose
disambiguates it. Do not describe it as a successful history-increment frame.
Primary3–35 has121 visual/195 EKF/0 history.26–35 has88 EKF and0 visual;
32–35 has29 EKF. Latest EKF use is last scan1517157259.026242(+34.837263s).

First intervention:scan2 at1517157219.289839(-4.899140s), before evaluation.
Initial translation changes1.088959558m, rotation0deg; Raw differs
1.002869108m/2.468919660deg, Used.512541938m. First EKF difference at
release1517157219.444208:.983410505m; source hash first changes scan3.
This is an actual closed-loop history change, not a same-state comparison
for every later frame. Mean/P95/max applied translation change over all226
frames=.776485373/1.451490713/1.543090903m; rotation replacement remains0.

## Fixed evaluation: all four layers

86 files hashed before raw GT opened. Original Control V2 0–3 full-pose
transform and GT interpolation reused without fitting. Origin1517157224.188979.
LiDAR stages converted to IMU origin; high-rate already IMU. Primary3–35,
same arrays for all methods. PREFIX-ALIGNED RELATIVE DRIFT, not absolute ATE.
Independent review reproduces all864 statistics and24102 error samples.

Translation RMSE/P95/max m, P21 minus P19:

| Layer / n | P19 | P21 | Absolute change | Percent change |
|---|---|---|---|---|
|Initial/316|1.192362409/2.367812327/5.763615210|9.826881768/23.242515882/26.980804049|+8.634519359/+20.874703555/+21.217188839|+724.1523/+881.6030/+368.1229%|
|Raw/316|1.191989542/2.326919708/5.784378607|9.822100327/23.200676354/26.969497146|+8.630110785/+20.873756646/+21.185118539|+724.0089/+897.0553/+366.2471%|
|Used/316|1.191525486/2.327122425/5.784378607|9.823767458/23.288437807/26.569271303|+8.632241972/+20.961315382/+20.784892696|+724.4698/+900.7397/+359.3280%|
|EKF/6396|1.248108326/2.552406530/6.247121760|9.944471989/23.555498592/27.709303868|+8.696363663/+21.003092062/+21.462182108|+696.7635/+822.8741/+343.5531%|

Rotation RMSE/P95/max deg:

| Layer | P19 | P21 | Absolute change | Percent change |
|---|---|---|---|---|
|Initial|2.789434210/3.516915409/14.546204608|3.049972133/4.021247505/13.978413263|+.260537922/+.504332096/-.567791345|+9.3402/+14.3402/-3.9034%|
|Raw|2.472013129/3.464336052/10.344414697|2.816790911/3.528565462/13.038733367|+.344777782/+.064229411/+2.694318671|+13.9472/+1.8540/+26.0461%|
|Used|2.783900939/3.496866526/15.472637915|3.043357645/3.688457308/14.932879404|+.259456706/+.191590781/-.539758510|+9.3199/+5.4789/-3.4885%|
|EKF|2.775292498/3.616353524/14.930006827|3.045416442/4.083210458/14.347151297|+.270123944/+.466856934/-.582855530|+9.7332/+12.9096/-3.9039%|

Preserve lower rotation maxima as counterexamples, not overall success.
Original no-vision Control V2 EKF reference1.887877320/5.011039044/5.667157000m.
It is not the single-factor comparison group. Full precision in CSV/JSON.

Source-conditioned NDT RMSE (Initial/Raw/Used): P19 visual cohort121
.519164/.528732/.528741m versus P21 .588802/.595766/.595775m;
P21 EKF cohort195 Initial12.500957/Raw12.494661m versus P19
1.461739/1.459116m. Both methods use identical cohort timestamps, not
independent histories. Within P21's SAME history, original history guess
RMSE12.513708→EKF guess12.500957→Raw12.494661m: a very small immediate
change cannot compensate for the accumulated bad history. No usable primary
history-fallback cohort exists; do not fabricate its metrics.

## Tail risk and local propagation evidence

| Window / high-rate n | P19 RMSE/P95/max m | P21 | >1/>2/>5m P19 | P21 |
|---|---|---|---|---|
|6–12/1200|.646516/.811471/1.207470|.661988/.822021/1.408987|20/0/0|20/0/0|
|26–30/799|1.271772/1.448459/1.461446|12.614070/16.980187/17.990660|699/0/0|799/799/799|
|30–35/1000|2.677601/5.531117/6.247122|22.311559/26.492802/27.709304|795/401/85|1000/1000/1000|
|32–35/600|3.344727/5.819384/6.247122|23.901137/26.966053/27.709304|568/401/85|600/600/600|
|34–35/200|4.896085/6.071543/6.247122|25.769614/27.479425/27.709304|200/200/85|200/200/200|
|3–35/6396|1.248108/2.552407/6.247122|9.944472/23.555499/27.709304|1514/401/85|2233/2067/1900|

Unchanged5s persistence: P21 >1m starts+24.3984451294s, >2m+24.8034369946s;
durations10.599328/10.194336s. P19 has no confirmed1/2m5s span before35s,
but its late crossing is truncated, not evidence of no drift.

P21 max1517157259.171744(+34.9827651978s)=27.709303867817365m;
P19 same time6.169210047704989m. P21 map error
[-27.700297492,.622462391,.334036360]m: predominantly backward map-X lag,
not proof that X is a Hessian weak eigenvector. Latest accepted scan395:
Initial26.980804049→Raw26.969497146→Used26.569271303m at1517157258.925443;
corrected27.273774664m at1517157259.076752; high-rate peak at its own time.
NDT one iteration, fitness.048121080, translation limit=true, OOSM APPLIED.
Do not assign different-time stage-error differences uniquely to EKF.
Latest visual remains+23.037375s; age at P21 peak11.945390s. Tail direct
visual0, EKF applications88. Main error is already in the Initial and Raw
position, not solely introduced by later step limit or EKF correction.

32–35s map-X mean velocities m/s,29 scans:
Initial-.139389→+.483360; Raw-.109642→+.544781;
Used-.106015→+.166776; GT same scan intervals+1.846674.
EKF release+.023746→+.187121 versus actual-release GT+1.928070;
vx error RMSE1.975742→2.735636m/s. This is not restored true forward motion.
No new average overspeed like P20, but wrong velocities remain; do not
claim every instantaneous velocity is bounded by these means.

Tail one-iteration19/29 in both; mean fitness2.441650→2.625975.
Translation limits0→29/29, rotation limits3/3. Raw-minus-Initial IMU-origin
mean.051376→.052091m. Raw–Used LiDAR-origin difference mean.432175m,
max.515029m in P21; original29 tail frames zero. Full translation limits
2→118, rotations29→27. Original limitNdtStep caps displacement from
previous USED, not from the new initial guess; its rule was NOT changed.
These are observed coupled-history/limiter responses, not isolated proof
that the limiter uniquely caused drift. Many low-fitness one-iteration
converged outputs remain unreliable despite legal EKF initials.

## Stability, resources and interpretation

Max adjacent high-rate position jump1.097917292→1.473578345m;
tail.857832930→1.387079341m. >2m jumps0, nonfinite0, late/OOSM errors0.
Rotation jump maximum6.954350→6.475719deg, >20deg0. No new gross jump,
but severe sustained accuracy regression must not be labelled safe benefit.

P19→P21 timings ms mean/P95/max:
NDT10.578261/15.356556/62.466587→11.013541/16.938091/63.725560;
Deskew13.459853/18.034934/20.462092→13.341974/18.103670/20.439120;
callback14.027344/19.207239/65.529823→14.383536/20.330846/68.942547;
stage sum27.487196/34.264547/84.851469→27.725511/35.198978/85.977842.
CPU NDT7.96→8.90s, EKF9.48→9.55s; wall169.709920→169.531496s.
VmHWM NDT78.019531→77.839844MiB, EKF17.976563→17.644531MiB.
Query/selection path across396 mean/P95/max.004209/.008049/.015994ms.
Received-sample validation only .020009/.030022/.552800ms; excludes history,
logging/pending NDT. Full subscription CPU cost cannot be isolated from the
changed matching workload. Independent RSS peaks double-count shared pages.
Offline visual CSV; no full online vision/10Hz deployment claim.

All engineering reference conditions fail. Legal coordinate/time transport
is supported, but direct absolute EKF replacement is not a reliable accuracy
repair here. EKF shares NDT feedback, so it is not independent position
evidence. This single closed-loop run cannot isolate each later optimizer,
limiter and feedback effect, establish one unique root cause or originality.

Unique NEXT proposal, NOT executed: keep P19 baseline and do not deploy
P21 replacement. At the fixed failure-onset scans, compare the two actual
initial poses with the SAME reconstructed/preprocessed source and target,
same original NDT/limit parameters, as a small offline paired probe. Verify
source hash first. Hypothesis: direct optimizer/limiter effects may differ
from accumulated feedback-history effects; a causal prior alone may offer
no correct alternative basin. Use at most a few fixed pairs, same frozen
evaluation, no extra full ROS run or tuning. Stop if source cannot be
recovered legally or if no better position is obtained; preserve negative
results. It is a mechanism diagnostic, not a new scoring/optimization
algorithm. Decision AI must authorize any next experiment; P21 ends here.
