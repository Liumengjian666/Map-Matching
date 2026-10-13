# P31-R1 Deskew contribution and visual ablation

FINAL_RESULT: `DESKEW_CONTRIBUTION_AND_VISUAL_ABLATION_COMPLETE`.
Experiment/math/time/finite/root gates PASS. Accuracy outcomes are mixed across modes; no claim that D or E improves accuracy.

START_SHA `eedcc2228158ee88bbd5a6b2bd2a84f809ae2a0c`; runtime CODE_SHA `beb2ba5ca6fda4406b1e76cc4d22f8f42d56d12c`;
analysis SHA `dd32c2dfaaa75e10fd30b08f78320998a31c3bbd`. Final END_SHA supplied in Git/chat. PUSH_EXECUTED=false.
P10 research workspace only; no writes to protected robot/paper workspaces or old archives/caches.

## Scoped implementation

New default-false `deskew/experimental_history_nominal_consistent_enable`, mutually exclusive with P30 scan-local.
LEGACY_HISTORY passes original config; D retains historical-node query with a local false/false midpoint/gravity-heuristic copy.
SCAN_LOCAL keeps original P30 builder/query. E disables only visual measurement updates; common time adapter/clones/IMU/root/OOSM stay ON.
No changes to NDT/DCReg/Multi/MSCKF noise/selection, velocity_blend0.6, Step Limit, gravity, extrinsic or filter/history mathematics.
See MATH.md/source_changes.md/TESTS.md. Same GNU9.4 -O2 -g -Wall; NDT binary unchanged.

## Bounded actual runs

Exactly one D21s short, one D40s and one E40s; no repair/replay/search or A/B/C rerun.
Dshort208 NDT/converged,207 due NDT OOSM,4198 high-rate,144 actual MSCKF updates: PASS.
Dfull396 NDT/converged,396 OOSM/corrected,8004 high-rate,312 actual MSCKF updates: PASS.
Efull396 NDT/converged,396 OOSM/corrected,8004 high-rate,0 actual MSCKF updates: PASS.
All actual timestamps match the prescribed frozen sequence, physical params differ only as specified. Late/nonfinite/invalid-root/future-IMU0;
all due NDT and visual transactions finish legally; E389 visual transactions all DISABLED, not lost observations.
Original bounded Multi-NDT outcomes D644 quick/1 extra/1 replacement (397 total aligns); E638/3/3 (399 total aligns).
No extra align is introduced by the Deskew branch. Eframe371 has a finite NONCONVEX DCReg characterization; nominal is retained.
Raw outputs were hashed/frozen BEFORE GT. No source reconstruction/capture or stepwise covariance audit in P31.

## Shared Control V2 accuracy

PREFIX-ALIGNED RELATIVE DRIFT, unchanged Control V2 0–3s full-SE(3), IMU origin, same interpolation. No new fit.
A/B=P29 historical, C=P30 historical; their archived metrics are reused exactly unchanged.
Each3–35s EKF row6396 samples. All four layers and deltas are in accuracy_metrics/comparison.csv.

|3–35s group|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|
|---|---|---|
|A|1.976924726 / 4.881597129 / 6.485653811|2.737326840 / 3.616325969 / 14.232664229|
|B|3.104707802 / 8.799511452 / 12.883491212|2.819498723 / 3.698021032 / 14.678118768|
|C|0.788369339 / 1.107826963 / 3.280421686|2.670388254 / 3.613658670 / 13.494033069|
|D|4.855348733 / 11.328267741 / 11.993582120|2.859214381 / 3.896876854 / 14.245781017|
|E|7.126678034 / 18.336510524 / 20.678766276|3.081246511 / 3.754042967 / 15.939119240|

|26–35s group|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|
|---|---|---|
|A|3.613071520 / 6.249051037 / 6.485653811|3.482010603 / 8.665038416 / 14.232664229|
|B|5.782601009 / 11.834723627 / 12.883491212|3.722440835 / 11.052596494 / 14.678118768|
|C|1.170516696 / 2.045275552 / 3.280421686|3.323240733 / 9.382555762 / 13.494033069|
|D|8.915843802 / 11.561028006 / 11.993582120|3.858527835 / 9.016579346 / 14.245781017|
|E|13.366816289 / 20.310698555 / 20.678766276|4.437389212 / 12.512794911 / 15.939119240|

|34–35s group|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|
|---|---|---|
|A|6.187017888 / 6.466967562 / 6.485653811|1.630375735 / 2.428784779 / 2.553696461|
|B|11.756361533 / 12.791086511 / 12.883491212|1.625251317 / 2.417305692 / 2.477293545|
|C|2.122533129 / 3.128137722 / 3.280421686|1.456057737 / 2.319726843 / 2.384210621|
|D|11.565145720 / 11.911975802 / 11.993582120|1.268604149 / 2.201877945 / 2.293115705|
|E|20.202932832 / 20.646985416 / 20.678766276|1.462215560 / 2.383320467 / 2.446758273|

## Conditional contribution contrasts

B→D: RMSE +1.750640931m (+56.3867%); P95 +2.528756289m (+28.7375%); Max -0.889909092m (-6.9074%).
Thus convention alignment alone does NOT reproduce P30 overall benefit. Its lower Max and ~1s RPE are retained positive countereffects.
D→C: RMSE -4.066979393m (-83.7629%); P95 -10.220440779m (-90.2207%); Max -8.713160433m (-72.6485%).
D is far from C, supporting substantial additional closed-loop benefit of scan-start continuous integration under the shared convention.
E→C: RMSE -6.338308695m (-88.9378%); P95 -17.228683561m (-93.9584%); Max -17.398344590m (-84.1363%).
C also improves rotation RMSE/P95/Max by13.3342%/3.7395%/15.3402%. Real MSCKF benefit is observed under SCAN_LOCAL in this scene.
Neither SCAN_LOCAL alone (E) nor consistent history alone (D) establishes the combined benefit. Strong closed-loop interaction remains;
these contrasts are NOT independent/additive causal fractions, per-frame isolated visual contributions, or statistical significance.
B/D Source first differs frame1; D/C Source first differs frame1 with first Raw difference frame2 (~1.23e-7m).
This precedes visual updates and includes integration partition/history-reset effects, not proof of a unique visual historical rewrite.
E/C Source and Raw first differ frame7, Initial frame8, consistent with the visual feedback sequence; time order alone is not unique cause proof.
Independent review confirms first six E/C Source hashes and Raw/Used pose strings exactly match.

## RPE and residual risks

~1s RPE translation RMSE A/B/C/D/E:0.456187938/0.893484530/0.446870515/0.875559926/1.154982849m;
6192 pairs each, actual interval1.004863977–1.004960060s. Full details including rotation are in rpe_summary.csv.
No>2m adjacent state jumps. Whole-run D/E maximum1.108778070m. GT-evaluable maximum D1.029778355/E0.740741132m;
C1.111954817m remains larger, a retained countereffect. Localization error is not an adjacent state jump.
D persistent>2m begins25.913325s; E25.003373s (historical5s definition). C still reaches3.280421686m at34.997773s;
C first>2m34.527837s, no5s persistent crossing before35s is censored evidence, NOT solved tail drift.
The existing C main translation metrics improve frozen P19 Visual1.248108/2.552407/6.247122m;
this supports this-scene accuracy baseline consideration ONLY, not complete robustness/statistical/deployment acceptance.

## Costs and comparability

Mean/P95 ms D: Deskew13.568185/18.282690; NDTalign10.428697/15.680351; NDTcallback21.554520/28.502078;
actual MSCKF including replay18.009379/22.612649. E: Deskew11.900272/16.548728; NDTalign10.877823/16.650942;
NDTcallback22.089953/29.261974. E MSCKF update time unmeasured/count0, not free0ms; DISABLED bookkeeping remains.
D EKF/NDT peak71,077,888/81,457,152bytes, CPU29.07/10.78s; E70,967,296/80,945,152bytes, CPU23.53/11.04s.
Wall170.717856/170.700536s at rosbag rate0.25. RSS per process, not summed as unique system footprint.
Historical C used bounded capture/sparse motion diagnostics, P31 omits them; timing comparisons include this logging difference.
C mathematical path is retained and targeted regression passes, but no new-binary full C/default-off replay is claimed.
P28 offline frontend mean10.214718/P9513.810167ms, wall7.938437s, peak196704KiB; not newly measured online.
No full camera10Hz/100MiB claim. No other scene/robot deployment evaluated; no unique root cause or covariance statistical calibration proved.

## Verification and one NEXT

Unified build/targeted mode test PASS; gate regressions3 PASS; artifact metrics/deltas/historical-values/freeze tests PASS.
One bounded mode/gate review fixed admitted-tail test coverage and gate-only release/NaN checks; no runtime repair/replay.
Final bounded independent raw four-layer/three-window/RPE recomputation agrees within9.44e-13; see INDEPENDENT_REVIEW.md.
Cross-model options were offered; no external CLI run without task-specific exact-command authorization.
NEXT: decision AI first check real code/archive, then decide acceptance of the existing C combined baseline and next contract.
Execution stops here; no autonomous innovation, parameter search, scene extension or new feedback repair.
