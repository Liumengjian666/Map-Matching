# P30-R1 scan-local IMU Deskew feedback isolation

FINAL_RESULT: `DESKEW_FEEDBACK_ISOLATION_EFFECTIVE` — scoped to this frozen
Corridor01 trajectory-construction intervention, not a new innovation, unique
root-cause proof, generic safety guarantee or a pure visual-only attribution.

START_SHA `9ed197900419660845e41eebce603606852ae126`.
CODE_SHA `488b93372e09d4ea997ac33ae115e1d4c2302662`.
END_SHA is the final archive commit supplied in chat/Git; no self-referential
commit SHA is invented. PUSH_EXECUTED=false. Existing research branch/P10 clone.
No task writes to the protected robot/paper workspaces or historical archives.

## Implementation and mathematical validity

Default-false `deskew/experimental_scan_local_imu_enable`. One causal
scan_start state p,v,R,ba,bg, then raw IMU continuous ZOH integration, matching
the effective existing P28 nominal model. Existing relative SE(3), gravity,
extrinsic, point times/order, safety gates and OOSM orchestration unchanged.
No later EKF poses used as local nodes; no new filter updates or NDT calls.
The anchor itself can legitimately include earlier visual corrections.
See MATH.md and source_changes.md for the exact model and limitations.

GNU9.4 build with unchanged final `-O2 -g -Wall` passes. Directed math test and
original Deskew contract test pass. Timestamp duplicate/order and future-input
poisoning regressions pass. Default-off source path retained; no default-off
ROS trajectory parity replay was performed or claimed. NDT binary unchanged;
new EKF SHA `f9b24466f68aaf20c7948a9c07a1e7e34e52bb64f81eca04a9d9490056fb7e2a`.

## Exactly two C runs, no A/B rerun

One21s short:208 NDT/converged,207 due NDT OOSM,4198 high-rate,148 actual
MSCKF updates,2 captured Sources (7/193). Numerical/runtime gate PASS.
One40s full:396 NDT/converged,396 OOSM,8004 high-rate,315 actual MSCKF
updates,8 captured Sources. Time series and all other physical parameters agree
with P29 B. Root invalid0, nonfinite0, future IMU0, due transaction losses0.
Existing Multi-NDT now makes646 quick scores,2 extra aligns and1 replacement
vs B656/4/3. Strategy/parameters are unchanged; these are closed-loop outcomes,
not additional experimental switches or a newly proven acceptance mechanism.
Due visual transactions are also checked, including legal rejected updates.
All captured XYZ SHA/stable hashes match their own actual online Source rows.
All raw outputs were hashed/frozen BEFORE GT use. Original caches remain intact.

Short and full first two captured Sources agree. Ordered full pre-filter
point/time correspondence is exact. RMS differences (m):

|frame|old A/B|B/C|A/C|
|---:|---:|---:|---:|
|7|0.000180266|0.001844329|0.001737673|
|193|0.020040835|0.014779702|0.005434352|

Frame7 worsens relative to A — retained counterexample, not suppressed.
Frame193 C is closer to A in geometry, not automatically more accurate in GT.
Post-voxel/max1400 Source membership can differ; nearest-neighbor distances
are separately labeled, NOT claimed to be ordered same-point differences.

On same CURRENT C history, selected8 legacy boundary residual maxima are
3.92e-6–8.83e-6m and0.003841–0.012069deg. Local maxima are8.60e-12–1.12e-10m
and~1e-15deg. This verifies coherent local node construction. The comparison
includes midpoint/ZOH and camera-subdivision differences, not only updates.
It is not reconstruction of P29's internal historical state.

## Fixed Control V2 accuracy, IMU origin

One frozen0–3s full-SE(3) transform for A/B/C, unchanged interpolation, formal
3–35s window. PREFIX-ALIGNED RELATIVE DRIFT, not absolute map ATE.

|EKF window|B RMSE/P95/Max m|C RMSE/P95/Max m|
|---|---|---|
|3–35s|3.104707802 / 8.799511452 / 12.883491212|0.788369339 / 1.107826963 / 3.280421686|
|26–35s|5.782601009 / 11.834723627 / 12.883491212|1.170516696 / 2.045275552 / 3.280421686|
|34–35s|11.756361533 / 12.791086511 / 12.883491212|2.122533129 / 3.128137722 / 3.280421686|

Main translation RMSE/P95/Max decrease74.6073% /87.4104% /74.5378% vs B.
Historical A MSCKF OFF:1.976924726 /4.881597129 /6.485653811m.
Main rotation RMSE/P95/Max B2.819498723 /3.698021032 /14.678118768deg;
C2.670388254 /3.613658670 /13.494033069deg. Rotation risk is not eliminated.

Other main layers, translation RMSE/P95/Max (m),316 paired samples:

|layer|B|C|
|---|---|---|
|Initial|3.011431004 /8.436245258 /12.513619767|0.761565687 /1.090986381 /2.834414950|
|Raw|3.014480770 /8.446812980 /12.506520549|0.758307595 /1.094020368 /2.856688097|
|Used|3.012347547 /8.446238674 /12.506520549|0.757258730 /1.093344763 /2.856688097|

Approximately1s RPE translation RMSE B0.893484530 → C0.446870515m;
P952.590852612 →0.765571224m; Max2.930617040 →2.660903193m.
Actual intervals1.004863977–1.004960060s,6192 paired samples, global-alignment
invariant. No evaluation transform, gate or parameter fitted using GT.

C has95 samples>2m and0>5m vs B1055 /641 in the main6396 samples.
Peak C3.280421686m at+34.997773s, not solved drift. First C>2m at+34.527837s;
no5s persistent crossing is observed, but the35s endpoint truncates follow-up.
Last scan initial2.834415m → Raw/Used2.856688m, one iteration/converged.
Historical NDT initial recursion is unchanged and can still retain error.

No new>2m adjacent-state jumps. Whole replay maximum B1.110958639m,
C1.111954817m. In the GT-evaluable interval, B0.678911003m vs C1.111954817m:
this increase is retained explicitly. Localization error is not a state jump.

## Causal interpretation and limits

The first Source/Raw change is frame1, BEFORE the first visual update;
Initial first changes at2, and EKF first differs at1517157219.339216.
Thus gain belongs to the full Deskew trajectory-construction intervention.
It cannot all be attributed to preventing a particular MSCKF historical
rewrite. The effective ZOH convention also differs from legacy midpoint
Deskew. Subsequent state/Source/initial/Multi-NDT histories are closed-loop.
No same-state optimizer isolation or single-factor submechanism proof is
claimed here. No strict local-minimum/acceptance-rule failure is proved.

## Costs

Mean/P95 ms B→C: Deskew13.529801/18.044460 →11.781156/16.272321;
NDT align10.660290/15.413800 →10.813098/15.889047;
NDT callback22.249141/27.804463 →22.118692/28.270743;
applied MSCKF including replay17.711449/22.898211 →18.098365/23.019316.
C core update-only4.984483/7.417843ms, separately from replay/integration.
Replay wall171.948603s at rate0.25. EKF CPU29.32s, NDT11.08s.
C EKF VmHWM70,553,600bytes; NDT81,121,280bytes. Per-process values,
not their sum claimed as one system footprint. Sparse selected diagnostics
are included in callback timings. Old covariance audit is not enabled.

Inherited offline real feature frontend cost: mean10.214718/P9513.810167ms,
wall7.938437s, peak196704KiB (P28 measurement, not newly measured online).
No full online-camera10Hz/100MiB claim. PSD/root validity is not statistical
consistency; C trace grows1.537846→1609.538534, not the old CI exponential blowup,
but no new claim that covariance calibration is solved.

## Verification, Git and one NEXT

Artifact tests recompute metrics/deltas and preserve frozen outputs; one finite
independent review corrected two interface issues. User-authorized external
CLI ran once, exit0, No substantive issue; source-only scope, no final GT review.
Posthoc timing-field/empty-OFF-series and diagnostic acos-rounding issues were
fixed only in analysis. No runtime rebuild, replay or parameter change afterward.

One NEXT for decision AI approval: freeze a minimal contribution-separation
comparison for ZOH consistency vs historical-node isolation. This is NOT
executed. Keep current default-off mode until research acceptance. Do not
infer new innovation, unique root cause or generic performance from this scene.
