# P29-R1 — fixed Source feedback isolation

FINAL_RESULT: `FIXED_SOURCE_FEEDBACK_ISOLATION_COMPLETE`.
This is mechanism characterization, not an algorithm fix or localization gain.
START_SHA `9b6f32616fae7f408f1950ebba094f1ce72cadc1`; existing research branch.
END_SHA is supplied by the final chat/Git commit. No push is executed without
authorization. P28/older archives and protected workspaces are unchanged.

## Validity

One A and one B capture, same P28 NDT/EKF binaries, frozen healthy-SSD inputs,
all old PnP paths OFF, no runtime math/parameters changed. Both: 396 converged
NDT, 396 NDT OOSM APPLIED, 396 corrected, 8004 high-rate, no nonfinite/Late.
B visual updates 309. Real Source 8/8 in each run, SHA/ordered stable hashes
agree with online diagnostics and original P28. Complete P28 physical parity
passes at 1e-9 m / 1e-8 deg. Source/pose capture uses existing C++ capture;
the passive observer records only eight full pre-filter deskew messages.

Offline 32/32 native aligns, 96 objective values, no new candidate search or
state propagation. Sixteen self-cells reproduce online Raw/Used/fitness and
discrete states EXACTLY (all numeric differences 0). All eight selected frames
have online selected_candidate=-1; no candidate acceptance failure is proved.
The map target is the original 226164-point grid. No GT is used until inputs,
Sources/poses and optimizer outputs are frozen. Frame7 is outside 3–35s and
is only a first-divergence diagnostic. Remaining seven are fixed samples, not
a random dataset or full-trajectory algorithm test.

## Time and feedback

First visual measurement ns=1517157219654659032, replay-relative +0.465680122s.
Its real fixed-lag release is 1517157219.809216 (+0.620237s), also the first
different high-rate state (#125): 0.002715148m / 1.332112504deg. Direct visual
position correction is 0; position differs after IMU replay.
Frame7 scan starts at 1517157219.794140100 and ends about1517157219.894971s;
scan_start is a reference, NOT a processing clock. The visual release occurs
inside this acquisition interval, before its full Deskew processing. Ordered
full point correspondence is verified; frame7 point-delta RMS=.000180266m,
P95=.000296585m, max=.000312536m. Preprocessed Source hash first differs at7;
390/396 hashes differ, first six equal. Same IA=IB but different Source yields
Raw difference .006110443m at IMU origin (.005970161m at LiDAR), .143272987deg.
Initial first differs at8: .012073524m / .143273025deg. First Raw displacement
>=.1m is frame193, evaluation+14.363964s, .128736897m.

The image at1517157219.858260648 within frame7 releases only1517157220.0091519:
it is NOT the already-applied measurement responsible for frame7. Tables keep
image times, real releases and Deskew horizon separately. First/last2ms cohort
rigid fits quantify relative-motion differences and approximation residuals
(roughly .00057–.00270m), NOT exact internal history poses. No P22 reconstruction.

Observed path: visual selected rotation→OOSM/IMU history rewrite→different
Deskew/Source→different NDT Used→previous_pose*delta_pose initialization history.
The frame7 fixed-cell result verifies a direct Source effect; later frames show
large initialization/history effects and nonlinear interaction. This is not
proof that Deskew math is wrong or that this is a unique root cause.

## Fixed-cell Raw displacements (IMU origin, m)

|frame|Source at IA|Source at IB|Initial on SA|Initial on SB|Observed AA→BB|interaction|
|---:|---:|---:|---:|---:|---:|---:|
|7|0.006110443|0.006110443|0.000000000|0.000000000|0.006110443|0.000000000|
|193|0.020715837|0.015149249|0.113812285|0.108103198|0.128736897|0.006161968|
|270|0.005572232|0.006307783|0.366563615|0.360342263|0.363865814|0.006500760|
|320|0.009993977|0.001967353|0.212602101|0.222822215|0.213307222|0.010435796|
|340|0.001229762|0.004266143|0.401492214|0.398649140|0.398230449|0.003539743|
|347|0.006533214|0.005076838|0.530960785|0.540667657|0.534215596|0.010134815|
|365|0.149990455|0.141423142|1.033236151|1.020185032|1.102985024|0.239025825|
|396|0.005752539|0.002467064|6.166841081|6.164143846|6.169228720|0.004087675|

## Raw translation errors (fixed Control V2, IMU origin, m)

|frame|SA,IA|SA,IB|SB,IA|SB,IB|
|---:|---:|---:|---:|---:|
|7|0.077381210|0.077381210|0.077938618|0.077938618|
|193|0.372066614|0.396001012|0.374833583|0.400280254|
|270|0.635439586|0.674448152|0.636676273|0.676870141|
|320|1.206562582|1.063000610|1.215141190|1.064000725|
|340|2.031580680|1.678161993|2.031611769|1.679530553|
|347|2.638044905|2.146114995|2.643788652|2.141585097|
|365|4.092360671|4.822996839|4.078120957|4.952323704|
|396|6.380050760|12.504104109|6.384636559|12.506520549|

IA/IB are the two captured full SE(3) priors, not translation-only replacements.
Values above are vector-displacement magnitudes, not additive error attribution
percentages. Cross-source fitness/objective values are NOT comparable as one
objective. All exact Initial/Raw/Used matrices and per-cell metrics are archived.

Frame365: on SAME SA, IA vs IB produces20 vs4 iterations, fitness .050393765
vs2.497458025, terminal objective2126.918637 vs716.142979, Raw GT error4.092361
vs4.822997m; rotation GT error1.595158 vs12.175152deg. Both claim convergence.
On SAME SB, 19 vs6 iterations, fitness .048823749 vs2.062735939, objective
2167.693003 vs717.083737, error4.078121 vs4.952324m. This proves initialization
sensitivity and different inferior/superior converged solutions, NOT strict
mathematical local minima (stationarity/curvature not tested).

Frame396: with SAME SA, IB has higher terminal objective2351.832864 vs
2214.075695 and better fitness .043409200 vs .045240002, but GT error12.504104
vs6.380051m. A real same-source objective/GT disagreement; one iteration each.
Changing Source at fixed Initial affects Raw by only.002467–.005753m, whereas
changing Initial at fixed Source affects it by6.164144–6.166841m.

Step Limit is separately history-dependent: for SB,IA at396 Raw error6.384637m
becomes Used11.534176m under PB; SA,IB Raw12.504104 becomes Used6.572631 under
PA. Source-conditioned PA/PB contexts are fixed within each source's pair;
cross-source Used is NOT a pure source effect. At365 rotation limiting also
changes a Raw1.595deg rotation error to Used10.658deg. These observations do
not prove Step Limit uniquely causes full drift.

## Alignment / formal accuracy

P28 actual evaluator uses ONE frozen Control V2 transform for both A/B.
Formal A/B transform differences: 0m,0deg. No independent-fit amplification
explains these formal metrics. Auxiliary independent prefix fits differ only
.012946735deg / .000603520m; B34–35 RMSE changes11.756361533→11.751264207m,
only .005097325m, not the multi-meter regression. Shared A-prefix auxiliary B
tail RMSE is11.753230089m. Formal errors/individual fits are kept separate.

EKF3–35: A RMSE/P95/Max1.976924726/4.881597129/6.485653811m;
B3.104707802/8.799511452/12.883491212m. 34–35:
A6.187017888/6.466967562/6.485653811, B11.756361533/12.791086511/12.883491212m.
All reproduce P28. Frame347 is evaluation+29.895428s (NOT34–35), LiDAR raw
AB=.534224328m; frame396 is+34.837263s, raw AB=6.169278184m at LiDAR and
6.169228720m at IMU. The premise that all34–35 Raw differences are only
.4–.6m is not supported by these frozen data.

Nominal1s RPE: actual accepted intervals1.004863977–1.004960060s,6192 paired
samples. Translation RMSE A=.456187938m, B=.893484530m; P95 A=1.026413772,
B=2.590852612m. It is independent of a global SE(3) alignment and likewise
shows regression. No evaluation transform, primary window or parameter is tuned.

## Cost, limits and one NEXT

Actual per-cell align timings, cold first call, whole-process wall/RSS and
capture CPU/RSS are preserved in runtime/resource tables. No online visual
frontend is measured here. No 10Hz or system-memory claim.

An independent read-only review found mixed LiDAR/IMU effect origins. This
was actionable and corrected in derived analysis only; a nonzero-lever-arm
rotation regression test was added. P28/capture/native output freezes remain
unchanged. RPE actual intervals and release-vs-measurement caveats are explicit.
External CLI review, if authorized/completed, is recorded separately.

One NEXT for decision AI approval: scan-local continuous-motion Deskew feedback
isolation. Target processReadyImuDeskewCloudsLocked / processMsckfLocked history
interface: test whether fixing a legal scan-start propagation anchor prevents
intra-scan delayed state rewrites from changing the physical relative-motion
trajectory. Common-left pose correction must cancel in T_ref^-1*T_point;
time-varying or inconsistent corrections need not. Preserve SKF measurement,
NDT/DCReg/Multi-NDT/OOSM parameters; no new acceptance rule or noise search.
This proposed experiment is NOT executed, and is not a proven repair.

Raw Sources, deskew bags, large outputs and build files remain local in the
independent .p29 cache. Git CSV copies normalize only CRLF→LF; original cache
bytes remain frozen, with separate archive hashes. No Source/map/bag is committed.
