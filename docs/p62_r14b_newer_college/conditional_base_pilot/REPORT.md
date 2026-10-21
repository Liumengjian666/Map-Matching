# P62-R14B-BASE-CONVENTION-CONDITIONAL-PILOT

Result: CONDITIONAL_BASE_PILOT_COMPLETED; NO_NATURAL_RECOVERY_EVENT.
Recovery effectiveness,5degree misrejection and independent physical correctness: NOT_EVALUABLE.

## Identity

Workspace `/home/jian/livox_ws/p62_r14b_newer_college_ws`; branch `research/p62-r14b-newer-college-prior-map`; baseline `c614f5fd1d8837b4c96ec10cc1199e41d5b2418b`.
Actual native CODE_SHA `742bb5f6297d2eebd467db9123a36a744319c081`.
Native binary SHA256 `27d913d505a139b19e23f804f2c05a93099880f0350cbf0ecb44328dcddd3dff`.
Preregistration SHA256 `1b7525585ef49204fcabf75e9c8096d4e43f18be34b982369bfe92ab36e000d1`; original ordinals767–787, anchor767, evaluation768–787.
Archive-only additions follow the CODE_SHA commit; no push executed by this task.
Old R14B blocking archive is unchanged. The original ambiguity is explicitly conditionally admitted, not rewritten as closed.

## Input and map

Reused21 originals from15301 PCDs and existing official Leica map/calibration/reference; no dataset download.
All21 native PCL payload/XYZ checks pass;3846-byte zero suffix excluded; no canonical copy.
Static50m ROI centered on one reference anchor under Base chain;0.20m voxel;224627 map points;SHA `b9f285c27a31f2fb1d4079dc80773285176c50e68c15bcc6ce336a58772a3742`.
Map build3419.641460ms; preflight peak558504KiB; anchor overlap0.937857, not accuracy.
T_M_L=T_M_B*T_B_C*T_C_I*T_I_L, meter/xyzw/inverse checks; file-level integer-ns pairing exact. No deskew.

## Native causal experiment

Exactly20 Full +20 Split +0 Extra =40 native PCL NDT align calls.
Both converge20/20; Full admission20/20; Split admission20/20; Proposed accepts nominal20/20; natural events0, rejected0, replacements0, legal distinct alternatives0, robust support0. Budget0/20 used. Each pool has only nominal0: Native and Held-out Mean select0, Robust returns-1 (no supported replacement), and final state accepts admissible nominal0. This is the expected abstention semantics, not a nontrivial comparison preference. No forced probes.
Common CV prediction depends only on previously admitted Full history. No EKF feedback or online evaluation reference.

CoverageV2 active blocks: B=7 on19 frames and B=6 on1 frame; Training674–731 points. No frame lacked structural input. Maximum Full/Split correction from TP was0.228823m/1.210158degrees, below formal bounds. This quiet short window does not test boundary failures or recovery paths.

## Reference-relative errors

Translation meters; rotation degrees; n=20 for every method. Exact-time reference parsed only AFTER native outputs/decisions were sealed. No fitted alignment.

|Method|n|t RMSE|t Median|t P95|t Max|R RMSE|R Median|R P95|R Max|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|PREDICTION|20|0.127082632|0.100399626|0.174370532|0.350234418|1.999114972|2.011629910|2.722026268|3.363927632|
|CONTROL_FULL|20|0.097666197|0.088399120|0.128057883|0.165390116|1.708573959|1.751214296|2.065261168|2.467996582|
|CONTROL_SPLIT|20|0.095621172|0.086605002|0.138860005|0.158506568|1.706832828|1.729234504|2.083449472|2.517520402|
|PROPOSED|20|0.095621172|0.086605002|0.138860005|0.158506568|1.706832828|1.729234504|2.083449472|2.517520402|

Reference=MAP_ANCHORED_BENCHMARK_REFERENCE, not independent physical GT. Base axes are PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION; CSV exporter binding remains unproved. Full/Split improvements relative to prediction in this window are descriptive; Proposed==Split and recovery untested.

## Costs

Native process wall688.642000ms, launcher including guard/hashing 1.348605s; peakRSS67940KiB. Map construction and formal map loading are separate one-time costs; formal load/grid/tree54.374287ms.

|phase ms|Mean|P95|Max|
|---|---:|---:|---:|
|pcd_read_convert_ms|5.319414|5.460207|5.487610|
|source_preprocess_ms|4.780137|4.899269|4.913999|
|prediction_ms|0.003209|0.003870|0.004400|
|full_align_ms|11.222226|17.693715|38.488910|
|split_align_ms|5.143252|7.771832|7.978378|
|event_ms|0.003105|0.003890|0.006146|
|coarse_grid_ms|0.000000|0.000000|0.000000|
|seed_selection_ms|0.000000|0.000000|0.000000|
|extra_align_ms|0.000000|0.000000|0.000000|
|fitness_aux_ms|2.648733|2.766130|2.802820|
|coverage_ms|1.712325|1.802880|1.841030|
|V3_decision_ms|0.005256|0.008695|0.012013|
|frame_wall_ms|31.056212|40.094824|59.801485|

No10Hz claim. No-event coarse/extra phases are zero measured execution, not a performance validation of the unexecuted recovery path.

## Protection and tests

All21 PCD +4 side-input SHA unchanged; map,33 source/dependency identities and binary unchanged; production snapshot unchanged (tracked/nonignored155 files plus metadata; ignored files not claimed); historical tracked R6–R13 and old R14B archive unchanged. Available89804861440bytes,>=20GiB; new sensor downloads0. See INPUT_IDENTITY_AFTER.json, PRODUCTION_BEFORE/AFTER.json, BUILD_IDENTITY.json, TESTS_AND_REVIEW.md and retained pre-main loader/build failures. C++/Python tests pass; headless read-only ROS visualization smoke passes, GUI not tested.

## Scientific hypotheses

H1: mathematical Base chain PASS only, not file-axis certification.
H2: actual map/PCD inputs PASS.
H3:20 causal native frames PASS.
H4:no natural event.
H5:no recovery candidates, NOT_EVALUABLE.
H6:no supported alternative, NOT_EVALUABLE.
H7:measured descriptive cost only.
Core novelty remains NOT_ESTABLISHED. No independent correctness/ATE/generalization/recovery or5degree-threshold success is claimed.

## Unique NEXT

请科研总控另行预注册同序列、仅依据原始输入运动条件选窗的自然事件覆盖实验；继续保留Base解释和非独立参考限定，不自动执行。

Completed and stopped; noR15 automatically initiated.
