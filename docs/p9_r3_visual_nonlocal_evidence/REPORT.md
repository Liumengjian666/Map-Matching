# P9-R3 — fixed multi-lag map-independent visual-motion evidence gate

FINAL_RESULT = `VISUAL_NONLOCAL_EVIDENCE_COVERAGE_INSUFFICIENT`

NEXT = `VISUAL_FRONTEND_COVERAGE_REDESIGN`

The fixed four-lag P4 frontend produced no additional available frame beyond the adjacent archive. Smallest-valid availability remains 2/9 MAJOR (22.22%) and 10/23 NO_MAJOR (43.48%), below the preregistered 6/9 and 14/23 gates. Classification, post-hoc GT and complementarity are deliberately **NOT_RUN**, not zero-valued failed metrics. This is a coverage result, not a refutation of visual nonlocal evidence or DUAL-U. R2B's formal failure and auxiliary LiDAR-side evidence remain unchanged.

## Git and scope

- Branch: `research/p9-r3-visual-nonlocal-evidence`.
- START_SHA: `4957015ce21c5c9071f17746c172f6fe13a9bde6`.
- Contract commit: `3b723388d2d99079365e8c7e7ebe7f9ec5316727`.
- Evaluation-preparation commit used for extraction: `86b91df1fcfedd1f51881c4b95892a297e144243`.
- Final archive commit is resolved through Git history; it is not embedded recursively inside its own files.
- Workspace: `/home/jian/livox_ws/dog_loc_paper_ws`. Stable workspace and production localization core untouched. Original unrelated untracked files preserved.
- NEW_NDT_CALLS = 0; GT_USED_FOR_EVIDENCE = NO; GT_LOADED_THIS_RUN = NO; no pose switch, estimator update, covariance or fusion.

This is **MAP-INDEPENDENT VISUAL-MOTION EVIDENCE**. Reference LiDAR depth is used, so it is neither sensor-independent vision nor pure monocular evidence. Only existing R2B B12 non-oracle representatives are considered. No canonical basin pose/ID is loaded by evidence construction.

## Historical audit and extraction

All 32 targets match P4 `transaction_cur` uniquely. Historical statuses: VALID=12, INSUFFICIENT_CORRESPONDENCES=18, SYNC_INVALID=2; group parity is 2/9 and 10/23. The label-free aggregate audit precedes extraction; grouped parity is checked only after the evidence SHA256 freeze. Camera input is the P4 canonical three-shard Floor01 stream, `/cmu_sp1/camera_1/image_raw`, frame `d`, bgr8/640×480, with actual image headers validated against frozen sync indices/stamps. Request scan-end stamps and LiDAR/map frames are also checked.

Unchanged P4 rules: official MEI rectification; Shi-Tomasi 500/.01/10; KLT 21×21, level3, criteria=(30,.01); FB<=1px; projected reference LiDAR depth<=2px; EPNP RANSAC 100/2px/.99; >=30 correspondences, >=20 inliers; LM refinement. Single OpenCV thread, deterministic seed=current transaction. Every prescribed lag is archived; invalid synchronization is never forced through PnP.

| Policy | VALID /32 | MAJOR /9 | NO_MAJOR /23 | Insufficient correspondences | Sync invalid |
|---|---:|---:|---:|---:|---:|
| lag1 | 12 | 2 | 10 | 18 | 2 |
| lag2 | 11 | 2 | 9 | 18 | 3 |
| lag4 | 11 | 2 | 9 | 18 | 3 |
| lag8 | 6 | 2 | 4 | 23 | 3 |
| smallest VALID | 12 | 2 | 10 | 18 unavailable | 2 unavailable |

128 pair records; 117 frontend attempts; 40 VALID pairs; 77 insufficient-correspondence pairs; 11 sync-invalid pairs; no PNP_REJECT. All 12 selected measurements use lag1. Longer lags repeat a subset of already-valid frames, rather than rescue an unavailable target. The 20 unavailable targets consist of 18 with no lag reaching 30 correspondences and two (1854,3796) with invalid target synchronization at all four lags. No threshold, lag or input cohort was changed.

## Pose and candidate contract

`T_Ccur_Cref` maps reference camera into current camera. `D_vis=T_IC inverse(T_Ccur_Cref) inverse(T_IC)=inverse(T_map_imu_ref) T_map_imu_cur`. Nonidentity synthetic camera/IMU/LiDAR tests pass. NDT poses are map_T_lidar; conversion to IMU is `map_T_lidar inverse(T_IL)` with original P4 raw calibration semantics.

Reference states are existing R1 same-objective raw nominal states at t−lag, never GT. The earlier R2A SHA256 of R1 `dual_u.csv` pins all used raw xyz/quaternion values and stamps; registration and pinned archive ID sets agree. The registration file is additionally frozen prospectively. All 32 target matrices match R2A T0: max translation 0m; max rotation 6.184854083457503e−6deg. 124 distinct lag references exist, of which120 are reference-only and4 also belong to the target cohort. 152 selected request clouds and 146 selected camera images have byte hashes archived.

R2B B12 supplies 25 inherited competitive representatives. Strict center separation (>0.2m OR >2deg) retains 18 and excludes 7 inside-center representatives; excluded representatives remain in the audit. Score competitiveness is inherited with EPS_SCORE=2.747604276e−4, never converted to probability or used to switch pose.

For each available visual pair, `E_k=inverse(D_vis) inverse(T_ref_IMU) T_k_IMU`; primary `r_t=||translation(E_k)||` meters. `G_visual=r_nom-min(r_alt)` and `U_visual=max(0,G_visual)`. Rotation is reported separately. Available/no-alternative has U=0 and absent r_alt. Unavailable visual has **blank residuals and evidence**, never U=0. Local cluster names below are non-oracle R2B names, not canonical IDs.

## Complete frame table

`C` is the strict-separated competitive candidate count, excluding nominal. `—` is unavailable/undefined, not zero. Inliers and reprojection are for the selected smallest-valid lag. Residuals, margins and U are meters; reprojection is pixels.

| TX | Proxy label | Available | Lag | Inliers | Reproj | C | r_nom | r_alt | G | U |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|120|NO_MAJOR|yes|1|41|.022114|1|.007261|.995047|−.987786|0|
|244|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|368|MAJOR|no|—|—|—|1|—|—|—|—|
|616|MAJOR|no|—|—|—|4|—|—|—|—|
|740|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|838|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|839|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|864|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|924|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|925|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|1111|NO_MAJOR|yes|1|60|.539385|0|.031930|—|0|0|
|1235|NO_MAJOR|yes|1|30|.412665|0|.022259|—|0|0|
|1359|NO_MAJOR|yes|1|54|.796789|0|.025132|—|0|0|
|1497|NO_MAJOR|yes|1|65|.785459|0|.038697|—|0|0|
|1498|NO_MAJOR|yes|1|68|.708091|0|.027202|—|0|0|
|1556|NO_MAJOR|yes|1|43|.560297|0|.099333|—|0|0|
|1557|NO_MAJOR|yes|1|51|.660595|0|.041318|—|0|0|
|1606|NO_MAJOR|yes|1|69|.479800|0|.033149|—|0|0|
|1730|NO_MAJOR|yes|1|85|.505586|0|.015106|—|0|0|
|1854|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|2102|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|2226|MAJOR|no|—|—|—|1|—|—|—|—|
|2350|MAJOR|no|—|—|—|7|—|—|—|—|
|2598|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|2722|MAJOR|yes|1|92|.471176|0|.010612|—|0|0|
|2846|MAJOR|no|—|—|—|0|—|—|—|—|
|3094|NO_MAJOR|no|—|—|—|3|—|—|—|—|
|3217|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|3341|MAJOR|no|—|—|—|0|—|—|—|—|
|3631|NO_MAJOR|no|—|—|—|0|—|—|—|—|
|3796|MAJOR|no|—|—|—|0|—|—|—|—|
|3962|MAJOR|yes|1|50|.420012|1|.009310|1.020949|−1.011639|0|

NO_MAJOR is only the frozen old-oracle evaluation proxy, not proof of absence of any optimizer ambiguity. The available subset must not be treated as a representative 32-frame classifier cohort.

## Key cases and risk controls

| TX | Correspondences lag1/2/4/8 | Measurement | Strict candidates | Interpretation |
|---|---|---|---:|---|
|616|23/25/0/0|unavailable|4|No lag passes the unchanged 30-correspondence gate.|
|2226|11/7/3/0|unavailable|1|Cannot arbitrate nominal versus competing terminal.|
|2350|21/13/15/10|unavailable|7|Strong LiDAR-side candidate evidence remains, but no usable visual measurement.|
|2722|92/66/56/35|lag1 valid|0|Inherited representative is inside center and correctly excluded.|
|3341|9/11/12/11|unavailable|0|FB tracking is plentiful at short lag, but associated depth correspondences remain below30.|
|3796|sync-invalid at all lags|unavailable|0|Invalid target synchronization cannot be rescued by reference lag selection.|
|3962|51/46/43/39|lag1 valid|1|Nominal r_t=.009310m; alternative r_t=1.020949m. Separation=.978362m/87.867677deg; G<0, U=0.|

120: lag1 VALID,41 inliers,.022114px; nominal r_t=.007261m versus C003=.995047m, separation=.938220m/86.389070deg, G=−.987786m, U=0. This is lack of visual translation preference for that alternative, not a GT correctness claim or a safety guarantee.

3094: lag1/2/4 correspondences4/4/20; lag8 sync-invalid. Three strict alternatives remain, but visual evidence is missing, not zero. Both historical risk controls are retained; no new GT was read in this run.

## Primary statistics / post-hoc boundary

Available MAJOR N=2; available NO_MAJOR N=10. AUC, permutation p, LOFO balanced accuracy and delete-one-major AUC: **NOT_RUN — COVERAGE_GATE_FAIL**. The predeclared PCG64 seed20261012/10000-replicate test is implemented and synthetic-tested, but no real label permutations were performed. `lofo.csv`, `delete_one_major.csv`, `permutation_manifest.csv` and `posthoc_gt.csv` are header-only receipts, not missing attempted experiments.

POSTHOC_GT improved/same/worse = NOT_RUN. SECONDARY_COMPLEMENTARITY = NOT_RUN. No quality classifier, rotation weighting, threshold tuning, additional lag, candidate selection policy or fusion was introduced.

## Cost

Component means below average over all32 targets (non-attempted pairs contribute0). Units ms/pair; total includes both rectifications conservatively. Projection/association are components of depth, not extra terms added again to total.

| Lag | Attempts | Feature | KLT | Projection | Association | PnP | Both rectifications | Total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|1|30|3.8880|8.0418|2.2245|1.4487|.3378|3.0146|18.9979|
|2|29|3.6940|9.7902|2.1316|1.3885|.3661|3.1137|20.5225|
|4|29|3.6637|11.6520|2.1053|1.3706|.4217|2.8979|22.1480|
|8|29|3.6683|13.2701|2.1614|1.3315|.2717|3.0739|23.8144|

Four-lag component-total sum=2735.448646ms, mean85.482770ms/frame. Descriptive serial smallest-valid early-stop estimate=50.338114ms/frame; it was not executed as an optimization. Extraction wall time95.566555s includes input hashing/bag I/O; selected cloud/image I/O48.122211s. These are not production timing claims.

## Information freeze, verification and review

Before label loading:

- `visual_measurements.csv` SHA256: `da01ea8eef596e6d2bd9c3c362cc4b15bb65ab311ca9a7161ce38f1e518c7b1f`.
- `visual_candidate_residuals.csv`: `9cd4f5d4a5fc3a45e149667e34030858f7d21e729744818a766f0bdef09b164d`.
- `visual_nonoracle_evidence.csv`: `7af90c7aeaaf6e1601a185ac2eea1344536d2965f48bcda012d7f019bfcb0027`.
- Frozen four-lag measurements: `40f53e246f74d5222a0a7b1ca32589022d9e4e601a62c9704f5286d5a9f4f00a`.

Release build and28/28 P9 tests pass. P4 PnP direction/MEI sanity error1.45904016849e−6; nonidentity IMU/LiDAR transform, smallest-valid selection, missing-versus-empty set and frame-statistical gate self-tests pass. CSV/JSON are recomputed, frozen-source/data SHA chains checked, and raw inputs rehashed before final archive verification. `verification.json` and `artifact_hashes.json` retain the audit receipts and binary hashes.

Fresh-context read-only reviews covered transform/provenance and statistical gates. Actionable findings fixed before extraction: missing/empty-set tests, camera-frame identity, and reference-only nominal provenance/transaction-ID guards. A two-frame test fixture conflicted with the frozen32-frame guard; the fixture was corrected without relaxing the guard. Per user instruction, no external CLI second opinion was run. Git/incremental/doubt/debugging/review skills influenced source safeguards, bounded review and preservation of the preregistration, not experimental parameters.

Only next research action: `VISUAL_FRONTEND_COVERAGE_REDESIGN`. No fusion is authorized by this coverage result.
