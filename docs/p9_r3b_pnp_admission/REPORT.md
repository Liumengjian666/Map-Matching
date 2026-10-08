# P9-R3B metric PnP admission realignment

FINAL_RESULT = METRIC_PNP_COVERAGE_RESTORED

NEXT = HELDOUT_VISUAL_NONLOCAL_EVIDENCE_GATE

This closes frontend metric-measurement availability, not nonlocal ambiguity discrimination, pose selection, or fusion. DUAL-U remains within-basin directional observability plus inter-basin/competing-attractor ambiguity. The 32-frame cohort is DEVELOPMENT ONLY; no ambiguity AUC was computed.

## Git and scope

Branch: research/p9-r3b-pnp-admission-realignment. Start/parent/current precommit HEAD: 15d2d38aae93201aa376c9f640524172e7900312. Worktree: /home/jian/livox_ws/dog_loc_paper_ws. This archive is a precommit snapshot: resolve its containing commit/end SHA with Git, never infer it from the scientific result. No push performed. Stable workspace, production core, estimator/fusion, old archives and unrelated untracked files were not edited.

Development NEW_IMAGE_EXTRACTION=0, NEW_DEPTH_COMPLETION=0, NEW_NDT_CALLS=0. The conditional full-sequence run necessarily extracts images/depth once, AFTER the development gates pass; it is not included in those development-zero counters. GT is used only for post-hoc image-time IMU increment fidelity.

## Immutable measurement contract

Only the pre-PnP admission minimum changes: 30 -> 20. The reason is the existing final requirement of >=20 inliers; below20 cannot satisfy it. Final acceptance remains the identical P4 EPNP RANSAC100/2px/.99, >=20 inliers, LM, finite transform and positive inlier cheirality. Feature/KLT/FB/MEI/sync20ms, R3A DIRECT/plane completion parameters, lag1/2/4/8 and smallest-valid-lag selection are unchanged. There is no new quality threshold, label/GT tuning, essential-matrix model, NDT call, or candidate arbitration.

Development uses committed R3A frozen reference camera XYZ/current pixels, in increasing feature_index order, with DIRECT/PLANE_COMPLETED provenance. Float32 pixel carriers and double XYZ round-trip and combined byte hashes are audited. The reused P4 call receives the same canonical float64 pixel array as R3A. Development does not reextract images or recompute depth.

## Stage isolation and provenance

1. execution_manifest.json freezes R3A input/source lineage, calibration, reused numerical binaries, new construction sources and THEORY before replay.
2. measurement_freeze.json seals all128 PnP pairs, diagnostics, selected lags, regression and unlabelled availability. Neither labels nor GT is loaded in replay.
3. coverage_freeze.json binds the committed label SHA and coverage CSV; labels enter only here. Its gate is recomputed from frozen availability, not trusted as a flag.
4. posthoc_quality.json binds GT SHA, all post-hoc group rows and the development gate, AFTER both earlier freezes. The pair denominators are recomputed and metadata tampering is rejected.
5. The full run uses an exclusively created once-only marker and rejects existing full outputs. Full measurements are SHA-frozen before their GT scoring. All raw inputs are rehashed before and after the one run.

Key hashes: R3B plan ed210d391f51806ab02279cb7d2c88d2165c5d555e5b061b722b6b7f1e55c77a; development admission CSV 7987ca741f43cbf453b531f6b54eb2d9786e35082c96abf606c0c43c5fb87d65; full measurements 96a6d3de0791399e8eb572c17ea9e0fbfc82ab6b2184811fc30bc19c740427af; GT b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f. Complete input/code/library/binary/artifact maps are in artifact_hashes.json and the frozen manifests.

## Development parity and coverage

>=30 controls: 48/48 PASS, all status and inlier counts identical. Maximum pose translation difference0m, rotation difference5.62247946e-16deg; reprojection difference0px. Original17 R3A available frames retain their original VALID lag and remain available: lost_valid=0. None changes selected lag. This is exact augmented30-vs-augmented20 parity, not the separate historical DIRECT-only comparison.

| Frontend | MAJOR | NO_MAJOR | All |
|---|---:|---:|---:|
| R3A augmented/admission30 | 4/9 | 13/23 | 17/32 |
| R3B augmented/admission20 | 6/9 | 17/23 | 23/32 |

Development coverage PASS. Newly available targets: 244,368,925,2226,3094,3217. Of18 eligible20–29 pairs, 11 become VALID and7 remain PNP_REJECT. They are not treated as independent frames.

## All low-support PnP pairs

Occupancy and hull are current inlier image measures; parallax is median raw inlier pixel displacement, descriptive only. Rejected pairs have no accepted pose/reprojection/distribution; blanks are missing, not zero.

| Frame | Lag | Corr | Inliers | Ratio | Reproj px | Grid fraction | Hull fraction | Parallax px | Status |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
|244|2|27|27|1.0000|0.4510|0.4167|0.22958|4.066|VALID|
|368|1|28|25|0.8929|0.7654|0.5833|0.16357|10.218|VALID|
|368|2|27|16|0.5926|||||PNP_REJECT|
|616|2|28|27|0.9643|0.5923|0.3333|0.13769|4.741|VALID|
|925|1|29|28|0.9655|0.4863|0.4167|0.14967|4.013|VALID|
|925|2|28|27|0.9643|0.6866|0.3333|0.12641|3.174|VALID|
|925|4|22|18|0.8182|||||PNP_REJECT|
|1111|8|22|14|0.6364|||||PNP_REJECT|
|1235|8|22|16|0.7273|||||PNP_REJECT|
|1556|8|25|20|0.8000|1.1030|0.2500|0.05621|92.524|VALID|
|1557|8|28|22|0.7857|0.7558|0.3333|0.06016|91.153|VALID|
|2226|1|23|21|0.9130|0.5785|0.3333|0.12410|13.611|VALID|
|2350|2|24|22|0.9167|0.6835|0.2500|0.06023|5.793|VALID|
|2350|4|23|19|0.8261|||||PNP_REJECT|
|2350|8|20|16|0.8000|||||PNP_REJECT|
|3094|4|28|21|0.7500|0.5254|0.3333|0.12750|48.815|VALID|
|3217|1|27|26|0.9630|0.4681|0.1667|0.06235|14.845|VALID|
|3631|1|21|18|0.8571|||||PNP_REJECT|

Key outcomes: 368 passes with25/28 atlag1;2226 passes21/23 atlag1. 244 passes27/27 atlag2;925 passes28/29 atlag1;3094 passes21/28 atlag4;3217 passes26/27 atlag1.3631 gets a PnP attempt but remains rejected18/21.2846 and3341 remain below20 at every lag;3796/1854 remain sync-invalid. No fallback retuning or sync relaxation occurs.

## Post-hoc development geometry

All groups are evaluated at actual image timestamps, D_vis=T_IC inverse(T_Ccur_Cref) inverse(T_IC), against relative IMU GT without extrapolation. No GT record is dropped or used to alter admission, correspondence set or selected lag.

| Group | Pairs | Translation median/RMSE/P95/max m | Rotation RMSE deg | Catastrophic>.25m |
|---|---:|---|---:|---:|
| R3A existing-valid original lag |17|.024409/.028410/.043799/.070432|.883814|0/17|
| R3A existing-valid new selected lag |17|same as original|.883814|0/17|
| Newly available selected targets |6|.014418/.016301/.023292/.023813|.880510|0/6|
| ALL newly VALID20–29 pairs |11|.023813/.140645/.265529/.452197|.974360|1/11=9.09%|

Quality PASS under the preregistered<=10% catastrophic rule for BOTH new groups, with missing GT0. The one bad pair is1556/lag8, translation error.452196933m: preserved in the all-pair denominator, not hidden by the smallest-valid policy. Its target already has an unaffected VALID lag1. This outlier remains a frontend safety limitation; the admission contract does not guarantee every20-inlier solve is accurate.

## Full Floor01 once-only generalization

Development gates pass, so the frozen completion+admission20 frontend runs ONCE on all4126 adjacent pairs. All375 sync-ineligible pairs remain in the coverage denominator. Full measurements freeze before GT. No parameters change, no failed pair is retried.

| Metric | Historical DIRECT30 | New frozen completion+admission20 |
|---|---:|---:|
| Coverage |1802/4126=43.67%|2715/4126=65.802230%|
| Image-time IMU translation RMSE m |.029304|.034584818149|
| Rotation RMSE deg |historical descriptive|.848582598|
| VALID20–29 pairs |pre-gate blocked|278/2715=10.239411%|

Full coverage PASS>=60%; translation PASS<=.03517m, but only narrowly: deterioration versus historical RMSE is approximately18.02% (20% allowed). This comparison includes R3A completion AND R3B admission; the entire913 increase is not attributed solely to admission realignment.

All2715 VALID pairs have GT, translation median.013620m/P95.057954m/max.610625m; catastrophic4/2715=.14733%. The278 low-support adjacent VALID pairs have translation RMSE.024243m/P95.043990m/max.120455m and catastrophic0/278. No full-run GT-based filtering is used.

Historical VALID retention is1801/1802, separate from development17/17 parity. TX3351 changes from historical DIRECT47corr/21inliers VALID to augmented69corr/19inliers PNP_REJECT. Completion changes the correspondence set; both counts exceed30 so this is not an admission-gate difference. No causal attribution beyond that is claimed, and no rerun or exception repairs it. Historical-valid intersection translation RMSE.03120159m is descriptive only; ALL2715 is the primary gate denominator.

## Cost

Full wall time193.384439s (streaming/extraction included, pre/post whole-input hashing excluded). For3751 sync-eligible attempted pairs, mean/median/P95 ms:

| Component | Mean | Median | P95 |
|---|---:|---:|---:|
| Feature |4.3861|3.9510|6.7585|
| KLT |8.9933|7.7716|19.1164|
| Projection |2.8738|2.8308|3.1896|
| DIRECT association |1.5408|1.4929|1.7986|
| Completion (search+fit) |14.4421|12.6822|32.2106|
| PnP |.6656|.4959|2.6142|
| Frontend |33.2768|31.4172|56.6877|
| Total with rectification |37.4122|35.6183|60.9207|

Development smallest-valid serial early-stop mean48.7700ms/median51.5078ms/P95 86.1329ms; four-lag mean124.9562ms. These are explicitly ESTIMATES using historical R3A shared-stage times plus current replay PnP time, not new development extraction timings or a production realtime claim. Component times may overlap and must not all be summed independently.

## Verification and decision boundary

Release build PASS; P9 tests33/33 PASS. Admission19/20/29/30 boundary, cheirality, >=30 canonical pixel/pose parity, original17-frame retention, fixed source/input/actual-loaded numerical binaries, GT/label stage-order and gate-metadata tampering, once-only full run, archive gate/final-quality self-tests PASS. Independent read-only frontend, stage-lineage and archive reviews closed actionable findings before the relevant artifacts stood; no external CLI. CSV/JSON hashes/gates, all raw inputs, source/test binaries, staged/tracked scope and whitespace are audited. Tests only run under /tmp.

The result authorizes a new HELD-OUT visual nonlocal evidence task only. It does NOT authorize development-cohort ambiguity AUC, candidate switching, visual state correction, EKF, covariance fusion, or final DUAL-U formulation. No new search or NDT experiment occurred.
