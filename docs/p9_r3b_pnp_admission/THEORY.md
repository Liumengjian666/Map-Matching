# P9-R3B preregistered metric PnP admission realignment

Start15d2d38aae93201aa376c9f640524172e7900312; branch research/p9-r3b-pnp-admission-realignment. This is frontend measurement-availability engineering, not DUAL-U innovation or confirmatory ambiguity evidence. The same32-frame cohort is DEVELOPMENT ONLY. No NDT, ambiguity AUC, candidate switching, estimator/fusion, essential-matrix model or support method is run.

## Admission and immutable quality

The attempt minimum is20 because the already-fixed final P4 quality gate requires at least20 inliers; fewer correspondences cannot satisfy it. This is not a label-derived threshold. The actual P4 pnp() function is reused byte-for-byte: EPNP RANSAC100/2px/.99,20 inliers, LM, finite transform, all accepted inlier camera depths positive. R3A DIRECT/plane completion, feature/KLT/FB/sync/calibration, lags1/2/4/8 and smallest-valid lag are unchanged. No parameter is changed after inspection of results.

## Frozen correspondence replay and isolation

Check the committed R3A measurement-freeze/execution-manifest/artifact chain and frozen numerical environment. Phase1 reads only the128-pair manifest,256-row coverage carriers and frozen per-feature correspondences, not labels, canonical poses or GT. Recover camera XYZ and reference/current pixels in original increasing feature_index order; accepted points retain DIRECT/PLANE_COMPLETED provenance. Their17-digit carriers are round-trip checked and hashed. All eligible20–29 pairs are attempted; all>=30 pairs replay as parity controls; below20 remains insufficient, ineligible pairs remain missing. No new image/depth extraction occurs in development.

Freeze PnP results, low-support diagnostics, selected lags,17-frame original-valid regression, and **unlabelled** availability coverage first. Only then load labels to compute major/no-major coverage and SHA-freeze that separate gate. A labelled coverage tally mathematically cannot precede label access; this split preserves the requested isolation without claiming impossible ordering. Only after both freezes may a separate post-hoc process read GT.

Every original>=30 control must reproduce R3A status, inliers, transform<=1e-10m/1e-8deg and reprojection<=1e-10px. All17 previously available frames must remain available. Preserve original-lag parity separately from any earlier newly VALID lag selected by the unchanged policy. For low-support VALID pairs report current inlier 4x3-grid occupancy, current inlier image convex-hull fraction, and raw median inlier reference/current pixel displacement; these are descriptive ONLY, without added thresholds.

## Development gate and conditional full sequence

Development coverage requires>=6/9 MAJOR AND>=14/23 NO_MAJOR. Frozen labels are evaluation proxies, not evidence input. Post-hoc GT compares IMU increments at actual image timestamps, with D_vis=T_IC inverse(T_Ccur_Cref) inverse(T_IC); no GT interpolation extrapolation or GT-derived filtering. Report R3A-existing-valid at its original lag (and new selected lag separately), R3B newly-available targets' smallest-valid20–29 pairs, and all20–29 newly VALID pairs. The last denominator is pairs, not duplicate independent frames. Catastrophic translation>.25m rate must be<=.10 for the entire newly VALID low-support pair group and separately for newly-available selected targets; no newly VALID pair is excluded on GT grounds. Missing GT is explicit and does not silently pass quality.

Only if coverage, parity/lost-valid and post-hoc quality all pass, run the full4126 adjacent pairs once. Use precisely frozen R3A tracking/depth and the new20 admission; no redesign/tuning. Retain sync failures in the4126 denominator and freeze measurements before GT. Verify historical1802/4126. Full coverage>=.60 and all GT-evaluable VALID image-time IMU translation RMSE<=.03517m are required; rotation is descriptive. Report valid20–29 count/fraction and the historical-valid intersection separately.

## Result precedence and verification

Scientific results: unsafe newly-valid geometry -> LOW_SUPPORT_PNP_GEOMETRY_UNSAFE; otherwise insufficient development coverage -> SPARSE_DEPTH_PNP_COVERAGE_INSUFFICIENT; passed development but failed full -> PNP_ADMISSION_COHORT_SPECIFIC; all gates passed -> METRIC_PNP_COVERAGE_RESTORED. A parity/hash/input violation is an implementation-contract STOP, never an algorithm result. Success NEXT=HELDOUT_VISUAL_NONLOCAL_EVIDENCE_GATE; other results NEXT=TWO_VIEW_VISUAL_INERTIAL_MEASUREMENT_GATE. No further sparse-PnP threshold/depth adjustment is authorized.

Release build/P9 tests, admission19/20/29/30 boundary,>=30 parity, cheirality, information-order tampering tests, CSV/JSON/source/input/numerical-binary hashes and diff checks precede commit. Tests run only in a /tmp build directory. Stable workspace, production core, old archives, .vscode, existing Testing logs and anomalous unrelated untracked files are not modified or staged.
