# P9-R3A — preregistered metric-depth coverage engineering gate

This is a visual frontend engineering fix, not a DUAL-U innovation or a depth-completion contribution. The frozen 32-frame P9 cohort is now a **DEVELOPMENT COHORT**. No ambiguity AUC, classifier, candidate arbitration, pose switching, estimator update, covariance, new NDT call or support-branch method is permitted.

## Frozen input and unchanged frontend

Start: `f0e8be52a553fe04b2a3e116f8a54b01089dcd3c`, branch `research/p9-r3a-metric-depth-coverage`. Reuse the R3 128-pair manifest and raw input SHA chain; no canonical competitor pose enters this tool. Official Floor01 MEI, Shi-Tomasi 500/.01/10, KLT 21x21/level3/(30,.01), FB<=1px, image/scan mismatch<=20ms, lags1/2/4/8, smallest VALID lag, EPNP RANSAC100/2px/.99, >=30 correspondences, >=20 inliers and LM stay unchanged. PnP returns T_Ccur_Cref; D_vis=T_IC inverse(T_Ccur_Cref) inverse(T_IC).

## DIRECT and conservative fallback

Keep P4's positive camera Z, image bounds, nearest-Z-per-rounded-pixel z-buffer and 2px cKDTree DIRECT lookup exactly. Points remain feature rays multiplied by the historical selected Z, not the LiDAR neighbor's XYZ. DIRECT has precedence and is never replaced. Replay the old frontend to compare DIRECT count, 3D bytes and PnP; compare all recoverable historical R3 pair records and the original12 selected VALID pairs.

Only DIRECT-missing FB features use the SAME z-buffered visible reference-cloud points. Take up to12 nearest projected neighbors inside16px, at least6. Fit PCA covariance with eigenvalues lambda1>=lambda2>=lambda3; require lambda3/sum<=.02 and median absolute plane residual<=max(.05m,.01*median Z). Require Z spread<=max(.50m,.10*median Z). No neighborhood trimming or second trial is allowed. The feature must lie in a nondegenerate neighbor image convex hull, including its boundary; use OpenCV's float32 hull convention. No extrapolation.

Use the unit PCA normal n and the UNNORMALIZED rectified ray r=Krect^-1[u,v,1]. Require |n.r|>=.10. Intersection r*(n.center)/(n.r) must be finite, positive Z and lie within [.9*Zmin,1.1*Zmax]. All parameters are uniform and frozen before extraction; no GT-based change or per-frame tuning. Record every FB feature's DIRECT/PLANE_COMPLETED/MISSING provenance, neighbor IDs/count/radius, planarity, residual, spread, intersection and rejection reason. Return all accepted points in original FB feature order.

## Coverage, geometry safety and stage gates

Freeze pair measurements/provenance before loading development labels or GT. The smallest-valid policy must reach >=6/9 MAJOR and >=14/23 NO_MAJOR. NO_MAJOR is a frozen oracle proxy, not proof of absolute health. Record all six key-frame funnels per lag.

At the original12 R3-valid selected pairs, compare NEW_AUGMENTED with replayed OLD_DIRECT_ONLY. Median translation<=.02m, rotation<=.5deg; P95 translation<=.05m, rotation<=1deg. Losing any of these12 measurements makes geometry safety FAIL. Percentiles use NumPy linear interpolation. Relative pose is inverse(D_old) D_new, at the IMU origin. No gate uses GT.

Only AFTER coverage/regression artifacts and their gates are SHA256 frozen may a separate post-hoc process read GT. Evaluate old-valid pairs and each newly-recovered target's smallest-valid pair at actual image times, with P4's IMU-origin increment convention and no extrapolation. Report translation/rotation median/RMSE/P95/max and missing GT. Newly-recovered translation>.25m is catastrophic; rate must be<=.10. Missing GT must be explicit and cannot be silently dropped to pass a quality gate.

Run full Floor014126 adjacent pairs ONCE with the identical frozen implementation only if development coverage AND geometry safety pass (and no failed post-hoc quality gate). Freeze full measurements before GT scoring; retain all pairs including sync failures. Old coverage=1802/4126 is checked from P4, not redefined. Full coverage>=.60 and image-time IMU translation RMSE<=.03517m are required. Report both all GT-evaluable VALID and historical paired-comparison subsets to expose denominator differences. Rotation is descriptive.

## Final result precedence and cost

DIRECT parity mismatch is an implementation-contract STOP, never a scientific failure. Failed old-valid geometry safety or catastrophic quality gives DEPTH_COMPLETION_GEOMETRY_UNSAFE. Otherwise failed development coverage gives DEPTH_COMPLETION_COVERAGE_INSUFFICIENT. Full generalization failure gives DEPTH_COMPLETION_COHORT_SPECIFIC. All gates passed gives METRIC_VISUAL_COVERAGE_RESTORED, NEXT=HELDOUT_VISUAL_NONLOCAL_EVIDENCE_GATE. Non-success NEXT follows the task's measurement-model/generalization reassessment, never fusion.

Report projection/DIRECT association, neighbor search, plane fitting, completion, PnP and total mean/median/P95, excluding parity replay, bag I/O and GT; include both rectifications conservatively. Completion includes search+fit, so its components must not be added twice. Four-lag and serial smallest-valid early-stop costs are diagnostic estimates, not production benchmarks. Source/input/artifact/binary hashes and stage receipts preserve the gate order. Stable workspace, historical files and unrelated untracked files remain untouched.
