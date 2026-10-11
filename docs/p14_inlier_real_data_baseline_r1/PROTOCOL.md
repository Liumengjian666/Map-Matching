# P14 frozen author-core baseline

TASK: P14-INLIER-REAL-DATA-BASELINE-R1.
START_SHA: f3a996da317a08bad0886f14dc499c0d24031ce6.

Before real encoding/query results: use author InLiER
515759c00fe1e2f6d4d4a005e8c3a6b3f71d17ac, MIT, unchanged external C++17
`inlier_core`. No Python backend fallback; no ROS / NDT / EKF / OOSM changes.

Use the author's shipped `inlier/config/default.yaml`, not its different
library dataclass defaults. Encoder settings remain untouched. Ground RANSAC
250 iterations / 1m threshold / 200 minimum inliers; shape PCA radius1.5m /
minimum8 neighbours; keypoints maximum256 per slice /1280 total. Input adapter
casts to float32, drops all-zero and nonfinite points, keeps the first point in
each0.5m voxel in original order. Test against actual author `prepare_points`.
No NDT source cap, no added range filter or new deskew.

MINT uses compact L1 intersection,3 shared height rows, top10% (39/389), with
negative call overrides so the configured percentage is actually honored.
BEAM top20,4 spatial bins,3 azimuth columns, threshold0. Verify all top20,
500 RANSAC iterations,1m distance,32 minimum correspondences,16 RANSAC inliers,
8 keypoint inliers, spatial tolerance0. Seed0 is the author C++ default and
the author's reproducible unit-test policy. OpenMP fixed to1 thread. Rerank
disabled as in author YAML. Optional later GICP NOT_RUN, separately recorded;
this is the complete Encoder/MINT/BEAM/core-Verify chain, not the GICP-refined
full-cloud result.

Reuse and stream389 real Loop2 mapping reference scans and24 P12-frozen Loop3
queries. Build reference token/keypoint DB once. Retain keypoints/tokens only,
not all raw reference clouds online. First-six IDs fixed before results:
16,45,124,708,1285,1891. Remaining18 retain their P12 membership and ascending
ID order. If none of the first six has a nonempty encoding and positive MINT
and BEAM candidate, stop rather than pretending zero-score padded IDs are
recognition evidence. Otherwise process the remainder without changing rules.
No sample replacement or descriptor tuning. Raw single scans first; multiframe
accumulation NOT_RUN unless separately justified by the permitted sparse-input
condition. Existing samples were development-selected P12 strata, not a fresh
independent/generalization set.

`Verify.T_sensor` maps query sensor points to reference sensor points:
`p_reference = T_sensor * p_query`. No guessed reference-state/extrinsic
conversion to map_T_lidar. Ground alignment remains the author's algorithm.
Failure outputs carry default identity: they are NOT valid pose estimates.
Count valid relative poses only for successful finite rigid Verify outputs.
NaN/Inf in successful outputs is an error; failure RMSE=Inf is the author's
failure sentinel and is logged explicitly, not treated as finite success.

Measure PCD I/O, preprocessing, encoding, MINT, BEAM, summed top20 Verify,
core compute total, actual wall end-to-end (including I/O), CPU and RSS.
Persist actual per-query and per-candidate times. DB serialized bytes and
resident-RSS delta are distinct measurements, neither is total peak RSS.
Keep all cache/binary/large point data in the persistent external directory
`/home/jian/rosbag/p14_inlier_r1_20261011/` (ext4).

Loop3 has no independent map-frame correctness labels. GT_LOADED=NO;
ACCURACY_GAIN_NOT_EVALUATED. Positive retrieval and geometric verification
are functional evidence, not place correctness or localization success rates.
P12/P12B430–455ms top3 DCReg is a reference only: full-map point-plane
registration, different point preprocessing and candidate pools. No strict
speedup factor or proven replacement claim. Stop after this bounded baseline.
