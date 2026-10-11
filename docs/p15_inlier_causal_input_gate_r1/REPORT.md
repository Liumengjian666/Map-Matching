# P15 — causal multi-scan input gate

TASK: P15-INLIER-CAUSAL-INPUT-GATE-R1.
FINAL_RESULT: **INLIER_CAUSAL_ACCUMULATION_GEOMETRY_OBSERVED**.
NEXT (one only): **DECISION_REVIEW_SUCCESSFUL_INLIER_CASES**.

The fixed six-query experiment is complete and stopped. N5 and N10 each
produce one successful author-core Verify at TX4691 / reference308. No
parameter changes, additional queries, GICP, ROS replay or filter feedback.
This is limited functional geometry evidence, NOT a localization-accuracy gain
or exact-calibration certificate. ACCURACY_GAIN_NOT_EVALUATED; GT_LOADED=NO.

## Git and code

Branch: research/p9-r4-heldout-visual-evidence.
START_SHA / verified REMOTE_HEAD: 01fee6a8d1743cb86b236e872ee5452993e2a9f2.
Compiled/runtime CODE_SHA: a3f6e93b11694e0a3d754b25b638d005bc9396b0.
END_SHA is the subsequent archive commit reported at handoff; no circular hash.
PUSH_EXECUTED=NO. Safe workspace /home/jian/livox_ws/p10_corridor01_oosm_r1_ws.
The dirty main paper workspace and stable robot workspace are not modified.

Only own isolated P15 input/orchestration/statistics files are added. P14 files,
NDT/EKF/OOSM/DCReg/SOLiD and author mathematics remain unchanged. Author source
515759c00fe1e2f6d4d4a005e8c3a6b3f71d17ac remains clean. Direct author C++
Encoder::Encode / Matcher::Add / Finalize / Shortlist / BeamScore / Verify.
The unchanged author Python build_submap performs point accumulation; this is
not a Python feature/verification fallback. Original min32, 500RANSAC, 1m,
16RANSAC-inlier, 8keypoint-inlier and official encoder settings are retained.

Runtime input/caller is committed at CODE_SHA. Post-hoc analyze.py is added
in the archive commit, never called by the experimental binary. Execution
receipt's source directory hash list also captured a then-uncommitted analysis
helper; that non-runtime entry is not the compiled caller identity. Preserve
receipts unchanged. Binary SHA256:
f71f1f98d406c0480578a483127e366238c7696aa1c6560ea62a69f3e52d57b7.

## Input protocol and limits

Current and consecutive past scans only, newest-first, anchored at current.
Reference and query share the same definition. Raw invalid zeros are removed
before transform so translation cannot create fake returns. Empty members and
gaps are rejected. All 5910 admitted member records satisfy current-end cutoff;
pose matrices are finite SO3. Per-member matrices/times are external, with
path/hash in results.json. New large point caches remain on persistent ext4:
/home/jian/rosbag/p15_inlier_causal_r1_20261011/ (about1.1GiB).

N1 reuses P14 six-query results and389 single references. N5/N10 each use388
references and all six queries. Original reference0 cannot be accumulated
because early member scans lack causal mapping state; N10 also has only six
past scans available. It is rejected, not given fake poses, and original IDs
1..388 are preserved. Thus N1 vs N5/N10 also differ in boundary DB support,
not a strictly identical-pool mathematical ablation.

Loop2 state is map_T_imu/body. Historical actual extrinsic not certified:
use explicitly APPROXIMATE_BODY_AS_LIDAR, documented present identity-rotation
assumption, omitted unknown historical lever arm, last state<=member end,
maxage0.25s. Loop3 uses existing corresponding NDT final_used map_T_lidar,
midpoint-associated raw undeskewed scans. These are approximate non-GT relative
poses, NOT truth. No future query scan or future EKF/OOSM state is consumed.
Raw bag sources are existing P11-hashed bags, no download/full re-extraction;
only bounded indexed windows are read. Whole large bag SHA is historical,
not falsely claimed recomputed this round. P14 single inputs and mapping-state
SHA were actually rechecked; generated inputs rechecked before query launch.

## Six-query outcome

Entries are Token count / maximum actual correspondences among top20.

|Query|N1|N5|N10|Verify successes N1/N5/N10|
|---|---:|---:|---:|---:|
|TX16|1 /1|3 /2|5 /3|0/0/0|
|TX45|2 /2|2 /2|2 /1|0/0/0|
|TX708|6 /2|2 /1|3 /2|0/0/0|
|TX1891|31 /8|43 /18|68 /27|0/0/0|
|TX1994|23 /7|49 /20|95 /31|0/0/0|
|TX4691|51 /16|71 /34|111 /43|0/1/1|

Mean tokens19.000 /28.333 /47.333. Positive MINT+BEAM candidates60 /60 /60,
in the same3/6 queries, NOT location correctness. Positive MINT117 per
condition; positive BEAM97/108/109 includes zero-MINT-origin candidates,
which are not counted as effective retrieval evidence. Verify120/120/120,
of which N1 is reused; NEW Verify calls=240. RANSAC entered0/1/1. Success0/1/1,
finite rigid relative transforms0/1/1, nonfinite successful poses0.
N5/N10 each119 early correspondence rejects, no later RANSAC/keypoint rejects.

TX1994 is the important dense-but-failing case: N10 has95 tokens, but only31
correspondences, still below32. No threshold reduction. First three queries
have at most5 accumulated tokens and one height layer, so MINT remains zero.
TX708 even loses tokens versus N1: accumulation is not monotonic success.

## Actual successful geometry, and a counterexample

Both positive top1 candidates refer to original ID308.

|N|MINT|BEAM|pairs|RANSAC inliers|KP inliers|residual m|Verify ms|complete query ms|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|5|0.733646|0.243902|34|24|30|0.715455|0.095264|15.332990|
|10|0.694158|0.253165|43|31|38|0.618576|0.119150|20.583483|

Actual T_sensor direction is reference<-query. N5 translation
(-0.026229,-0.223764,-0.662452)m; N10(-0.209384,-0.274227,+0.505735)m.
Full rotation/4x4 matrices are in successful_cases/results and candidate CSV.
**Same query/reference transforms differ1.183534m /6.853046deg between N5/N10.**
Both satisfy the author's finite rigid-success checks, but their agreement
and correct-place accuracy are NOT established. A lower geometric residual
does not settle correctness; ground estimation/keypoint support, approximate
accumulation and recorded NDT drift are possible confounders, not proven causes.
Do not call these two independent correct place-recognition successes.

## Cost

Units ms, mean/P95, fixed6 queries. Original author C++ core, OpenMP1.
Complete query means accumulated PCD read through top20 Verify including
preprocessing/diagnostics, not live ROS/bag acquisition or online deployment.
Author accumulation timing from already loaded member arrays is separate.

|Stage|N1 (historical)|N5|N10|
|---|---:|---:|---:|
|Accumulation|NOT_RUN|1.073398 /1.362717|2.202611 /3.124808|
|Encoder|1.433420 /3.257306|1.131611 /2.168220|1.467503 /2.782723|
|MINT|0.064092 /0.123673|0.078898 /0.143316|0.096195 /0.183963|
|BEAM|8.301864 /8.758016|7.703022 /8.091534|7.692662 /8.108297|
|Verify top20 sum|0.055967 /0.079132|0.094322 /0.183910|0.119569 /0.245249|
|Complete query|11.628267 /15.703502|15.567018 /20.216204|20.093357 /21.487952|
|Peak RSS MiB|21.140625 historical full P14|25.179688|32.085938|

Complete-query max17.026507 /21.680139 /21.789441ms. Actual successful Verify
times above are not early-reject times; successful query costs are separately
shown. N5/TX4691 accumulation0.825739ms, N10/TX4691 1.612856ms; no historical
stage times are added to fabricate same-run end-to-end latency. Query timing
includes warm file-cache effects. Only one success per condition: no reliable
successful-Verify P95 distribution can be inferred from a single observation.

N5 offline388-reference encoding+index2.472187s, process2.602608s; N10
4.916746s /5.061140s. Shared indexed extraction/clean/accumulate/write preparation
wall394.02s, peak369944KiB=361.273438MiB. That offline preparation is NOT the
32.1MiB C++ online-style query peak or a10Hz deployment claim.

## Verification and decision

Release compilation PASS. Causal SE3 direction tests, rotation/translation,
empty/gap/future rejection and zero-return tests PASS. Bounded independent
source review identified and closed silent-empty skipping and original-ID
mapping. No external-model CLI ran. CSV/JSON/finite-success/hash audit and
git diff --check PASS. Incremental independent adapter preserves production
and historical implementations; no algorithm parameters changed after results.

Post-run archive generation initially rejected a relative script path against
the absolute repository root. Resolving both paths before constructing hash
keys fixed this bookkeeping error. The relative-command regression and hash
audit were rerun; no Encoder, retrieval or Verify experiment was repeated.

Worth retaining as a **limited mature functional baseline**, not a certified
online replacement/accuracy baseline. There is now genuine geometry, so the
user's mandatory no-success stop condition is not met. Nonetheless this round
has stopped; no more accumulation sizes or tuning. Recommend decision AI
review these two successes and cross-N disagreement against independent
correctness evidence before deciding any further use. Do not deploy feedback
or start Dual-U from these results alone.
