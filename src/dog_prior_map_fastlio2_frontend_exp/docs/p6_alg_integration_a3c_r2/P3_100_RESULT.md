# P3_100_POST_REPAIR_SHORT_LINK_PASS

Exactly one run, process exit0, frame_limit100, 51 raw scans before handoff,
49 actual Window-owned deskews and49 complete terminal optimizations. This is
an engineering gate, NOT localization accuracy.

Manifest/hash gate PASS. Completed49 nominal NDT +24 positive/negative probe
pairs =97 logged align calls. U_obs and sparse pre-measurement P15 valid49/49.
46 LiDAR factors committed,3 normally rejected by selected NIS.

Schur60/60 oldest removals PASS,30 enforcement episodes each requiring2
removals. All stored prior max symmetry defects0 and recorded minimum
eigenvalue0. Consumed H/Hmm symmetry exact0; raw Schur maximum across all
attempts2.3283064365386963e-10. No solve jitter used in this real run.
At tx90 BOTH attempts succeeded and final span1.916218042s; not just a removed
error message. See TX90_REPAIR_RESULT.md.

No optimizer failure, NaN in recorded poses, illegal recorded quaternion or
timestamp reversal. No legacy input leakage, sparse fallback, visual factor/event,
candidate basis callback or post-handoff IKFoM call. Non-LiDAR covariance
requests stopped. Completed max nodes40, span1.958914906s. Original sparse
marginal production path and block-sparse optimizer are retained.

Resource:39.14s wall,33.29s user,2.45s system,94,308KiB max RSS.
Enabled forensics are included; NOT a formal performance benchmark.

All gate checks completed BEFORE the conditional one P3-200 was started.
Exact small CSVs and read-only audit output: RUN_P3_100/.
