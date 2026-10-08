# R4 frozen source-cloud provenance reconstruction

This is input provenance reconstruction, not a new discovery experiment. The
previous R4 scientific result remains NOT_RUN. Topic-bag `cloud_end_frame` is not
the source trajectory of the frozen SAME_OBJECTIVE current baseline.

## Frozen implementation and inputs

Use detached commit `9945c4f5c3d7759104de108a594bcaf2553fd78c` and the historical
SAME_OBJECTIVE IMU CSV, filter-scans CSV, raw-timed index/binary, map PCD and
parameters file. Initialization stamp is 0; all 4127 frames execute in their
original sequential order, with the same prediction, scan-window preparation,
deskew, source construction, preprocessing, nominal NDT and state corrections.
The historical provenance JSON and control clouds are anchored in a prior
Git-pinned R2A receipt, not a newly generated manifest. The external FAST-LIO2
dependency remains pinned at `7cc4175de6f8ba2edf34bab02a42195b141027e9`.

## Export-only instrumentation

The sole source patch replaces the historical 32-ID export predicate with a
192-ID predicate and changes the expected export count to 192. Targets are the
32 historical controls plus the 160 IDs in the immutable R4 ordered pool,
SHA256 `e877aeec2b6df1fa49dd11bc837b748612ec17b0b5a2d850fa018ed99b624d37`.
The union is sorted for export selection only; held-out selection order is not
changed. No cloud values/order, NDT call, filter, update or algorithm changes
are permitted. A literal patch verifier and compiled selection self-test check
this boundary. The patched runner is not production code or a replacement R4
NDT implementation.

## Single authorized replay and ordered gates

Build, input/hash checks and synthetic self-tests precede the replay. An
exclusive-create start receipt enforces one scientific replay invocation; a
crash or incomplete replay stops the task without retry or parameter changes.
Every replay NDT call is OFFLINE_PROVENANCE_ONLY.

Audit gates are ordered and must all pass:

1. 32/32 historical raw float32 XYZ clouds match historical SHA256 bytes.
2. All 4127 historical registration rows match source point counts and 64-bit
   source hashes, with complete ordered transaction/stamp identity.
3. Existing frozen `p9_r4_source_audit` admits all 160 recovered held-out clouds
   under the original preprocessing/hash contract.
4. Only `raw_cloud_file`, `raw_source_sha256` and `raw_point_count` may differ in
   the recovered manifest. All other field strings and row order are exact.

Failure at an earlier gate prevents evaluating subsequent source gates. Missing
or malformed exports cannot silently reduce denominators. Raw cloud format is
the historical packed float32 XYZ sequence, without resampling/reordering.
`cloud_data_sha256`, retained in the old manifest, is a legacy topic-payload
checksum, not the recovered raw-source checksum; it is not relabeled.

## Scope, outputs and costs

Raw clouds and complete replay outputs are stored outside Git in the recorded
`/tmp` cache, because the preferred external-media path is read-only here. Git
contains paths, hashes, counts, patch/command receipts and audit CSVs only.
Existing blocker receipts, incorrect manifest and negative audits are preserved.
New replay poses, trajectory and U_obs/curvature sidecars are diagnostics only;
original SAME_OBJECTIVE T0, predictor, score and U_obs remain authoritative.

Scientific cost is exactly 4127 baseline nominal NDT calls and is excluded from
R4 oracle263, B12 candidate generation and online-method runtime. Wall time,
prediction/deskew, NDT-total and NDT-alignment times are reported separately.
The frozen runner does not separately instrument export I/O. Export I/O is
therefore NOT_SEPARATELY_MEASURED; a conservative upper bound uses untimed
frame residual on export-target frames plus wall time outside timed frames.
This bound also contains cloud assembly, diagnostics and other uninstrumented
work. It is not an exact export timing or a method-runtime claim.

R4_ORACLE_CALLS=0; R4_CANDIDATE_CALLS=0; NEW_VISUAL_EXTRACTION=0; GT_LOADED=NO.
No labels, canonical basin poses or visual/GT artifacts are read by recovery.
Successful closure authorizes only the next task:
R4_RESUME_HELDOUT_ORACLE_FROM_RECOVERED_SOURCES.
