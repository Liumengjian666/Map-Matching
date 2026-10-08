# R7-R2: storage restored; historical v1 lineage not recovered

FINAL_RESULT = HISTORICAL_V1_LINEAGE_UNRECOVERABLE_IN_AVAILABLE_SOURCES

NEXT = PROSPECTIVE_P9_SCAN_END_INPUT_PROTOCOL

## Storage: AUTHORIZED

The new session has unrestricted filesystem access. Under the requested
Corridor01/results parent, mktemp created `p9_r7_r2_recovery.l8b521`.
A small file was written with apply_patch and read back exactly; its SHA is in
storage_authorization_receipt.json. The disk has about 459 GiB free. No chmod,
sudo, remount or symlink bypass was used. This resolves R1's storage blocker.
The directory retains only the probe, not a reconstructed dataset.

## One bounded historical search: FAIL to recover v1 lineage

The adapter workspace and both custom adapter packages have no Git metadata.
The nested Velodyne repository is at the required commit, clean. Custom adapter
file history is absent from the inspected paper repositories' all-ref/all-reflog
path searches. Branch/reflog listings were examined, not only current HEAD.

The real historical P2B manifest is recoverable from commit
`71aaafeb6357f9b8e8e5b3415b6cbe26cfc4809b`, blob
`622d977106bc61615e2132665b371a0cb493f360`.
It supplies checksums, not missing source content. Seven of its ten directly
listed files still match; launch/CMake/package do not. Its matching wrapper
includes the shared adapter, whose old bytes are not fixed by that manifest.

A second Git object, `8aaf44d09eb27af9eae89f6ad9742de0ac218502`, is a later
adapter_manifest.txt. It explicitly records no adapter Git metadata, and its
shared-source/header/binary hashes equal today's full-SE3 implementation.
It documents full-SE3 v2, not an independent rotation-only source snapshot.

Current build dependencies confirm wrapper -> shared cpp -> full_se3_deskew.hpp.
Only current object/build records were found, not preserved historical .i/.ii
source. The inspected ZIP with 'source' in its name contains a GT CSV and map
PNG, not adapter source. Its directory listing alone was read; GT was not loaded.
Related backup/experiment filename scans found no historical source candidate.
Search roots, scope and access limitations are explicit in the search CSV.
This is not a universal claim that no off-machine or renamed copy exists.

The current implementation publishes full_out on the default points topic,
rot_out separately, and waits for previous odometry after bootstrap. Neither
changing that topic nor deleting the full-SE3 path is authorized historical
recovery. No such modification or inferred generator was created.

## Prerequisites and execution

Raw-bag SHA PASS is reused from R1's full-byte measurement, not redundantly
recomputed: c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811.
Velodyne commit and VLP16db hash remain matched. Current source hashes were
rechecked against R1. Dependency records are current-state hashes, not archived
historical source copies or a complete historical dependency certificate.

INPUT_RECOVERY = BLOCKED. BASELINE = NOT_RUN.
Reconstruction attempts, baseline replay runs and baseline NDT calls are zero.
Oracle263, B12, visual extraction and GT loading are zero. Message parity and
source/T0/U_obs/W2 repeatability remain NOT_RUN. Runtime/RSS/NDT mean/P95 are
null because no reconstruction/replay occurred. Historical tracking risk and
all R4/R5/R6/R7 scientific conclusions remain unchanged.

Only recovery_r2 receipts are added; no stable workspace or algorithm change.
Release/P9/source-hash/CSV-JSON checks are software/archive checks, not a v1
equivalence or scientific PASS. Verification results are recorded separately.

Git start: dcda85dd94aa7f948582cd02087a77eb6002c9f2.
Branch: research/p9-r4-heldout-visual-evidence.
End: the commit containing this report. No push executed.
