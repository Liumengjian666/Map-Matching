# R7-R1 input and baseline recovery — blocked before reconstruction

FINAL_RESULT = CROSS_DATASET_INPUT_RECOVERY_BLOCKED

NEXT = RESOLVE_DATASET_PROVENANCE

## What was verified

Writable research worktree exists, clean at start, on
`research/p9-r4-heldout-visual-evidence` at
`11966f46fff946984012510e0bbaca103c472efc`.

A bounded read-only filename search of the SuperLoc data tree, historical
adapter workspace and relevant mounted experiment directories found no v1 bag
candidate. Scope/patterns are archived in historical_input_search.csv. This
does not prove that no renamed or off-machine backup exists. Old sidecars and
the existing v2 files were preserved; none was substituted for v1.

The complete 6473379689-byte raw official bag was rehashed, not merely stat'ed:

`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`

This exactly matches history. Velodyne commit matches
`29abd0e1361cb7f5eda451d2b51c35eeca45e0d5`, with a clean tracked worktree.
VLP16db calibration matches its historical SHA. No raw messages were decoded.

## Two unresolved prerequisites

1. **Persistent output authorization.** `/media/jian/HIKVISION` has roughly
   459 GiB free and permissive filesystem mode bits, but is outside this
   session's writable roots (workspace and `/tmp` only; approval unavailable).
   This is a declared sandbox-policy constraint, not an observed ENOSPC or
   permission-denied syscall. No forbidden write probe or permission bypass
   was attempted. The user expressly forbids keeping the only reconstructed
   data in temporary storage, so no reconstruction was launched.
2. **Historical implementation closure.** 7/10 directly listed adapter files
   still match the P2B manifest; CMakeLists.txt, package.xml and launch differ.
   The matching wrapper includes a shared implementation not pinned by that
   manifest. The current 656-line shared implementation publishes full-SE3
   output on the default points topic and waits for prior odometry. Its
   presence cannot establish historical rotation-only v1 equivalence. Current
   source/binary hashes and installed dependency versions are archived, not
   preserved file copies or certified historical implementation artifacts.

No claim is made that the historical source is irrecoverable everywhere.
No original v1 generation script was identified by the scoped filename/content
search in the current adapter source/config tree. In particular, this receipt
does not invent a reconstruction command or silently use current defaults.

## Execution status

INPUT_RECOVERY = BLOCKED_BEFORE_RECONSTRUCTION.
Reconstruction attempts = 0; the one-shot budget is unconsumed.
Cloud/IMU/timestamp parity = NOT_RUN (not 0% measured parity).
Scan-end, baseline runs A/B, source/T0/U_obs/weak-subspace parity = NOT_RUN.
Required sequence length remains 2776; no frames were processed or discarded.
Baseline/oracle263/B12 calls and visual extraction = 0.
R6 primary statistics = NOT_RUN; GT_LOADED = NO.
Historical whole-segment NOMINAL_TRACKING_RISK is retained; no new tracking
finding or GT-risk score was produced. GT_RISK_NOT_AVAILABLE.

No experiment-output file is fabricated. The five baseline artifacts listed
in results.json are absent; CSV status rows explicitly say NOT_RUN. Alignment
mean/P95/RSS and replay wall time are null, not measured zeros.

## Verification scope

Release build, existing P9 tests and the historical synthetic IMU/LiDAR frame
test are run only as software checks. They do not certify input equivalence or
dataset replay. The frame test returns vector round-trip 1.439e-15 and pose
round-trip 3.846e-16. R7 input-audit self-test passes. Detailed verification
receipt is separate; real source parity and deterministic replay tests remain
NOT_RUN because no recovered bag or baseline exists.

Only this recovery archive is added. Stable/original workspaces, data, converter,
adapter, NDT and R4/R5/R6/R7 historical receipts are not modified. Commit hash is
the commit containing this report. No push is performed.
