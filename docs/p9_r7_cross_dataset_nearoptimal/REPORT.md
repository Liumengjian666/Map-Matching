# R7 input-contract stop

FINAL_RESULT = CROSS_DATASET_INPUT_CONTRACT_BLOCKED

NEXT = RESTORE_CROSS_DATASET_BASELINE_CONTRACT

## Decision

The specified v1 derived bag does not exist at its frozen path. The full expected
SHA was recovered from the real historical manifest, not the truncated task
display. The bounded local search found sidecars only. No alternate v2 bag was
used and no new bag was generated. The exact missing path and hash are in
`input_manifest.json`; full coordinate/timing caveats are in DATASET_CONTRACT.md.

Available official map, normalized map, extrinsics, adapter config and three
historical timestamp/hash receipts match their historical hashes (7/7).
This does not establish parity of the missing payload.

The old 2776-scan deterministic P2B baseline had persistent whole-segment drift.
That remains a historical NOMINAL_TRACKING_RISK, not a new R7 finding. Old
rotation-only scan-start processing is not automatically a scan-end P9
source/T0/U_obs certificate. No claim is made that rebuilding such a chain is
impossible; it has not been established in this blocked run.

## Scientific execution

- New NDT, baseline replay, oracle263, B12 and visual extraction: all zero.
- No GT poses, candidate labels or new U_obs were loaded for evidence.
- Cohort/prefix/labels, primary metrics, temporal statistics and GT risk: NOT_RUN.
- Absent scientific CSVs are deliberately not replaced by fabricated rows.
- R4, R5 and R6 results remain unchanged. No cross-dataset efficacy verdict.

## Verification and scope

Release build passed in `/tmp/p9_r4_release.Eirto1`.
P9 CTest: 41/41 passed, 9.67 seconds. R6 numerical/type/tensor and label/GT
isolation self-tests passed. New read-only input-audit tests passed, covering
full vs truncated SHA, mismatch, missing input and read error.
Independent read-only review found an extraction guard defect: a malformed
named hash could borrow a later field's valid hash. It was fixed by binding
extraction to the immediate field value and rejecting duplicate/malformed
fields. Regression tests pass; the final parser reproduces all eight archived
file statuses/hashes. No algorithm or experimental contract was changed.
These tests do not change the six dataset gate statuses.

Commands (from writable R4 worktree unless noted):

```text
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/audit_r7_input.py self-test
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/audit_r7_input.py audit
cmake --build /tmp/p9_r4_release.Eirto1 -j 2
ctest --output-on-failure  [cwd=/tmp/p9_r4_release.Eirto1]
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/p9_r6_nearoptimal.py self-test
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/p9_r6_nearoptimal.py isolation-test
```

Test environment: PYTHONDONTWRITEBYTECODE=1, OPENBLAS_NUM_THREADS=1,
OMP_NUM_THREADS=1, LD_LIBRARY_PATH=/lib/x86_64-linux-gnu.
Input audit is read-only and cannot launch NDT or authorize scientific execution.
Only R7 audit code/docs are added; frozen algorithms, old archives, data,
original read-only workspace and stable workspace are untouched.

Git branch: research/p9-r4-heldout-visual-evidence.
Start: df2e0a2f48ed77457f056c14ef097a955e9984d4.
End: the commit containing this receipt (resolve with git log -- this REPORT).
No push executed. Restoration is the only next task; do not silently substitute
v2, rerun an exporter, tune a baseline, or start oracle263.
