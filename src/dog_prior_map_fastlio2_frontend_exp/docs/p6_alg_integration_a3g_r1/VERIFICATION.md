# Offline verification and scope

START gate: correct branch/HEAD88831c3906f05589031af8d06c95bc405d9d24d9,
clean worktree. No baseline reset, rebase, merge or worktree creation.

Frozen external ledger SHA pinned to
4992c4f56fa49e2b8837f57733b9edc2bde51efb997e6c475a716be35a3cdf05;
all22 external files passed exact SHA/size before parsing. Official calibration
also passed pinned SHA. ARTIFACT_SHA_GATE.json preserves paths/SHA/bytes.

Offline helper tests: **7/7 PASS**:

1. Integer sensor-time ordering despite inverted transaction numbers;
   duplicate transaction/stamp fail closed.
2. Persistence counts consecutive observations, not identifier gaps.
3. No-preview geometric failure is not reclassified as NIS rejection.
4. Extrinsic frame round-trip and row-major matrix interpretation.
5. SO3 log magnitude at small/large/pi angles and quaternion sign invariance.
6. Exact external SHA gate; tampering rejects; a self-consistent replacement
   ledger cannot impersonate the pinned frozen run.
7. Normalize float NDT LiDAR rotation before composing inverse extrinsic.

All seven generated CSVs and both JSONs were independently regenerated into a
new temporary output directory: **byte-for-byte identical**. Old outputs are
never overwritten by the analyzer; generated copies were installed only in the
new A3G-R1 docs directory. The script leaves all frozen inputs untouched.

Additional frozen-artifact assertions: lastcommit182, first persistent completed
reject183, persistent exact NO_VALID188, zero-active202–366/count165,
increment crossings160/176/215/285, deskew invariant failures0,
P15 available314, audited covariance rank/full positivity81/81,
all314 completed terminals Window-owned provenance: **PASS**.
These assertions corroborate reported facts, not physical localization accuracy.

The script uses Python3 and existing NumPy1.24.4. Analysis script SHA256:
7ce5036a7afed46ba6f62ad95de5b974ea6756be8442007e3e55982a86f266a9.
Reproduction (new directory required):

```bash
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p6_a3g_r1_support_forensics_test.py
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p6_a3g_r1_support_forensics.py --output /tmp/new_a3g_r1_analysis
```

Fresh-context single-model adversarial review covered the new helper/data/report
only. Three substantive findings were accepted and fixed: float rotation
normalization order; distinguishing failed366 from completed measurement
rejections; pinning the ledger identity rather than trusting any self-consistent
ledger. No frozen estimator audit/rework. External cross-model review remains
manual after handoff, per the user's prior choice; no external CLI was invoked.

Production source/include/CMake/existing tests/old docs unchanged. No Release
build or CTest rerun was requested or performed in this offline-only stage;
no prior-stage test results are relabeled as newly run. All changes are new
analysis scripts or report artifacts. Final staged git diff-check is recorded
in the execution handoff. No production or rank-gate fixes.

REAL_REPLAY_COUNT=0; GT_USED=false; READY_FOR_FORMAL_EXPERIMENT=NO.
