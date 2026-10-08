# R6 verification and review receipt

Scientific starting commit: `5a95362861464d9b8756e444ea3dcf999e032d6a`.
Only new R6 files are changed. No R4/R5 archive, frozen solver/frontend,
U_obs/W2, original workspace or stable workspace is modified.

## Checks

- Release build PASS; existing build is configured Release. See release_build.log.
- Existing P9 suite: 41/41 PASS; see p9_tests.log.
- R6 standalone numerical tests PASS: inclusive near-score boundary, negative
  scores, source normalization, T0 versus nominal representative distinction,
  TYPE A/B channel separation, representative suppression, and frozen spatial
  chart/nonconvergence/iteration-limit tests. See numerical_self_test.log.
- R6 isolation tests PASS: deny labels, canonical clusters, labeled R5 results,
  external data and a copied temporary file opened through r/a+/r+.
- Exploratory statistics tests PASS: AUC direction and ties.
- Cluster source SHA and representative/partition/geometry parity: 192/192 PASS.
- Original STRICT count parity: 192/192 frame-pool records PASS; aggregate
  B12 17/40 and 3/56, FULL263 30/40 and 5/56.
- CSV/JSON/tensor/hash audit verifies separate outer-product sums, inactive
  zero channel, distributions, AUC, event counts and complete artifact inventory.
- Blind evidence was frozen before the evaluator opened labels. The exact data
  reads and input/source/binary SHA256 are in evidence_freeze.json.
- NEW_NDT_CALLS=0; BASELINE_REPLAY_CALLS=0; VISUAL_EXTRACTION=0; GT_LOADED=NO.

## Independent adversarial review

The reviewer first inspected math, source carriers and information isolation.
It identified a real read-guard defect: update modes such as a+ could bypass a
mode-string test, and copied data outside selected prefixes was not covered.
This was fixed before any labels were loaded. The reviewer rechecked the changed
guard and found no remaining required issue.

Final independent recomputation verified all 5,332 candidate eligibility
decisions, 192 STRICT parity rows, event counts and all four exploratory AUCs.
No required findings remain. The reviewer specifically assessed the limited
PROMISING_EXPLORATORY wording against increased B12 proxy false alerts and
FULL263 saturation; it is a descriptive judgment, not a numerical gate PASS.
No external-model CLI was executed.

## Errors preserved and resolved

1. Initial blind build stopped before evidence_freeze.json existed:
   `LABEL_GT_READ_GUARD_DENIED: .../evidence_freeze.json`.
   A low-level write open has integer flags rather than a string mode. The guard
   mistook it for a read. It now uses O_ACCMODE, allows pure writes only beneath
   the R6 output directory, and checks all readable opens against an explicit
   allowlist (apart from installed runtime resources under /usr, /lib, /lib64).
   No labels were loaded, no scientific parameter was changed, and the corrected
   label-blind build completed before the separate evaluation process.
2. First final-report packaging failed because relative `__file__` was passed to
   relative_to with an absolute root. Resolving the code path fixed hash packaging.
   Evidence and evaluation were unchanged; no parameter or input was substituted.
3. A later report round-trip audit caught dictionary-order-dependent rendering
   in the added NO_MAJOR support-structure summary. Numeric values were identical.
   Its inline JSON now sorts keys, and a report-before/after-JSON-round-trip guard
   runs before saving. No evidence or evaluation was rerun for this formatting fix.

These were implementation/packaging errors, not scientific input-contract or
admission failures. The original R4/R5 conclusions remain unchanged.

## Scope of timings and conclusion

The measured R6 calculation reuses cached complete-link clusters. It is not a
fresh clustering benchmark and does not remove the historical ~827ms/frame B12
alignment cost. Offline reference FULL263 is not an online budget recommendation.
The final classification is posthoc development only. New-dataset validation is
the sole proposed next step; no EKF, fusion, pose switching or threshold tuning.
