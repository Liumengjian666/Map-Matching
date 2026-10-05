# Bounded read-only review disposition

Reviewer /root/p9_r1c_contract_review, three bounded cycles. No reviewer writes,
optimization, network, GT access or additional delegation. External Codex CLI
not invoked without new task-specific command confirmation.

Cycle1: numerical FD/strong/predictor artifact, no actionable findings.

Cycle2 actionable findings:

- frameContext's nominal dynamic evaluation was omitted from counters. Counted
  once per selected target, adding7 formal setup evaluations; total81.
- elapsed time excluded the final joint FD while counters included it. Timer
  now ends after that stencil; row runtime and evaluation scope agree.
- support-history and root/prediction flow lacked tests. Extracted the actual
  state helpers, added immediate/recurrent/long-cycle, failed stationarity,
  closure rejection and no-prediction-from-failed-root regression checks.

Cycle3 actionable findings:

- Aggregate certificate count was inferred from zero accepted roots. It now
  derives independently from audited rows; a certified-but-closure-rejected
  fixture confirms certificate/acceptance denominators remain distinct.
- FD PASS flags were trusted. Audit now independently recomputes all42 flags
  from the fixed h and uniform error limits, verifies exact target/direction
  coverage, and tests the actual P09 failure numbers as a regression.

All fixes classified valid/actionable against the source. The current formal
data still have0/14 certificates and41/42 FD diagnostics PASS. No algorithm was
rerun to improve those numbers after review; only audit/derived JSON regenerated.
Source and sidecar hashes are checked after final edits. Seven P9 tests pass.
Stop at the three-cycle bound; no fourth unchanged-artifact review is claimed.

Scientific limits remain: real-data continuation never started, many archived
terminals/frozen float FD stencils fail the strict stationarity gate, and the
model is not mathematically disproved. No positive multibranch/discovery or
full-refine safety claim is made from conditionally empty tables.
