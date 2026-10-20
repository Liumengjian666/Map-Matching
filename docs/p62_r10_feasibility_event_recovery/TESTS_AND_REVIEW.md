# R10 pre-run evidence

The new test_events C++ program was built using existing GCC9.4/Eigen3.3/PCL1.10 and returned R10_EVENT_STATE_TESTS_PASS new_aligns=0. It does not run the R9 test program or any old experiment.

Covered cases: legal nominal without event; translation/rotation/support admission failure events; old nonconvergence/maxiter/high-median triggers; nonfinite pose/scores, invalid finite SE(3), missing validation, insufficient coverage; event detection with exhausted budget; supported alternative; eligible but unsupported alternative; stale selector targeting invalid, singleton or deduplicated candidate; exact D=.02 rejection; B4/B5 final decision; individual final rejection for each tested bad nominal; invalid prediction cannot become a legal fallback; global20-token exhaustion. No native align was added by these tests.

Local adversarial review, three bounded state/driver cycles:
1. Required additional final-rejection, legal-nominal fallback, strict evidence and dedup tests. Added. Old-trigger availability was clarified against the actual R9 source, not silently changed.
2. Found exception-path loss of already returned raw candidates before record(). Fixed by flushing RAW_NATIVE_OUTPUTS before validation and recording validation failures separately.
3. Confirmed the exception-path fix and found no new state-machine issue. Reviewers read source only; they did not run native programs or audit R9 result archives.

The optional generic definition-of-done reference bundled with the skill is unavailable locally; the explicit task gates and actual build/tests govern this research artifact. User declined cross-model review; external CLI executions remain0. No production edit or GT use.

Actual full-run receipts, counts and post-run checks will be added without replacing this pre-run history.

Additional runner review identified that the inherited native wrapper performed fitness NN before return and that binary/source hashes alone did not constitute build binding. R10 now captures/flushed optimizer output before auxiliary fitness and skips fitness for unsafe pose/score; unit callbacks verify zero fitness calls on unsafe values. A committed-source cold build captures source identities before/after, commands and executable hashes plus the actual zero-align test receipt; the runtime gate verifies that binding. It does not run any R9 experiment or test program. These safety changes do not alter optimizer math, objective, initial seeds or thresholds.
