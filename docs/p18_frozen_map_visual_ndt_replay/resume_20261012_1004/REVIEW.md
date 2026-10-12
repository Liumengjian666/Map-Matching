# Bounded independent review

Fresh-context read-only reviewer inspected the small config/runner diff and
short-prefix contract. No ROS or GT was used. No external CLI was invoked.
The review found one actionable analysis bug: truncated EKF lineage could
pass if each publish category retained only one row. Counts and final sensor
timestamps are now derived from the actual frozen bag interval. The same
two-row counterexample now fails with four completeness mismatches. Full
Control self-comparison passes. No tolerance was loosened.

Reviewer confirmed 18.5s bag endpoint 1517157237.684208 and last covered IMU
1517157237.683104, expecting 3698 high-rate and 182 corrected states; the
first 180 release nodes have complete coverage. Map SHA/size, map-path-only
YAML diff and runtime map check before play were also verified. Final review
had no unresolved actionable finding. Cross-model was offered; no CLI is
authorized without an explicit command/input confirmation.

The enabled short run actually applied vision 106 times. The first audit
incorrectly required quaternion-reconstructed translation to match the raw
PCL float-matrix computation within the unrelated pose-parity tolerance.
Its original FAIL receipt is preserved in .p18_map_recovery/visual_smoke_audit.
appendPoseCsv normalizes the quaternion and loses nonorthogonal float-matrix
entries; the maximum reconstructed residual was 2.9833e-8 m, rotation change
was zero. A second independent read-only review confirmed the source formula
and serialization distinction, with no new divergence. This residual is now
explicitly diagnostic-only; the frozen Control tolerance is unchanged.
A direct C++ regression with a nonidentity, float-entry rotation checks the
actual apply formula and unchanged guess rotation at 1e-12 m. It passed.
Only that test target was rebuilt; both NDT/EKF binary hashes stayed fixed.

Posthoc review (fresh-context, read-only, no GT) found missing required
freeze entries and lack of explicit paired-time equality. The evaluator now
requires the actual full run CSV/params/bag/resources plus old Control data,
fixed transform source and application ledger, and verifies hashes before
GT access. It asserts identical paired times rather than taking a favorable
intersection. CAUSAL wording is explicitly limited to the contract-defined
controlled-switch whole-trajectory result, not framewise causal isolation.
All findings were reconciled without a new fit or changed math. Cross-model
was offered; no external CLI was run. Numeric negative maxima are retained.
