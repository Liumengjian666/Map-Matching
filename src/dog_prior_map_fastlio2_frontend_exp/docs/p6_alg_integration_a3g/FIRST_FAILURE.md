# First failure — tx366, pre-measurement covariance

Authoritative row: `RUN_FULL_FIRST_FAILURE/trajectory.csv.a3f_r1_covariance.csv`,
transaction 366, stamp 1517157256000664307.

Backend SQUARE_ROOT_QR; valid false; reason
SQUARE_ROOT_COVARIANCE_RANK_DEFICIENT; rows/columns/rank = 600/600/498;
threshold 907.22622380221742; min/max abs R = 0.028400000820474211 /
6809639472427348; pivot ratio 4.1705586522557578e-18.

QR duration 30.324398 ms. Triangular solve and P15/Pmap construction were not
reached. The initialized residual field 0 is not evidence of a successful solve.
Legacy covariance shadow NOT_REQUESTED; no fallback.

Last completed terminal tx365 pose position is [-51.903319, -885.890686,
-151.160057] m. Full-state finite checks passed, but large finite trajectory
scale/increments cannot be treated as motion-plausibility or accuracy PASS.
No GT or magnitude-based tuning/gating was used.

Health row: finite states/prior; 40 nodes; span 1.916278839 s; prior 15×600;
39 IMU / 0 LiDAR / 0 visual active factors; IDs 39; revision 1331 vs optimized
1330; optimizer NOT_RUN. Earlier marginalization PASS is not a current attempt.

`LAST_20_COMPLETED_EVENTS.csv` contains the actual preceding chronological
events, ending at tx367 scan-start stamp 1517157256000623941, 40366 ns before
the failing terminal. The current failure is present in the health/covariance
rows, not fabricated as a completed event. `LAST_5_PREOPT.csv` ends at tx365;
tx366 has no NDT/preopt measurement row because the covariance gate was first.

Last successful marginalization attempts and optimizer outcomes are retained;
there is no optimizer or marginalization failure capsule because neither stage
failed here. Process naturally returned 1 after flushing evidence; monitor did
not need to kill it. The launcher saved run_result before its postprocess-only
field-name error; corrected offline summarization is in health_summary.json.

No GT, no second replay, no repair, no rank/noise/threshold modification.
This stage does not distinguish true observability loss from numerical scale
effects. Full A_full/R_full/state matrices were not captured. No speculative
primary root-cause classification is assigned.
