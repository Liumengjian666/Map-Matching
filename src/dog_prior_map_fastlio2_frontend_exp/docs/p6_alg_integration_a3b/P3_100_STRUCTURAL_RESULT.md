# RUN-2: P3 / 100 raw scans — STOPPED ON OPTIMIZER FAILURE

Mode: `FULL_FIXED_LAG_V3_EXPERIMENTAL`, policy `ADAPTIVE_SELECTED_NIS`, profile `corridor01`, `visual=NONE`, `visual_provenance=NONE`, frame limit 100, handoff `1517157224188979000`.

The run stopped at transaction **83**, event `LIDAR_SCAN_END`, timestamp `1517157227458992315`, with `P6_I1_RUNNER_FAILED: producer_optimizer:all_optimizer_candidates_rejected`. RUN-3 was not started.

## Successfully logged before failure

- 31 LiDAR terminal events completed; all 31 NDT registrations converged.
- U_obs valid on all 31. Nine U_nonlocal probes were logged (18 additional align calls).
- 31/31 factors attempted, accepted by selected NIS, and committed. Logged NIS range: `0.000…1.3490417`; thresholds: `11.345…16.812`.
- Pre-measurement covariance: 31 available, 0 unavailable. Non-LiDAR event requests were zero.
- Logged NDT calls: 49 (31 primary +18 probe). Transaction 83 reached its terminal producer path and failed only when `optimizeCurrentWindow()` returned false, so its primary NDT call had occurred. The exception path did not flush an exact align/probe counter; total calls for the failed run are **at least 50, exact total unavailable**.
- Window maximum 40 nodes / 1.958915 s; 12 observed node-count drops before failure.
- All 63 logged events have `ACCEPTED_UPDATE`; that count excludes the failed event. Solver status in completed runtime rows was `BLOCK_SPARSE_SIMPLICIAL_LDLT`, fallback 0.
- Visual events/factors 0/0; post-handoff IKFoM calls 0. Logged state values finite; max quaternion norm error `4.44e-16`.
- Successful trajectory contains 31 scan rows; its adjacent translation increments mean/max are `0.173016 / 0.287389 m`. This is not an accuracy metric.

## Failure event evidence gap

Deskew sidecar transaction 83 proves the raw scan was read and Window-owned deskew completed with 28,943 input/output points; point timestamp min/max equal the raw scan interval. The immediately preceding successful `LIDAR_SCAN_START` row records Window nodes=40, span=`1.916239666 s`, optimizer=`ACCEPTED_UPDATE`; the scan-start anchor and predicted scan-end poses are in the sidecar.

The producer writes `events.csv`/`runtime.csv` only after optimization. Therefore the failed terminal's NDT terminal pose/fitness, U_obs result, exact P15, U_nonlocal trigger/result, selected NIS, and per-event solver diagnostics were not persisted. They cannot be reconstructed without re-executing the failed event; this report deliberately does not rerun it. All successfully completed prior events, including the 20 immediately preceding rows, remain in `events.csv` and `runtime.csv`. The next 20 *scheduled* events are listed in `failure_after_stop_schedule.csv` with `NOT_EXECUTED_AFTER_STOP`; they have no estimator outputs.

See `P3_FAILURE_CONTEXT.md` for exact failure timing and scope. No parameter, threshold, math, map, or calibration adjustment was made.
