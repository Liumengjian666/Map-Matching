# P3-100 first-failure context

First failed optimizer event: **transaction 83, `LIDAR_SCAN_END`, `1517157227458992315 ns`**. The runner reported `producer_optimizer:all_optimizer_candidates_rejected` and exited 1. Stop was honored; no scan after tx83 was processed.

Immediately preceding successfully logged event: tx83 `LIDAR_SCAN_START`, `1517157227358153105 ns`; Window had 40 nodes over 1.916239666 s and status `ACCEPTED_UPDATE`. Deskew evidence for tx83 records anchor position `(4.711537, -3.380652, -0.140631) m`, predicted end position `(4.842427, -3.132160, -0.151738) m`, 28,943 input/output points, and point stamps exactly spanning the raw catalog interval.

The complete diagnostics for every successfully completed event before the exception are retained in `RUN2_P3_100/events.csv` and `runtime.csv`; these files include more than the 20 immediately preceding events and were not truncated. The tx83 scan-start row is the last row in both. `trajectory.csv.deskew_evidence.csv` includes the raw/deskew evidence for tx83, but the event logger writes its row only after optimization.

Failure-time Window summary immediately before the failed terminal is therefore the tx83 scan-start summary above. The exception identifies the optimizer failure status. The exact tx83 NDT terminal (pose/fitness), U_obs result, P15 values, U_nonlocal result, selected NIS, optimizer candidate diagnostics, and failed-event timing were not flushed before the exception and are **unavailable**. The code order places NDT processing and factor selection before the optimizer call, but no per-event values are inferred here. Re-running tx83 to recover them would violate the stop-on-failure instruction and was not done.

The next 20 sensor events, as scheduled by the unchanged raw catalog, are preserved in `RUN2_P3_100/failure_after_stop_schedule.csv`, all marked `NOT_EXECUTED_AFTER_STOP`. They are schedule context only, not replay outputs.
