# A3G-R2 summary

Result: **the requested geometric cause was not isolated**. The required strict deskew reconstruction gate could not be met from the frozen A3G run artifacts, so the analysis stopped before transforming scans into map coordinates or counting geometric support.

This is an evidence-availability stop, not a production deskew mismatch and not evidence for any of the competing physical causes. No map-overlap, nearest-neighbor, or counterfactual support quantities were computed.

| Gate | Result |
|---|---|
| Required START_SHA / clean worktree | PASS at entry: `283be8cf8298e24200099c4a1e51eb3ed1c2d0f7` |
| Frozen run artifact ledger | 22/22 entries match SHA256 and byte size |
| Raw timed input, map, calibration, IMU identity | PASS; hashes match the frozen A3G/A3F identity |
| Selected raw point records | PASS: all 19 allowlisted scans match catalog and logged point count/time extrema |
| Strict production deskew reconstruction | BLOCKED: no saved point-level deskew output or full scan-start `WindowState` history |
| Production geometric-support reproduction | NOT RUN; prerequisite above failed |
| Real replay / GT | 0 / not used |
| Production algorithm changes | None |

Primary classification: `A3G-R2-F ROOT_CAUSE_NOT_ISOLATED`.

`A3G_R2_LIDAR_SUPPORT_ROOT_CAUSE_ISOLATED = FAIL`; `A3G_R2_STOP_GATE_COMPLIANCE = PASS`. The next required evidence is a frozen per-selected-scan 15D scan-start state (rotation, position, velocity, gyro bias, accelerometer bias), the exact causal IMU slice/noise/gravity and calibration identity, plus either the exact deskewed point cloud or a deterministic reconstruction path that can be checked against a saved point-level reference.
