# A3G-R3 exact production evidence capture

Status: `A3G_R3_EXACT_PRODUCTION_EVIDENCE_CAPTURE = PASS`

This was one deterministic SuperLoc Corridor01 prefix run from the frozen handoff/initialization through tx190 `LIDAR_SCAN_END`. `REAL_REPLAY_COUNT=1`, `RETRY_COUNT=0`, and `GT_USED=false`. No support-cause inference or geometry counterfactual was run. Production estimator mathematics were unchanged; the only C++ change is an opt-in observer that serializes already-produced inputs/outputs.

## Identity and run

- Baseline: `484da9c88ad5866a58761aeef702a31a168607a2`
- Capture executable source: `65b803950902f0a36856d6a6187eb3e82eb72984`
- Release binary SHA-256: `baa82091545017c533d9f78ac70d1203541e760a5f52b73d9ef6f617c5b76098`
- Mode/policy: `FULL_FIXED_LAG_V3_EXPERIMENTAL` / `ADAPTIVE_SELECTED_NIS`
- Visual: `NONE`; initialization: `1517157224188979000`; frame limit: 190 (includes pre-handoff scans)
- Input manifest, raw LiDAR, catalog, filter schedule, IMU, map, official parameters, and frozen A3G external artifact ledger all passed their recorded SHA checks.
- The runner recorded one estimator invocation, exit 0. Strict endpoint audit found 139 terminal rows and exactly 278 scheduled scan-start/end event keys; the final terminal is tx190 at `1517157238250342442`, with no extra tx191 scan-start or later event.

## Captured evidence

All requested 14 transactions were captured: 159, 160, 163, 165, 166, 173, 174, 176, 182, 183, 185, 187, 188, 190.

- `SELECTED_WINDOW_STATES.csv`: 28 rows, one production scan-start 15D state and one scan-end premeasurement 15D state per selected transaction.
- Each transaction directory contains the exact raw timed records, the production deskew result captured before preprocessing, the exact preprocessed `Cloud` passed to nominal NDT, actual production IMU trajectory knots, schema, and metadata with pose/calibration/provenance/NDT/reliability/NIS/factor/revision/prior fields.
- Raw records are byte-identical to the corresponding frozen raw-timed catalog slices. Raw/deskewed point counts match on all selected frames. The full per-file byte counts and SHA-256 values are recorded in `CAPTURE_MANIFEST.json`.
- `DESKEW_KNOTS.csv` was explicitly materialized in production for all 14 scans (22 or 23 knots each).
- External evidence directory: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3g_r3_capture`
- External `MANIFEST.json` SHA-256: `fe29dd76d4906c674f7f4104b5b67df442d7ed9089492b10289802794fdac180`
- Evidence inventory: 103 payload files, 33,610,269 bytes excluding the manifest. PCD and raw binary payloads remain outside Git.

## Prefix and observer parity

The offline prefix audit passed all eight recorded tables: trajectory, events, preopt, deskew evidence, covariance, health, marginalization, and runtime. Every compared non-timing field matched the frozen A3G prefix; all comparison difference counts are zero. Time-cost columns were excluded as authorized.

The scan-end state check respects the frozen preopt writer's actual serialization contract: the old `predicted_position` field is emitted by `vectorField()` with 10 significant digits, while the new direct state capture retains 17 digits. The audit compares at that stored precision and compares quaternion rotation up to sign. The captured full-precision state itself is preserved in both CSV and JSON.

The synthetic capture-OFF/ON producer parity fixture passed. Release build and all 42 Release CTests passed; the 22-test Debug targeted suite passed, and the latest audit guards plus capture parity passed 4/4 in Debug. `git diff --check` passed.

## Per-frame capture summary

This table inventories production outputs; it is not a localization-quality assessment.

| tx | raw / deskew points | NDT source | NDT converged / iterations | U_obs status / reliable rank | support correspondences | selected NIS / threshold | committed |
|---:|---:|---:|---:|---|---:|---:|:---:|
| 159 | 27799 / 27799 | 1400 | yes / 56 | valid / 3 | 665 | 15.4822 / 11.345 | no |
| 160 | 27614 / 27614 | 1400 | yes / 80 | valid / 3 | 1030 | 2.7170 / 11.345 | yes |
| 163 | 28442 / 28442 | 1400 | yes / 80 | valid / 3 | 498 | 4.7576 / 11.345 | yes |
| 165 | 28780 / 28780 | 1400 | yes / 80 | valid / 3 | 931 | 11.6821 / 11.345 | no |
| 166 | 28692 / 28692 | 1400 | yes / 0 | `NO_VALID_GEOMETRIC_CORRESPONDENCES` / 0 | 0 | not reached | no |
| 173 | 29006 / 29006 | 1400 | yes / 80 | valid / 0 | 87 | not reached | no |
| 174 | 29047 / 29047 | 1400 | yes / 80 | valid / 3 | 424 | 9.4927 / 11.345 | yes |
| 176 | 29031 / 29031 | 1400 | yes / 52 | valid / 3 | 397 | 5.1847 / 11.345 | yes |
| 182 | 29094 / 29094 | 1400 | yes / 40 | valid / 5 | 228 | 14.2517 / 15.086 | yes |
| 183 | 29088 / 29088 | 1400 | yes / 0 | `NO_VALID_GEOMETRIC_CORRESPONDENCES` / 0 | 0 | not reached | no |
| 185 | 29093 / 29093 | 1400 | yes / 80 | `MAP_SUPPORT_INSUFFICIENT` / 0 | 17 | not reached | no |
| 187 | 29075 / 29075 | 1400 | yes / 49 | `MAP_SUPPORT_INSUFFICIENT` / 0 | 25 | not reached | no |
| 188 | 29070 / 29070 | 1400 | yes / 0 | `NO_VALID_GEOMETRIC_CORRESPONDENCES` / 0 | 0 | not reached | no |
| 190 | 29107 / 29107 | 1400 | yes / 0 | `NO_VALID_GEOMETRIC_CORRESPONDENCES` / 0 | 0 | not reached | no |

NDT objective/fitness, all timestamps, poses, exact state vectors, covariance-independent factor/admission fields, and the full event-by-event parity are available in the external metadata and committed manifest. No conclusions about why support was lost are made here.

## Boundaries

This capture is evidence acquisition only. It does not isolate root cause, does not evaluate GT/ATE/RPE, and does not authorize a full Corridor01 run or a formal experiment. `ROOT_CAUSE_ISOLATED=NO`; `READY_FOR_FORMAL_EXPERIMENT=NO`.
