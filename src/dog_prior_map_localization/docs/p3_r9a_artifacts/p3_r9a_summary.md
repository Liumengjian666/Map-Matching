# P3-R9A smoke summary

Status: engineering-only; no accuracy, physical-cause, or novelty claim. `P4_ALLOWED = NO`.

## Inputs and execution

- Raw Floor01 canonical bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag` (same source path is recorded in the experimental YAML).
- Selected windows were fixed from IMU-only motion statistics before inspecting localization output: normal startup `[0,30) s`; high-dynamic `[138,168) s`, each offset from first IMU stamp `1660857392.515903950`.
- New mode determinism: Run A, B, and locked-capture Run C each replayed from sequence start through 168 s. Legacy-CV comparator was replayed from start through 168 s.
- Run A/B/C: `1651` common NDT frames; per-frame diagnostic mismatches A/B = `0`. Locked point arrays were independently checked against their index SHA-256 before ordered point comparison.
- Startup scheduling caveat: A and B had one scan-stamp status difference at the coverage/queue boundary plus one raw cloud present only in B; this was confined to pre-initialization rejection. Both runs produced the identical 1,651 NDT input stamps and bit-identical accepted NDT diagnostics, and window capture hashes are identical. The startup rejection decision itself is not bitwise reproducible.

## Window comparison (descriptive only)

| Window | Common frames | Paired points | Ordered legacy-to-IMU delta mean / P95 / max (m) | IMU PCL score mean / P95 | Legacy PCL score mean / P95 |
|---|---:|---:|---:|---:|---:|
| Normal 0–30 s | 287 | 8151150 | 0.0150837 / 0.0604914 / 0.243447 | 0.255875 / 0.346896 | 0.257134 / 0.348518 |
| High dynamic 138–168 s | 294 | 6562202 | 0.131181 / 0.343376 / 0.852671 | 14.1324 / 20.8618 | 15.4502 / 23.1003 |

Point displacement mean, P95, and maximum in the table are pooled over all paired points; per-frame distributions are retained in each window CSV. PCL post-registration nearest-neighbor fitness is not an exact NDT optimized likelihood and is not interchangeable with reference-pose correctness.

## Safety and coverage

- New full run C: `1651` published, `12` rejected. Rejections: `{'missing_start_coverage': 8, 'pending_cloud_queue_full': 3, 'state_gap_exceeds_limit': 1}`.
- Published point count preservation: all rows. Non-finite published kinematics/displacements: zero. Maximum state gap and state bounds are in `p3_r9a_causality_audit.csv` and window CSVs.
- Current-scan NDT leakage is structurally unreachable in the deskew call graph: deskew takes state history/cloud/extrinsic only; the result is published to NDT after completion. Prior-scan NDT corrections may revise the later trajectory through OOSM replay.
- Future IMU is not used for a point: interpolation propagates from the lower, already-received sample; the upper sample is used only to establish coverage. The added contract test changes the upper sample drastically and verifies the interpolated pose does not change; violations are rejected.
- `future_measurement_used` / `current_scan_ndt_leakage` diagnostic columns are static-zero placeholders, not independent runtime counters. The audit table explicitly labels this distinction.
- `reference=end` 30 s smoke: 285 published and 12 rejected; every published reference equals scan end and every NDT input stamp matched a published reference.

## Initialization, resources, and limits

- First 200 IMU messages span 1.00493 s; accelerometer norm mean/std = 9.81950 / 0.00940 m/s²; gyro norm mean/max = 0.002255 / 0.006741 rad/s. This supports a static initialization window, but no gyro bias estimator was added; gyro bias remains the existing zero-initialized estimate.
- Acceleration integration is enabled only in the new experimental YAML; legacy/default configs remain `legacy_prior_ndt_cv` and their old `use_acc_for_position` behavior is unchanged.
- Resource samples cover EKF and NDT processes only, at about 1 Hz. Deskew processing latency and state samples used per scan are separately summarized in `p3_r9a_resource.csv`. `runtime.csv` emitted only its header, so total live history size was not independently sampled; configured retention is 2.0 s.
- This is a mature infrastructure port, not a localization accuracy experiment. No GT was consumed by runtime or window selection; no localization improvement/failure or physical root cause is inferred.
