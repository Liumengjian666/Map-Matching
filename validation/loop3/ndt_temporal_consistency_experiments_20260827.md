# NDT Temporal Consistency Experiments - 2026-08-27

Goal: test a lightweight short-window temporal consistency gate for the remaining repeated-corner mismatches. Runtime inputs remain `/livox/lidar`, `/livox/imu`, `/clock`, and the prior map only. FASTLIO2Location `/localization` is used only for full-bag offline evaluation.

## Implementation

Added a default-off sliding consistency hook in `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`:

- `lidar_update/ndt_temporal_consistency_enable`: disabled by default, so the current best baseline behavior is preserved.
- The hook treats NDT results far from the propagated initial guess as risky.
- Risky NDT corrections are temporarily held to the motion-prior prediction unless multiple recent frames support a consistent correction, or unless the residual is strongly better than the prior.
- It does not run extra NDT alignments, so compute remains close to the 10 Hz baseline.

The default-off parameters are documented in `src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml`.

## Baseline retained

Current best run remains `/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`:

| Metric | Value |
| --- | ---: |
| mean | 0.417 m |
| RMSE | 0.618 m |
| p90 | 1.243 m |
| p95 | 1.535 m |
| max | 2.694 m |
| corrected rate | about 10 Hz |

Important windows:

| Window | mean | p95 | max |
| --- | ---: | ---: | ---: |
| 345-351 s | 0.755 m | 2.321 m | 2.429 m |
| 398-403 s | 0.451 m | 1.465 m | 1.781 m |
| 565-576 s | 1.717 m | 2.581 m | 2.694 m |

## Full-bag experiments

| Run | Trigger | Notes | mean | p90 | p95 | max | rate | Decision |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `ndt_temporal_consistency_v1_loop3_20260827_160832` | 0.70 m | 7 temporal events; late segment worsened | 0.444 m | 1.379 m | 1.545 m | 4.143 m | 9.99 Hz | reject |
| `ndt_temporal_consistency_v2_loop3_20260827_161953` | 1.10 m | 8 temporal events; late segment still worsened | 0.436 m | 1.446 m | 1.552 m | 3.468 m | 9.92 Hz | reject |
| `ndt_temporal_consistency_v3_loop3_20260827_163119` | 1.60 m | 3 temporal events; avoids 565 s degradation but does not improve corners | 0.419 m | 1.264 m | 1.535 m | 2.694 m | 9.95 Hz | neutral/reject |

Window comparison for the least harmful variant (`v3`):

| Run | 345-351 s max | 398-403 s max | 565-576 s max |
| --- | ---: | ---: | ---: |
| baseline | 2.429 m | 1.781 m | 2.694 m |
| temporal v3 | 2.552 m | 1.781 m | 2.694 m |

## Diagnosis

- The sliding-window gate is compute-safe because it does not add NDT solves.
- However, delaying or holding even a few risky corrections can make the local trajectory slightly worse.
- The remaining corner errors are not simple one-frame outliers; they are locally plausible repeated-structure matches, so short temporal consistency alone does not distinguish them reliably.

## Decision

Keep `ndt_temporal_consistency_enable: false` in the default config. The hook is committed as a reversible diagnostic tool, but it is not an accepted improvement over the current baseline.

Next useful direction should use additional independent evidence rather than more NDT-only gating: for example, camera/visual place cues at the two corners, stair/elevation semantic detection, or an offline smoother that optimizes a short segment jointly instead of holding online frames one by one.
