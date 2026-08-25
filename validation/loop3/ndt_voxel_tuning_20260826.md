# NDT voxel/resolution tuning - 2026-08-26

Baseline before tuning: `safe_step_limit_no_truth_loop3_20260826_010620`, compared to FASTLIO2Location `/localization` offline only. Runtime playback topics were `/livox/lidar`, `/livox/imu`, `/clock`.

## Result summary

| Run | Key parameters | mean | p95 | max | Decision |
| --- | --- | ---: | ---: | ---: | --- |
| baseline | source 0.35, target 0.25, resolution 1.0, max source 900 | 0.646 m | 3.515 m | 5.608 m | old baseline |
| `ndt_finer_map_source_loop3_20260826_044853` | source 0.25, target 0.15, resolution 0.8, step 0.08, max source 1400 | 0.417 m | 1.535 m | 2.694 m | keep |
| `ndt_fine_target_balanced_loop3_20260826_050015` | source 0.30, target 0.15, resolution 0.9, max source 1200 | 0.552 m | 1.743 m | 5.268 m | reject |
| `ndt_finer_res07_loop3_20260826_051104` | source 0.25, target 0.15, resolution 0.7, step 0.06, max source 1400 | 0.809 m | 4.772 m | 9.139 m | reject |

## Window analysis for kept run

- 390-410 s stair/corner window: max 1.781 m, p95 0.868 m, mean 0.281 m. Baseline max was 5.608 m.
- 420-445 s window: max 0.640 m, p95 0.605 m, mean 0.276 m. Baseline max was 3.942 m.
- 455-572 s window: max 2.694 m, p95 1.036 m, mean 0.399 m. Remaining peak moved to terminal/static region around 570.5 s.

## Applied config change

Kept run parameters were written back to `src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml`:

- `ndt_source_voxel_size`: 0.25
- `ndt_source_voxel_z_size`: 0.25
- `ndt_target_voxel_size`: 0.15
- `ndt_target_voxel_z_size`: 0.15
- `ndt_max_source_points`: 1400
- `ndt_max_iterations`: 40
- `ndt_resolution`: 0.8
- `ndt_step_size`: 0.08

## Next direction

This parameter-only change greatly reduces the stair/corner false match but does not meet `max < 0.5 m`. The remaining error is dominated by terminal/static-region offset. Next work should target terminal drift/loop-consistency rather than further reducing NDT resolution, because `0.7` resolution worsened full-bag metrics.
