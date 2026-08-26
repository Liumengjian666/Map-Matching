# NDT Corner Parameter Trials - 2026-08-27

Goal: respond to the two remaining small corner mismatches by testing parameter-only combinations around point count, target-map voxel size, optimizer resolution, and maximum per-frame motion limits. Runtime inputs remain `/livox/lidar`, `/livox/imu`, `/clock`, and the prior map only. FASTLIO2Location `/localization` is used only for offline full-bag evaluation.

## Parameters considered

Likely influential parameters in `src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml`:

- Source density: `lidar_update/ndt_source_voxel_size`, `lidar_update/ndt_source_voxel_z_size`, `lidar_update/ndt_max_source_points`, `lidar_update/scan_voxel_size`, `lidar_update/max_scan_points`.
- Prior-map density: `map/voxel_size`, `lidar_update/ndt_target_voxel_size`, `lidar_update/ndt_target_voxel_z_size`, `lidar_update/ndt_max_target_points`.
- Target selection: `lidar_update/ndt_use_full_map_target`, `lidar_update/ndt_local_target_radius`, `lidar_update/ndt_local_target_min_points`.
- NDT optimizer: `lidar_update/ndt_resolution`, `lidar_update/ndt_step_size`, `lidar_update/ndt_max_iterations`, `lidar_update/ndt_transformation_epsilon`.
- Motion constraints: `lidar_update/ndt_step_limit_max_translation`, `lidar_update/ndt_step_limit_max_rotation_deg`, `lidar_update/ndt_acceptance_enable`, `lidar_update/ndt_accept_max_translation`, `lidar_update/ndt_accept_max_rotation_deg`.
- Experimental priors kept disabled: `lidar_update/ndt_use_external_initial_guess`, `lidar_update/internal_odom_enable`, `lidar_update/ndt_large_jump_guard_enable`, `lidar_update/ndt_confident_step_limit_enable`.

## Baseline retained

Current best default remains unchanged:

- Run: `/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`
- Key parameters: source voxel `0.25`, target voxel `0.15`, source points `1400`, NDT resolution `0.8`, step size `0.08`, max step translation `0.5`, max step rotation `5 deg`.
- Full-bag vs FASTLIO2Location: mean `0.417 m`, p95 `1.535 m`, max `2.694 m`.

Baseline corner windows:

| Window | mean | p95 | max |
| --- | ---: | ---: | ---: |
| 345-351 s | 0.755 m | 2.321 m | 2.429 m |
| 398-403 s | 0.451 m | 1.465 m | 1.781 m |
| 565-576 s | 1.717 m | 2.581 m | 2.694 m |

## New full-bag trials

Temporary configs are saved under `/home/jian/rosbag/loop3/param_configs/`.

| Run | Changed parameters | mean | p95 | max | Decision |
| --- | --- | ---: | ---: | ---: | --- |
| `ndt_corner_try_source1600_trans040_rot4_loop3_20260827_0508` | source points `1600`, max translation `0.40`, max rotation `4 deg` | 13.754 m | 61.476 m | 71.560 m | reject |
| `ndt_corner_try_source1400_target014_res085_loop3_20260827_0520` | target voxel `0.14`, NDT resolution `0.85` | 0.718 m | 2.856 m | 14.549 m | reject |
| `ndt_corner_try_source1400_target016_res08_loop3_20260827_0535` | target voxel `0.16` | 9.415 m | 62.983 m | 75.914 m | reject |

Window details for the two completed trajectories where the bag was still available before cleanup:

| Run | 345-351 s max | 398-403 s max | 565-576 s max |
| --- | ---: | ---: | ---: |
| baseline | 2.429 m | 1.781 m | 2.694 m |
| source1600 + trans040 + rot4 | 1.566 m | 27.754 m | 70.171 m |
| target014 + res085 | 2.724 m | 4.173 m | 4.484 m |

The large rejected result bags were removed after extracting metrics to preserve disk space; the run folders, logs, eval JSON/TXT, and temporary configs were retained.

## Conclusion

Parameter-only tuning did not remove the last two corner mismatches. The new trials reinforce the previous conclusion: global point count, target voxel size, NDT resolution, and simple step limits are non-monotonic on this repeated-corner map. A setting may reduce one corner but strongly increases wrong attractors later.

Keep `src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml` unchanged. The next useful change should be code-level selective candidate verification around risky frames, not another broad scalar parameter sweep.
