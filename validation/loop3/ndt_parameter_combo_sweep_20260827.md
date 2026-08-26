# NDT Parameter Combination Sweep - 2026-08-27

Goal: identify and test parameter combinations that might remove the remaining two small corner mismatches without using dataset pose topics as runtime priors.

Runtime inputs remain `/livox/lidar`, `/livox/imu`, and prior map. FASTLIO2Location `/localization` is used only for offline evaluation.

## Parameters that can affect the corner mismatches

Source cloud:
- `lidar_update/scan_voxel_size`
- `lidar_update/max_scan_points`
- `lidar_update/ndt_source_voxel_size`
- `lidar_update/ndt_source_voxel_z_size`
- `lidar_update/ndt_max_source_points`
- `lidar_update/ndt_multiframe_source_enable`

Target/prior map:
- `map/voxel_size`
- `lidar_update/ndt_target_voxel_size`
- `lidar_update/ndt_target_voxel_z_size`
- `lidar_update/ndt_max_target_points`
- `lidar_update/ndt_use_full_map_target`
- `lidar_update/ndt_local_target_radius`

NDT optimizer:
- `lidar_update/ndt_resolution`
- `lidar_update/ndt_step_size`
- `lidar_update/ndt_max_iterations`
- `lidar_update/ndt_transformation_epsilon`

Motion and acceptance constraints:
- `lidar_update/ndt_step_limit_max_translation`
- `lidar_update/ndt_step_limit_max_rotation_deg`
- `lidar_update/ndt_acceptance_enable`
- `lidar_update/ndt_accept_max_translation`
- `lidar_update/ndt_accept_max_rotation_deg`
- `lidar_update/ndt_large_jump_guard_enable`
- `lidar_update/ndt_use_external_initial_guess`
- `lidar_update/internal_odom_enable`

## Baseline retained

Current best default config:
- `ndt_source_voxel_size: 0.25`
- `ndt_target_voxel_size: 0.15`
- `ndt_max_source_points: 1400`
- `ndt_resolution: 0.8`
- `ndt_step_size: 0.08`
- `ndt_step_limit_max_translation: 0.5`
- `ndt_step_limit_max_rotation_deg: 5.0`

Best full-bag run:
- `/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`
- mean 0.417 m, p95 1.535 m, max 2.694 m.

## Combination experiments rejected

- `ndt_combo_source1800_rot3_loop3_20260827_041223`: `ndt_max_source_points=1800`, `ndt_step_limit_max_rotation_deg=3.0`. Catastrophic wrong attraction: mean 27.142 m, p95 100.577 m, max 109.748 m.
- `ndt_combo_source1800_target018_loop3_20260827_042255`: `ndt_max_source_points=1800`, `ndt_target_voxel_size=0.18`, `ndt_target_voxel_z_size=0.18`. Catastrophic wrong attraction: mean 7.956 m, p95 53.162 m, max 60.203 m.
- `ndt_combo_source1800_trans045_loop3_20260827_043322`: `ndt_max_source_points=1800`, `ndt_step_limit_max_translation=0.45`. Catastrophic wrong attraction: mean 15.571 m, p95 78.759 m, max 84.754 m.
- `ndt_combo_source1200_default_loop3_20260827_044355`: `ndt_max_source_points=1200`. Catastrophic wrong attraction: mean 12.816 m, p95 61.176 m, max 70.456 m.
- `ndt_combo_stepsize005_default_loop3_20260827_045428`: `ndt_step_size=0.05`. Wrong attraction in later segment: mean 3.074 m, p95 27.926 m, max 40.079 m.

## Conclusion

No tested parameter-only combination improves the current default. In this dataset, point count and optimizer parameters are highly non-monotonic: they can reduce one corner error while making another repeated-structure attractor much stronger.

Keep the current default config unchanged. Further improvement should be code-level candidate verification: compare multiple local hypotheses by a combined cost of NDT score, nearest-map residual, and distance from the propagated motion prior.
