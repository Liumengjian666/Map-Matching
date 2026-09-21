# Delivery runtime manifest

Status: historical manifest superseded by `FINAL_CODE_REACHABILITY.md`  
Repository: `https://github.com/Liumengjian666/fuxianFASTLIVO2`  
Branch: `feature/visual-factor-window`  
Baseline: `6f5a0d5b315fd5d77570327a3bd0b059ca5de2be`

The canonical runtime remains the split NDT + IMU/EKF chain. Current cleanup
and reachability decisions are recorded in `FINAL_CODE_REACHABILITY.md`.

## Canonical entry points

| Item | Canonical value |
|---|---|
| Launch | `launch/dog_prior_map_localization_split.launch` |
| Configuration | `config/dog_prior_map_localization_ndt.yaml` |
| RViz | `rviz/dog_prior_map_localization.rviz` when `rviz:=true` |
| Default build option | `DOG_PRIOR_BUILD_RESEARCH_TOOLS=OFF` |

The split launch starts the independent NDT node and the EKF node. Its validated
default semantics are:

- `camera_enable=false`
- `directional_fusion_enable=false`
- `state_machine_enable=false`
- `lidar_update/enable=false` in the EKF
- `map/load_in_ekf=false` in the canonical launch
- NDT prediction disabled; local IMU gyro rotation prior enabled
- OOSM correction enabled
- existing future-deferral argument semantics preserved

## Runtime executables and source ownership

| Executable | Runtime role | Primary source |
|---|---|---|
| `dog_prior_map_ndt_node_cpp` | Livox scan preprocessing, prior-map NDT, NDT diagnostics and `/dog_livo/ndt_odom` | `src/dog_prior_map_ndt_node.cpp` |
| `dog_prior_map_ekf_node_cpp` | IMU propagation, NDT observation correction, OOSM replay, output and TF | `src/dog_prior_map_ekf_node.cpp`, `src/dog_prior_map_ekf_node_core.cpp`, `src/imu_processor.cpp`, `src/ros_output.cpp`, `src/core/state_history.cpp`, `src/core/oosm_replay_planner.cpp` |

The EKF executable no longer compiles the removed integrated LiDAR/map
translation units. The independent NDT node owns prior-map loading and
matching; the EKF consumes only its timestamped odometry observation.

`oosm_replay_planner_contract_test` is a ROS-free contract test, not a runtime
node.

## Inputs

- `/livox/lidar`
- `/livox/imu`
- `map/pcd_fallback_path` or `map/npz_path` from the canonical YAML
- camera topics are not part of the delivery path

## Outputs

- `/dog_livo/ndt_odom`
- `/dog_livo/odom_high_rate`
- `/dog_livo/odom_corrected`
- `/dog_livo/path_high_rate`
- `/dog_livo/path_corrected`
- `/dog_livo/diagnostics`
- `/dog_livo/prior_map`, `/dog_livo/filtered_points`, `/dog_livo/points_aligned`
- current-state TF (the corrected-state duplicate TF path is disabled)

## Dependencies

The current package build requires the ROS components in `package.xml`: roscpp,
rospy, sensor_msgs, nav_msgs, geometry_msgs, std_msgs, diagnostic_msgs,
cv_bridge, tf2_ros, tf, pcl_ros, pcl_conversions, and livox_ros_driver2. Eigen
and PCL remain build dependencies for the core/NDT targets. The Python fallback
dependency remains only for the historical user-owned scripts described below.

The offline research probes use Eigen and PCL and are not linked to either
runtime executable. They are only added to the build when
`DOG_PRIOR_BUILD_RESEARCH_TOOLS=ON`.

## Legacy and research surface

### Legacy entry points

- `launch/dog_prior_map_localization.launch`: integrated historical launch;
  retained and marked `LEGACY / NOT DELIVERY ENTRYPOINT`.
- `scripts/dog_prior_map_ekf_node.py`: Python fallback source; retained for
  reference but no longer installed by the default CMake install surface.
- `config/dog_prior_map_localization_deploy_light_odom.yaml`: referenced only
  by historical loop3/loop5 scripts using the separate `dog_light_loc_ws`
  workspace.
- The unused light-odom, old integrated, and KISS external-prior profiles were
  removed; `dog_prior_map_localization_ndt.yaml` is the only canonical config.

### Research-only tools

- `tools/offline/ndt_uncertainty_probe.cpp`
- `tools/offline/ndt_direction_alignment_probe.cpp`

Both are offline CSV/PCD analysis tools. They are excluded from the default
build and install surface, but remain available through the explicit research
option.
