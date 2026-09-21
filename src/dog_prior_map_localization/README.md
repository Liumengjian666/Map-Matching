# dog_prior_map_localization

FAST-LIVO2 offline prior-map localization for a quadruped platform. The
validated delivery path uses an independent NDT map matcher, IMU propagation,
and an EKF that applies timestamped NDT corrections with OOSM rollback/replay.

## Canonical delivery runtime

Use the split launch. It starts the two runtime executables and, optionally,
the project RViz configuration:

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/dog_visual_loc_ws/devel/setup.bash

roslaunch dog_prior_map_localization \
  dog_prior_map_localization_split.launch \
  rviz:=true
```

The canonical configuration is:

```text
config/dog_prior_map_localization_ndt.yaml
```

The runtime data flow is:

```text
/livox/lidar + prior map -> dog_prior_map_ndt -> /dog_livo/ndt_odom
/livox/imu -------------------------------> dog_prior_map_ekf
                                             |
                                             +-> /dog_livo/odom_high_rate
                                             +-> /dog_livo/odom_corrected
                                             +-> TF and diagnostic topics
```

The canonical launch keeps the validated delivery semantics: camera and
directional fusion are disabled by default, the integrated EKF LiDAR matcher
is disabled, NDT prediction is disabled in favor of the local IMU rotation
prior, and OOSM correction remains enabled. Existing launch arguments can be
used for diagnostics without changing those defaults.

## Main topics

Inputs:

- `/livox/imu`
- `/livox/lidar`
- the prior PCD/NPZ map configured under `map/`

Outputs:

- `/dog_livo/ndt_odom`: low-rate independent NDT pose
- `/dog_livo/odom_high_rate`: IMU-propagated EKF pose
- `/dog_livo/odom_corrected`: EKF pose after NDT correction/OOSM replay
- `/dog_livo/path_high_rate` and `/dog_livo/path_corrected`
- `/dog_livo/diagnostics`, `/dog_livo/lidar_degeneracy`, and related map/debug topics
- TF for the current high-rate state

## Build

The normal build excludes offline research probes:

```bash
source /opt/ros/noetic/setup.bash
cd /home/jian/livox_ws/dog_visual_loc_ws
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=OFF \
  --pkg dog_prior_map_localization
```

The two runtime executables are:

- `dog_prior_map_ndt_node_cpp`
- `dog_prior_map_ekf_node_cpp`

The ROS-free `oosm_replay_planner_contract_test` is a contract test, not a
runtime node.

## Research and experimental modules

The following code is retained for future experiments but is not part of the
default delivery runtime or default build:

- vision frontend and visual fusion code
- LiDAR degeneracy/directional-fusion code
- Schur and uncertainty diagnostics
- `tools/offline/ndt_uncertainty_probe.cpp`
- `tools/offline/ndt_direction_alignment_probe.cpp`

To build the two offline probes explicitly:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=ON \
  --pkg dog_prior_map_localization
```

They consume offline CSV/PCD inputs and are never linked into either runtime
executable.

## Legacy entry points

`launch/dog_prior_map_localization.launch` is retained for historical
integrated-mode experiments and is marked `LEGACY / NOT DELIVERY ENTRYPOINT`.
The Python `dog_prior_map_ekf_node.py` fallback is also retained as source for
reference, but is no longer installed by the default package build. The old
YAML profiles in `config/` remain in the repository until a later reference
cleanup confirms that they can be archived safely.

## Map conversion

For a faster NPZ map when desired:

```bash
rosrun dog_prior_map_localization pcd_to_npz_map.py \
  --input /home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd \
  --output /home/jian/rosbag/loop2/loop2mapping/pcd/loop2_prior_map_voxel_0p30.npz \
  --voxel 0.30
```
