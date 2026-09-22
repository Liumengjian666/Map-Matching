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

The canonical launch is deliberately a small delivery chain: there is no
active camera frontend, visual update, directional fusion, or integrated EKF
LiDAR matcher. NDT prediction is disabled in favor of the local IMU rotation
prior, and OOSM correction remains enabled. The remaining diagnostic switches
are opt-in and do not change the delivery state by default.

## Main topics

Inputs:

- `/livox/imu`
- `/livox/lidar`
- the prior PCD map configured under `map/pcd_fallback_path`

Outputs:

- `/dog_livo/ndt_odom`: low-rate independent NDT pose
- `/dog_livo/odom_high_rate`: IMU-propagated EKF pose
- `/dog_livo/odom_corrected`: EKF pose after NDT correction/OOSM replay
- `/dog_livo/path_high_rate` and `/dog_livo/path_corrected`
- `/dog_livo/diagnostics` and related map/debug topics
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

The final loop2 acceptance record is in
[`docs/validation/FINAL_DELIVERY_ACCEPTANCE_20260922.md`](docs/validation/FINAL_DELIVERY_ACCEPTANCE_20260922.md).
It includes the two full replays, reference-deviation metrics, resource
samples, and the known user-owned dirty-file boundary.

## Research and experimental modules

The following offline material is retained for future experiments but is not
part of the default delivery runtime or default build:

- `tools/offline/ndt_uncertainty_probe.cpp`
- `tools/offline/ndt_direction_alignment_probe.cpp`

The former vision and directional-fusion runtime branches are not active in
the delivery package. Future sensor work must enter through a separate typed
measurement module; it must not reintroduce direct access to EKF internals.

To build the two offline probes explicitly:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=ON \
  --pkg dog_prior_map_localization
```

They consume offline CSV/PCD inputs and are never linked into either runtime
executable.

## Legacy entry points

The canonical entry point is the split launch above. The old integrated launch,
Python fallback, and deploy-light-odom helpers are not part of the canonical
runtime. Files that are still referenced by user-owned staged evaluation
scripts remain explicitly deferred and are tracked as
`DEFERRED_DUE_TO_USER_STAGED_WORK` in
`docs/architecture/FINAL_CODE_REACHABILITY.md`.

The unused light-odom, KISS external-prior, and old integrated YAML profiles
have been removed. `dog_prior_map_localization_ndt.yaml` is the only canonical
configuration.

## Map format

The delivery NDT node loads the configured PCD map directly. The removed NPZ
conversion helper was not part of the runtime path; existing historical NPZ
files may remain as external data, but changing or regenerating them is not
required for the canonical launch.
