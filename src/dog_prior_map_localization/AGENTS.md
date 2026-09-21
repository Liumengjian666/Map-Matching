# AGENTS.md

## Project overview

This repository is a ROS 1 / Catkin package for low-cost prior-map localization on quadruped robots. The system fuses IMU propagation with LiDAR-to-prior-map matching, exposes corrected and high-rate odometry topics, and is designed to run on a robot with a prebuilt map.

Primary references:
- [README.md](README.md)
- [CMakeLists.txt](CMakeLists.txt)
- [launch/dog_prior_map_localization_split.launch](launch/dog_prior_map_localization_split.launch)
- [config/dog_prior_map_localization_ndt.yaml](config/dog_prior_map_localization_ndt.yaml)

## Build and run

Use the standard Catkin workflow from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
cd /home/jian/livox_ws/dog_visual_loc_ws
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg dog_prior_map_localization
source devel/setup.bash
roslaunch dog_prior_map_localization dog_prior_map_localization_split.launch rviz:=false
```

For offline evaluation or bag playback, follow the commands in [README.md](README.md). The package loads YAML parameters from the config directory and runs the C++ node by default.

## Repo structure and conventions

- `src/` contains the main C++ implementation.
  - `dog_prior_map_ekf_node.cpp` is the main executable entry point.
  - `dog_prior_map_ekf_node_core.cpp` holds the core node logic.
- `imu_processor.cpp`, `fusion/ndt_observation.cpp`, and the core files are feature-specific modules.
- `config/` stores ROS params and tuning profiles for different localization modes.
- `launch/` contains launch files for robot runtime and split-node setup.
- `scripts/` contains Python utilities for map conversion, evaluation, and dataset runs.
- `docs/` contains project reports and architecture notes; prefer them for algorithm intent and tuning history before changing behavior.

## Working conventions for agents

- Prefer minimal, targeted edits that match the existing ROS/Catkin and C++ style.
- Preserve ROS topic names, parameter names, and node names unless the change explicitly requires them to move.
- Keep C++ changes compatible with OpenCV, Eigen3, and PCL-based matching code and avoid introducing non-ROS dependencies without a clear need.
- If a change affects localization behavior, validate against the relevant launch + config pair and check whether the tuning YAMLs need to remain consistent.
- For map conversion or data-evaluation work, prefer the existing scripts in `scripts/` over creating new ad hoc pipelines.

## Change guidance

When investigating or editing this repo, start with these files based on the area:

- Parameter and launch behavior: [launch/dog_prior_map_localization_split.launch](launch/dog_prior_map_localization_split.launch), [config/dog_prior_map_localization_ndt.yaml](config/dog_prior_map_localization_ndt.yaml)
- Core node startup: [src/dog_prior_map_ekf_node.cpp](src/dog_prior_map_ekf_node.cpp)
- Matching and prior-map logic: [src/lidar_matcher.cpp](src/lidar_matcher.cpp), [src/map_loader.cpp](src/map_loader.cpp)
- IMU, TF, and output flow: [src/imu_processor.cpp](src/imu_processor.cpp), [src/ros_output.cpp](src/ros_output.cpp)
- Algorithm intent and historical context: [docs](docs)

## Important caveat

This project is tuned around ROS Noetic, Catkin, and a specific dataset/robot configuration. Do not assume a generic C++ or Python project structure; keep any edits aligned with the package layout and the existing ROS runtime assumptions.
