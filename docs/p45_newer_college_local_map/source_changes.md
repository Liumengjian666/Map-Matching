# P45 source changes

- `src/dog_prior_map_localization/include/dog_prior_map_localization/local_map_tiles.hpp` and `src/local_map_tiles.cpp`: metadata-only tile index, causal projected-source coverage selection, bounded active target cache, SHA/count validation, hysteresis and fail-closed update.
- `src/dog_prior_map_localization/tools/offline/build_ndt_map_tiles.cpp`: offline P44-equivalent target preprocessing and 20 m XY partition with manifest, hashes, counts and point-multiset fingerprint.
- `src/dog_prior_map_localization/tools/offline/p45_local_map_switch_probe.cpp`: finite component-only remote tile replacement probe; no NDT align and no GT read.
- `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`: use the actual current-frame causal NDT initial guess to request target coverage before this frame’s `align`; call `setInputTarget` only for an update; emit map-selection, timing and memory diagnostics; no full-map fallback in local mode.
- `src/dog_prior_map_localization/config/p45_newer_college_local_map.yaml`, `launch/newer_college_local_map.launch`, `rviz/p45_newer_college_local_map.rviz`: P45 overlay, local-mode one-click launch and visualization-only full-map process.
- `src/dog_prior_map_localization/scripts/ensure_output_dir_exec.py`: create configured run output directory and `exec` the original ROS process. The split launch defaults both new output-directory args to empty, preserving old consumers. The P45 launch invokes the helper via `rosrun`, which resolves the installed executable.
- `src/dog_prior_map_localization/scripts/monitor_p45_local_map_run.py`: process-tree monitoring for the P45 headless wrapper. The formal run used an earlier monitor revision that failed to account for ROS child process groups; its aggregate CPU/resource summary is excluded.
- `src/dog_prior_map_localization/CMakeLists.txt`: add tile manager/builder/probe and install the Python helpers.
- `docs/p44_newer_college_300frame/evaluate_trajectory.py`: correct 1 s RPE interval advancement and add frozen-output hash checking; ATE values remain unchanged.

No source changes were made in either protected dog-robot workspace. No NDT objective/optimizer, EKF/IMU model, initialization, OOSM, Deskew, NDT parameter, DCReg or recovery strategy was changed.
