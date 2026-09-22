# FINAL-HYGIENE-1 Reachability Audit

This audit starts from `b0b14755bda27473519ea538ad97c0c6361ddeb4` on
`feature/visual-factor-window`.  The canonical delivery path is the split
launch: independent `dog_prior_map_ndt_node_cpp` followed by
`dog_prior_map_ekf_node_cpp`.

## Runtime-required

- `src/dog_prior_map_ndt_node.cpp`: PCD loading, Livox/PointCloud2 input,
  local IMU gyro prior, NDT alignment, step limiting, and NDT diagnostics.
- `src/dog_prior_map_ekf_node.cpp` and `src/dog_prior_map_ekf_node_core.cpp`:
  EKF construction, parameter loading, and ROS wiring.
- `src/imu_processor.cpp`: IMU propagation and sensor-time IMU history.
- `src/fusion/ndt_observation.cpp`: external NDT observation correction,
  bounded future deferral, and OOSM rollback/replay dispatch.
- `src/core/state_history.cpp` and `src/core/oosm_replay_planner.cpp`:
  ROS-free state-history storage and OOSM plan construction.
- `src/ros_output.cpp`: odometry, Path, TF, runtime and OOSM output.
- `config/dog_prior_map_localization_ndt.yaml` and
  `launch/dog_prior_map_localization_split.launch`.

The active executables are exactly the independent NDT node and the IMU/EKF
node.  The core and fusion files are translation units of the EKF executable,
not additional ROS nodes.

## Validation-required

- `scripts/run_dataset_evaluation.sh`
- `scripts/evaluate_runtime_and_accuracy.py`
- `scripts/compare_odom_to_reference.py`
- `tests/oosm_replay_planner_contract_test.cpp`

The user-owned `scripts/run_loop2_determinism_trial.sh` is retained without
editing because it was already staged before this Stage.

## Offline-useful

- `scripts/build_livox_keyframe_scan_db.py`
- `tools/offline/ndt_uncertainty_probe.cpp`
- `tools/offline/ndt_direction_alignment_probe.cpp`
- translation/document tooling and historical validation reports.

These do not enter either runtime executable.

## Legacy-dead and removable

The following paths are not used by the canonical split runtime, CMake runtime
targets, or the formal final loop2 validation protocol:

- the Python integrated EKF fallback;
- the integrated launch that can select that fallback;
- the deploy-light-odom YAML and its old loop3/loop5 launch helpers;
- the unused PCD-to-NPZ conversion helper, because the canonical C++ runtime
  reads `map/pcd_fallback_path` and has no NPZ reader.

Their removal does not change the split runtime.  Git history retains the old
implementation.

## User-dirty deferred

`launch/dog_prior_map_localization_split.launch` is dirty in the user's
working copy and is not modified by this Stage.  Its historical diagnostic
arguments and `map/load_in_ekf` parameter are therefore deferred rather than
silently rewritten.  The same protection applies to the pre-existing staged
files recorded in `/tmp/final_hygiene1_staged_before.txt`.

## NDT prediction reachability

The canonical runtime uses `previous_pose * delta_pose` plus the local IMU
gyro rotation prior.  The former EKF high-rate pose feedback subscriber,
timestamped prediction history, and prediction watermark were unreachable
from the formal delivery path and are now removed from the source and YAML.
This Stage retains the IMU watermark, pending LiDAR queue, scan timing, local
gyro integration, step limiter, and determinism fields needed to reproduce
NDT.
