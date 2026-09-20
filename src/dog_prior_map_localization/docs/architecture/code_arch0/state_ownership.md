# State ownership audit

Current state is declared almost entirely in `include/dog_prior_map_localization/dog_prior_map_ekf_node.hpp` and then directly read/written by member functions in multiple translation units. ROS callbacks are serialized by `ros::spin()`; each node also uses `mutex_`, so the effective owner is the node instance, not an algorithm object.

| State | Declared | Written by | Read by | Lifetime / callback owner | Risk |
|---|---|---|---|---|---|
| `p_, v_, R_, ba_, bg_, P_` | EKF header lines 255–261 | `imu_processor.cpp::propagateImu`, `lidar_matcher.cpp::applyPoseCorrection` and integrated NDT direct assignment, `vision_observation.cpp::ndtObservationCallback`/visual yaw, `restoreStateSnapshot` | all matching, visual, output, histories | process lifetime; `imu`, integrated LiDAR, NDT observation, image callbacks | multiple direct writers; measurement policy mixed with state |
| `state_stamp_, last_imu_time_` | header 230–235 | `imu_processor.cpp::imuCallback`, NDT OOSM replay | snapshots, NDT observation, output | EKF callback mutex | sensor-time semantics are implicit |
| `state_history_` | header 292 | save/prune/restore/erase in `imu_processor.cpp`; OOSM code in vision file | `ndtObservationCallback` | bounded by `imu_history_keep_sec_`; EKF only | rollback logic lives in visual file |
| `imu_history_` (EKF) | header 289 | `imu_processor.cpp::imuCallback` | deskew, integrate delta, OOSM replay, visual diagnostics | bounded deque; EKF callback mutex | used by LiDAR and visual code without interface |
| `map_cloud_, map_kdtree_` (integrated) | header 553–554 | `map_loader.cpp::loadPriorMap` | `buildLocalSubmap`, `lidarMapUpdate`, cloud output | process lifetime; map load once | map ownership inside EKF class |
| NDT `p_, R_`, `previous_pose_`, `delta_pose_`, `prediction_history_`, `pending_lidar_frames_` | private fields in `dog_prior_map_ndt_node.cpp` lines 2053+ | NDT constructor/callbacks/`handleCloudLocked` | NDT timing/align/diagnostic methods | NDT mutex; queues/history bounded | independent duplicate state model from EKF |
| NDT `map_cloud_, target_cloud_, schur_target_tree_` | NDT private fields | `loadMap` only | NDT align and Schur | process lifetime | target map + diagnostic tree share allocation policy |
| visual frontend `last_gray_`, `last_features_`, `last_image_*` | EKF header lines 480+ | `imageCallback`, `applyVisualYawCorrection`, visual diagnostic | image callback and `ros_output.cpp` | image callback under EKF mutex | frontend state and measurement application in same class |
| visual telemetry `last_visual_*` | EKF header lines 490–529 | `imageCallback`, `applyVisualYawCorrection`, `updateVisualImuDiagnostic` | `publishDiagnostics`, runtime CSV | process lifetime | diagnostic flags can control legacy/directional paths |
| directional projectors/mode counters | header 318–347 | `lidarInformationCallback`, `rebuildDirectionalProjectors`, `updateLocalizationMode` | NDT observation and image update | only active when switches enabled | dormant but coupled to runtime update decisions |
| Schur result | local `SchurObservabilityResult` | `handleCloudLocked` local scope | determinism CSV only | per scan | good isolation except target tree lifetime |

## Hidden sharing / direct-writer findings

- The EKF class is the sole nominal state owner, but five different source files can mutate the same nominal state. There is no measurement object or update API enforcing invariants.
- In split mode the NDT process owns a second pose/temporal state and the EKF owns a third high-rate/replay state. The ROS message boundary is the only formal contract.
- `ndt_observation_velocity_blend_` turns NDT pose differences into `v_`, making a diagnostic-looking output topic influence future IMU propagation.
- `lidar_information` is diagnostic telemetry until `fusion/directional_enable && state_machine_enable`; then it changes update projection. `lidar_degeneracy` similarly feeds legacy visual/state decisions.
- `runtime.csv` and DiagnosticArray are read-only outputs, but they are emitted inside callbacks and can add wall-time cost when enabled.
