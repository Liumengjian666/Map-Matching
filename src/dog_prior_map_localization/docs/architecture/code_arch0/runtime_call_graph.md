# Runtime call graph (static)

## Split NDT path

```text
main() [dog_prior_map_ndt_node.cpp:2188]
 └─ ros::init("dog_prior_map_ndt")
 └─ DogPriorMapNdtNode()
    ├─ getParam* → loadMap()
    │  ├─ PCD read → voxelDown(map) → voxelDown(target)
    │  └─ if schur_diagnostic_enable: build schur_target_tree_ once
    ├─ advertise NDT outputs
    └─ subscribe LiDAR/IMU/(prediction when enabled)

livoxCallback / pointCloud2Callback
 └─ convert + makeScanTiming
 └─ enqueueCloud
    ├─ pending queue ordered by reference stamp
    ├─ prediction/IMU watermark check
    └─ processPendingCloudsLocked
       └─ handleCloudLocked
          ├─ preprocess
          ├─ choose initial_guess (prediction, previous_pose*delta, current pose)
          ├─ optional local IMU rotation prior
          ├─ ndt_.align
          ├─ information / geometry / optional Schur diagnostics
          ├─ optional degeneracy projection
          ├─ optional step limit
          ├─ update previous_pose_, p_, R_
          ├─ publishPose / aligned cloud / diagnostics
          └─ writeDeterminismRow

predictionCallback → sorted prediction_history_ → processPendingCloudsLocked
imuCallback → bounded imu_history_ → processPendingCloudsLocked
```

## Split EKF observation path

```text
main() [dog_prior_map_ekf_node.cpp:4]
 └─ ros::init("dog_prior_map_ekf")
 └─ DogPriorMapEkfNode() [core.cpp:7]
    ├─ read params, initialize p/v/R/ba/bg/P
    ├─ optional map load (disabled by split launch)
    └─ subscribe IMU, NDT odom, degeneracy, information, optional image/LiDAR

imuCallback [imu_processor.cpp:7]
 ├─ append/prune imu_history_
 ├─ gravity init or propagateImu
 ├─ saveStateSnapshot (when OOSM enabled)
 ├─ publishState(high-rate)
 └─ maybePrintRuntime

ndtObservationCallback [vision_observation.cpp:746]
 ├─ validate absolute pose + information timestamp
 ├─ updateLocalizationMode
 ├─ findStateSnapshotAtOrBefore / rollback
 ├─ apply pose correction and optional reliable-subspace projection
 ├─ NDT-difference velocity blend
 ├─ replay IMU history to current state_stamp_
 ├─ publishState(corrected)
 └─ write OOSM/prediction lineage CSV

imageCallback [vision_observation.cpp:226]
 ├─ convert/quality statistics
 ├─ Shi-Tomasi + LK
 ├─ essential/affine relative rotation
 ├─ optional yaw update (legacy or directional state machine only)
 ├─ visual–IMU diagnostic / optional metric scale
 └─ updateLocalizationMode
```

## Integrated LiDAR path

```text
livoxCallback/pointCloud2Callback [lidar_matcher.cpp]
 └─ optional in-file IMU deskew
 └─ handleLidarCloud
    ├─ preprocessScan
    ├─ buildLocalSubmap
    ├─ runNdtRefinement if registration_method == ndt
    │  └─ local/full PCL NDT → absolute/incremental acceptance → p_/R_
    └─ otherwise lidarMapUpdate
       ├─ nearest-neighbor / PCA plane residuals
       ├─ robust weighted 6D solve
       ├─ degeneracy eigensystem/projector
       ├─ acceptance gate
       └─ applyPoseCorrection
    └─ publishState / cloud / diagnostics
```

The same class therefore owns two distinct LiDAR correction routes: integrated direct matching and split absolute NDT observation. This is the most important route ambiguity for future refactoring.
