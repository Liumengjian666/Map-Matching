# Map and frame contract status

No official prior-map PCD is listed for Ground-Challenge Corridor1/2 or the SubT-MRS ICCV Long_Corridor row. No independently acquired same-scene map was found in the official sources reviewed for this bounded task.

Ground-Challenge's official repository provides `calibration.txt` and `psudo_gt`, but no map. The calibration file specifies camera and wheel relationships, not an explicit LiDAR–IMU transform. Since the bag could not be downloaded, the lidar/IMU frame IDs and relation between `psudo_gt` and any potential map cannot be checked.

The SubT-MRS Long_Corridor row provides a recording and extrinsics, not an official map. The bag download did not complete. The extrinsics YAML includes `laser_to_imu`, but the direction and precise frame contract remain unconfirmed. The public GT listing is access-limited and no GT was read.

Accordingly, no `T_map_lidar`, `T_map_imu`, or map-to-reference transform is established. No ATE/RPE is valid. A future same-recording map could only be called `SELF_MAP_PIPELINE_DIAGNOSTIC`, and cannot support independent-map accuracy or cross-pass generalization claims. P56 did not construct such a map because no complete sensor recording was available.
