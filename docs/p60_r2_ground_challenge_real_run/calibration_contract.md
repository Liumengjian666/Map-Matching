# Ground-Challenge sensor transform closure

Convention: `T_A_B` maps coordinates from B into A.

Actual cloud frame: `sensor`. Actual Xsens IMU frame: `imu`.

Official metadata checkout: `/home/jian/ground-challenge/official_metadata_full`, commit `46a7c19a8672cc9246ad8b14815b4006a374fae0`.

Official calibration source: <https://github.com/sjtuyinjie/Ground-Challenge/blob/46a7c19a8672cc9246ad8b14815b4006a374fae0/calibration.txt>.

Local calibration: `/home/jian/ground-challenge/calibration.txt`, 932 bytes, SHA256 `f8f591c46fa21c88d3986e65bf0558c7f79f2830b017165ec86b87190f745e08`.

Inspected configuration groups: `config_files_gc/vinsmono`, `vinsrgbd`, `viwfusion`, `tartanvo`. They provide visual/wheel relationships. `body_T_cam0` is not a LiDAR–IMU transform. Even a valid camera/body calibration cannot connect an absent LiDAR extrinsic.

Full bag TF edge inventory:

```
odom -> base_footprint
world -> map
base_link -> fl_castor_link -> fl_wheel_link
base_link -> fr_castor_link -> fr_wheel_link
base_link -> rl_castor_link -> rl_wheel_link
base_link -> rr_castor_link -> rr_wheel_link
base_link -> left_wheel_link
base_link -> right_wheel_link
```

Neither `sensor` nor `imu` appears in these edges. There is no `/tf_static` topic. Thus no TF chain supplies `T_imu_lidar`.

Required chain for fused localization:

`T_map_imu = T_map_lidar * inverse(T_imu_lidar)`.

Both the rotation and lever arm of `T_imu_lidar` remain unknown. They were not replaced with identity or another dataset's calibration. No calibration was estimated from the weakly excited corridor sequence, and no GT was fitted.

The raw first-scan registration map only defines `T_map_lidar_scan1=I` by construction. It does not determine `T_imu_lidar` or `T_map_imu` and does not supply an official map-to-pseudoGT transform.
