# Bounded official-source evidence

All local public-source bytes inspected have Git identities and SHA256 in
`source_manifest.json`. Negative findings are restricted to these files/trees,
not a claim that the entire Internet has been exhaustively searched.

| Source | Exact evidence | Interpretation |
|---|---|---|
| [Ground-Challenge](https://github.com/sjtuyinjie/Ground-Challenge/tree/46a7c19a8672cc9246ad8b14815b4006a374fae0) | README sensor/topic list; `calibration.txt:17–34` | VLP-16 and Xsens exist, but listed transforms are `body_T_cam0` and `body_T_wheel`, no `T_I_L` |
| Same repo | `config_files_gc/viwfusion/groundchallenge.yaml:11,24–43` | `/camera/imu`, camera extrinsic; not Xsens–LiDAR |
| [Ground-Fusion](https://github.com/SJTU-ViSYS/Ground-Fusion/tree/665cecd17964d929fb9fa4c3a01e9c5b8cd2f7cb) | `config/realsense/groundchallenge.yaml:20,32–63` | Same RealSense IMU/camera chain; no LiDAR transform |
| [Ground-Fusion++](https://github.com/sjtuyinjie/Ground-Fusion2/tree/aa1c8dc80b05df7a536f5759d2f7725ad12015a6) | `Ground-Fusion++/config/realsense/groundchallenge.yaml:20,32–63`; LIO config tree | Ground-Challenge VIO file exists; no Ground-Challenge LIO file in enumerated tree |
| Same repo | `Ground-Fusion++/lio/config/m2dgr.yaml:9–17` | `/handsfree/imu`, translation `[0.27255,-0.00053,0.17954]`; **REJECTED: different dataset/IMU** |
| Same repo | `.../lio/config/m2dgrp.yaml:9–20` | `/camera/imu`, translation `[0,-0.55,-0.03]`; **REJECTED: different platform/sensor chain** |
| Same repo | `.../lio/config/m3dgr.yaml:9–18` | Livox topic; **REJECTED: not Corridor2 VLP-16/Xsens** |
| [Dataset paper](https://arxiv.org/html/2307.03890), III-A/B | Separate RGBD embedded IMU and Xsens; software synchronization; camera–IMU calibration discussion | Does not give a complete numerical LiDAR–Xsens transform in inspected sections; synchronization description does not certify zero residual lag |
| Same repo `fig/robot.jpg` | Visually inspected platform photo | Sensor locations visible; no metrological 6DoF calibration or certified IMU height recoverable from a photograph |
| [Issue #10](https://github.com/sjtuyinjie/Ground-Challenge/issues/10), [#11](https://github.com/sjtuyinjie/Ground-Challenge/issues/11) | Titles visible in official issue index; issue body/comment API returns HTTP 403 | Discovery of questions only, not evidence of absence of an author answer |

The [Ground-Fusion paper](https://arxiv.org/abs/2402.14308) concerns a lidar-less
fusion framework; [Ground-Fusion++ paper](https://arxiv.org/abs/2507.08364) introduces
the M3DGR framework/benchmark. Neither title/platform correspondence makes its
other-dataset numerical calibration applicable to Ground-Challenge Corridor2.

No third-party calibration was imported. No pseudo-GT files were opened.
