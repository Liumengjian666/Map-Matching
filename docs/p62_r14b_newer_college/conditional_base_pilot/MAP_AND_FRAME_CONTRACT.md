# Frozen conditional coordinate chain

T_A_B maps B into A, column vectors, rightmost transform applied first.
M=official Leica frame; B=robotics Base; C=left optical; I=Ouster IMU; L=official Ouster point cloud frame.
The sole predeclared interpretation is T_CSV=T_M_B.
T_M_L=T_M_B T_B_C T_C_I T_I_L. Base and optical share origin, not rotation.
T_B_C is the fixed robotics/optical axis rotation with zero translation. T_C_I is cam0.T_cam_imu from the frozen official Kalibr YAML. T_I_L is the explicitly labelled LiDAR-to-IMU transform, meters, xyzw quaternion. The separately supplied inverse T_L_I was checked. No fitted transformation, manual orientation search or alternative convention was run.

All calibration rotations passed orthogonality/determinant and inverse tests at preparation; actual float native SE3 is checked with the frozen R10 guard. Quaternion normalization is limited to recorded decimal-roundoff, not calibration optimization.

T_B_C =
```json
[
  [
    0.0,
    0.0,
    1.0,
    0.0
  ],
  [
    -1.0,
    0.0,
    0.0,
    0.0
  ],
  [
    0.0,
    -1.0,
    0.0,
    0.0
  ],
  [
    0.0,
    0.0,
    0.0,
    1.0
  ]
]
```

T_C_I =
```json
[
  [
    0.7099216283860768,
    -0.7041416662469333,
    0.013992691115502887,
    0.015627525086609025
  ],
  [
    0.02460002682200807,
    0.004936229792296243,
    -0.9996851866041602,
    -0.01981648043281047
  ],
  [
    0.703850921879071,
    0.7100423561242176,
    0.020826240168402843,
    -0.07544143005003269
  ],
  [
    0.0,
    0.0,
    0.0,
    1.0
  ]
]
```

T_I_L =
```json
[
  [
    -1.0,
    0.0,
    0.0,
    -0.006253
  ],
  [
    0.0,
    -1.0,
    -0.0,
    0.011775
  ],
  [
    -0.0,
    0.0,
    1.0,
    -0.028535
  ],
  [
    0.0,
    0.0,
    0.0,
    1.0
  ]
]
```

T_B_L =
```json
[
  [
    -0.703850921879071,
    -0.7100423561242176,
    0.020826240168402843,
    -0.07207613788438523
  ],
  [
    0.7099216283860768,
    -0.7041416662469333,
    -0.013992691115502887,
    -0.0024978355832723717
  ],
  [
    0.02460002682200807,
    0.004936229792296243,
    0.9996851866041602,
    -0.008613836505025514
  ],
  [
    0.0,
    0.0,
    0.0,
    1.0
  ]
]
```

See TEST_FRAME_PREREGISTRATION.json for paths/SHA/anchor; PRIOR_MAP_MANIFEST.json for map bounds, point counts and build/runtime identity. The static50m sphere uses only the anchor position; 0.20m voxel is applied once. Anchor overlap0.937857 is support, NOT absolute accuracy. The post-run map poses/reference were never used to recrop.
