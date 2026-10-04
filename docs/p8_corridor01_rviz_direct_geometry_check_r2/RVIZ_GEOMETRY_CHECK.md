# Corridor01 TX666 RViz Direct-Geometry Check R2

## Scope and result

This is a single-scan visualization only. No localization, NDT, EKF, map, filtering, initial-pose, or GT input was changed. The green cloud is transformed by the official YAML pose and official LiDAR-to-IMU extrinsic. The red cloud is the already-recorded TX666 NDT terminal from the frozen replay; its failed update was not applied to navigation state.

The screenshots establish that the normalization transform is internally consistent and that the official and NDT-terminal placements are spatially distinct. They do **not** uniquely identify which repeated corridor segment is the correct one: the visible local geometry is dominated by repeated longitudinal planes, and no unmistakable door frame, column, or other unique transverse landmark resolves the competing placements. Result: `VISUAL_RESULT_AMBIGUOUS`.

## Frames and transform contract

The raw PCD has no embedded `frame_id`. For this visualization only, it is published in `world` under the existing dataset-contract assumption `raw_map_world_relation_status = IDENTITY_USED_PER_OFFICIAL_FRAME_CONTRACT`; the screenshots do not independently prove that official world and raw PCD coordinates are identical.

The normalized map uses the RViz frame alias `normalized_map` (the dataset config calls its underlying map frame `camera_init`). `T_NORMALIZED_WORLD` maps raw/world coordinates into normalized coordinates:

```text
T_world_imu =
[ 0.135989979134 -0.990409550441 -0.024405900304  1.968147000000
  0.990705105525  0.136027108544  0.000140097550 -6.879292000000
  0.003181110098 -0.024198101899  0.999702121836 -0.896125000000
  0              0              0               1            ]

T_imu_lidar =
[ 0.999991859723 -0.000516138108  0.004001760674  0.080000000000
  0.000519624185  0.999999486419 -0.000870145088  0.029000000000
 -0.004001309504  0.000872217416  0.999991614344  0.030000000000
  0              0              0               1            ]

T_world_lidar = T_world_imu * T_imu_lidar =
[ 0.135571886943 -0.990500518648 -0.022999696288  1.949572144359
  0.990767163313  0.135515820220  0.003986297786 -6.796086602484
 -0.000831607318 -0.023327773761  0.999727524579 -0.866581192492
  0              0              0               1            ]

T_normalized_world =
[ -0.143643602700  0.989002287400 -0.035227581860  7.363630829145
   0.987183928500  0.140696853400 -0.075314566490 -0.644385552171
  -0.069529883560 -0.045594558120 -0.996537387400  0.346859433882
   0               0              0               1            ]

T_normalized_lidar = T_normalized_world * T_world_imu * T_imu_lidar =
[ 0.960426252047  0.277126300211 -0.027971766345  0.392769627278
  0.273294842421 -0.956982622521 -0.097438116103  0.389278922629
 -0.053771160741  0.085937600504 -0.994848442737  1.384751032815
  0               0              0               1            ]
```

The same scan-end-deskewed 29,063-point cloud was used in both frames. As requested, its green official-initial placement uses the official 67 s anchor transform above, not the CSV prediction or NDT pose. The scan end is 91.700 ms after the re-anchor timestamp, so this visualization intentionally applies the specified anchor to the scan-end cloud; that small epoch mismatch is a caveat, not an extra pose fit.

The red terminal is read from the frozen TX666 registration record and transformed back into raw/world with `inverse(T_normalized_world)`. Its pose is not used for the green cloud.

## Map-frame consistency

Both maps contain 338,210 points. Applying `T_normalized_world` to every raw-map point and querying the normalized map gives nearest-neighbor residuals:

| Statistic | Residual |
| --- | ---: |
| Mean | 0.000006459 m |
| Median | 0.000003610 m |
| P95 | 0.000020782 m |
| Max | 0.000037265 m |

This is pointwise agreement to float/serialization precision. The raw and normalized screenshots show the same relative cloud geometry after the corresponding transform. Therefore the current evidence does not implicate the raw-to-normalized transform.

## TX666 source and frozen NDT record

```text
Transaction: TX666
Raw scan points: 29,063
Deskewed points: 29,063
Dropped points: 0
Deskew reference: scan-end LiDAR frame, existing ScanEndProcessor output
GT used: false
NDT status: ITERATION_LIMIT_EXHAUSTED
Iterations: 80
Navigation update applied: false
Translation correction: 1.943452055 m
Rotation correction: 0.296461765 rad = 16.988 deg
```

The archived frozen registration record reports the prediction-start overlaps `<0.2/0.3/0.5/1.0 m` as `0.154286/0.221429/0.361429/0.505714`, and terminal overlaps as `0.445714/0.587143/0.742143/0.879286`. Those start-overlap values belong to the scan-end prediction in the registration record, not the exact official 67 s anchor used for the green visualization. The larger terminal overlap is only evidence of a better local geometric fit, not proof of the correct corridor basin.

## RViz topics and visual interpretation

Published latched topics:

```text
/corridor01/raw_map                         338210 points, frame world
/corridor01/normalized_map                  338210 points, frame normalized_map
/corridor01/tx666_official_raw               29063 points, frame world
/corridor01/tx666_official_normalized        29063 points, frame normalized_map
/corridor01/tx666_ndt_raw                    29063 points, frame world
/corridor01/tx666_ndt_normalized             29063 points, frame normalized_map
```

Static TF includes `world -> imu -> lidar`, `normalized_map -> world`, and separate scan-end prediction / NDT terminal frames. RViz uses explicit XYZ axes for world, official IMU, official LiDAR, and NDT terminal; duplicate TF glyphs are disabled to keep axes readable. Colors are map gray, official scan green, NDT terminal red.

| View | Screenshot |
| --- | --- |
| Raw top | `screenshots/rviz_raw_top.png` |
| Raw perspective | `screenshots/rviz_raw_perspective.png` |
| Raw along corridor | `screenshots/rviz_raw_corridor.png` |
| Normalized top | `screenshots/rviz_normalized_top.png` |
| Normalized perspective | `screenshots/rviz_normalized_perspective.png` |
| Normalized along corridor | `screenshots/rviz_normalized_corridor.png` |

Visual checks:

- **Raw official alignment:** ambiguous. Green scan is in the mapped corridor region, but local longitudinal wall/ceiling returns repeat and do not uniquely identify the segment.
- **Raw NDT alignment:** locally better geometric support is visible; truth of the selected basin remains ambiguous.
- **Normalized official alignment:** same ambiguity as raw view.
- **Normalized NDT alignment:** same local fit and same ambiguity as raw view.
- **Corridor-axis check:** the raw map corridor is predominantly along raw/world `+Y`; `T_normalized_world` maps that direction predominantly to normalized `+X`. The two view sets preserve the same scan/map relationship.
- **Distinctive structure check:** no uniquely identifiable door frame, column, or transverse landmark resolves the candidate corridor segment in these views.

Therefore the evidence neither closes `official world == raw PCD frame` nor supports a confident wrong-basin verdict. It does rule out a material normalization-transform inconsistency. Per the task's Case 5, the visual result is ambiguous; the bounded next diagnostic, if authorized, is TX666 local multi-start basin comparison—not a longer replay.

## Implementation and verification

- `scripts/p8/tx666_deskew_export.cpp` exports the existing ScanEndProcessor output only; no NDT is run by the exporter.
- `scripts/p8/tx666_rviz_geometry_publisher.py` validates the frozen inputs, publishes the two maps/two scan placements in both frames, and writes the matrix/topic manifest.
- `scripts/p8/capture_tx666_rviz_views.py` writes six fixed-camera RViz configs and screenshots.
- `tests/p8_tx666_rviz_pcd_parser_test.py` checks valid binary PCD payload, zero padding, nonzero trailing bytes, and truncation.
- The raw PCD contains 3,831 trailing zero bytes after its declared point payload; the parser ignores only these all-zero bytes and rejects nonzero/truncated tails.
- CTest: `p8_tx666_rviz_pcd_parser_test` passed (1/1). Python byte-compilation and `git diff --check` passed.
- Isolated ROS master: port 11312. Pre-existing user ROS master/RViz on port 11311 was not touched.
