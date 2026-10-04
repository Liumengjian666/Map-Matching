# Corridor01 TX666 deskew-to-NDT impact A/B (R1)

## Question and decision

This experiment asks whether the measured difference between the P7 and public FAST-LIO TX666 deskewed clouds explains the large NDT correction. Only the source cloud changed. Both registrations used the same raw map, source preprocessing, official startup seed, and frozen NDT settings.

**Decision: `DESKEW_DIFFERENCE_NOT_PRIMARY_CAUSE`.** FAST-LIO deskew reduced the reported translation correction from 1.803 m to 1.500 m, but it still required a 16.56° rotation correction. Thus deskew-state differences do not explain away the large correction. The resulting terminals differ by 0.724 m and 1.468°—measurable terminal sensitivity, but less than one 0.8 m NDT resolution cell in translation. Two single-start runs do not establish separate stable basins, so the difference is not enough to classify this as a basin switch.

## Frozen inputs and provenance

- Raw map: `.../Corridor01/map/corridor01.pcd`, SHA256 `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`.
- P7 deskew source: `tx666_our_p7_scan_end_lidar.pcd`, SHA256 `6aec8bf2fc78d93c54ceb593f8274536313defa44df416daf13861b2322b3c67`.
- Public FAST-LIO deskew source, converted from scan-end IMU/body to scan-end LiDAR coordinates with the frozen LiDAR–IMU extrinsic: `tx666_fastlio_deskew_scan_end_lidar.pcd`, SHA256 `2a88c074f9dd6137d92a3b6d70cff92ef39f8a3cd57b919343d30677251d45b8`.
- Both source scans contain 29,063 points and represent TX666. The source cloud presented to NDT is filtered/voxelized/capped identically: 0.5–80 m range, 0.25 m source voxel, maximum 1,400 points. Raw target map has 225,826 points after the frozen map preprocessing.
- Same initial guess for both runs: `T_world_lidar = T_world_imu * T_imu_lidar`:

```text
[ 0.135469425292  -0.989725170109  -0.022931760286   1.949572159
  0.989994861125   0.135406510400   0.003975071803  -6.796086617
 -0.000832800619  -0.023378414558   0.999731103008  -0.866581202
  0                0                0                1 ]
```

- Frozen NDT: resolution 0.8 m, step 0.08, transformation epsilon `1e-5`, maximum 80 iterations; actual target grid 0.8 m. Runtime SVD: **NO**. Normalized map: **NO**. GT: **NO**.
- The raw composed seed rotation is not exactly SO(3) (`||RᵀR-I||F=0.00221488`, `det(R)=0.998441895`). It was passed unchanged to both runtime registrations. Nearest-SO(3) projection was used only to report diagnostic rotation angles after the runs.
- Runner: `public_raw_tx666_ndt.cpp`; executable SHA256 `01f4f60219fc05b716d44d2cfa92da21a839a89db9dd8bce2d97f62c2cfa68dd`.

## Before NDT

| Source | <0.20 m | <0.30 m | <0.50 m | <1.00 m | NN mean | NN median | NN P95 | NN-MSE |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| P7 deskew | 0.155714 | 0.221429 | 0.355000 | 0.510714 | 1.696931 m | 0.973127 m | 4.867848 m | 8.116096 m² |
| FAST-LIO deskew | 0.166429 | 0.243571 | 0.366429 | 0.519286 | 1.691423 m | 0.905426 m | 4.830004 m | 8.396590 m² |

Both initial clouds have similarly weak map support. FAST-LIO is modestly better in threshold overlaps and median/P95 distance; initial NN-MSE is slightly worse.

## Frozen NDT results

| Result | P7 deskew | Public FAST-LIO deskew |
|---|---:|---:|
| PCL converged | YES | YES |
| Iterations | 80 | 49 |
| Wrapper status | `ITERATION_LIMIT_EXHAUSTED` | `SUCCESS` |
| Translation correction (runner’s Euclidean pose-translation delta) | 1.802832 m | 1.499938 m |
| Rotation correction, report-only SO(3) diagnostic | 0.278775 rad / 15.9727° | 0.289019 rad / 16.5595° |
| Final PCL fitness / NN-MSE | 0.897366 m² | 1.015050 m² |
| Final overlap <0.20 / <0.30 / <0.50 / <1.00 m | 0.442143 / 0.565000 / 0.727857 / 0.875000 | 0.415714 / 0.553571 / 0.691429 / 0.856429 |
| Final NN mean / median / P95 | 0.516036 / 0.232914 / 2.101987 m | 0.561142 / 0.259617 / 2.200087 m |

P7 reached the 80-iteration cap despite PCL reporting convergence; FAST-LIO returned success in 49 iterations. FAST-LIO reduced translation correction by 0.302894 m, but neither run is close to a small-correction result. P7 has the better final overlap and fitness in this fixed-seed comparison.

### Terminal poses

P7 terminal:

```text
[-0.127822518349 -0.990760028362  0.045342717320  0.436046451330
  0.990259230137 -0.130035802722 -0.049773186445 -5.817438602450
  0.055209461600  0.038538910449  0.997730731964 -0.907697737217
  0               0               0               1]
```

FAST-LIO terminal:

```text
[-0.130722165108 -0.988899886608  0.070630431175  0.471900492907
  0.989825844765 -0.134217664599 -0.047226946801 -6.540042400360
  0.056182570756  0.063738219440  0.996383965015 -0.893791139126
  0               0               0               1]
```

Using the requested `Delta_terminal = inverse(T_terminal_P7) * T_terminal_FASTLIO`, computed from the unmodified runtime matrices:

```text
translation component = [-0.719380242, 0.058977560, 0.051467056] m
translation norm      = 0.723626383 m
rotation diagnostic   = 0.025625509 rad = 1.468234 deg
```

Only the diagnostic angle used nearest-SO(3) projection; neither registration was altered. The terminal centers differ by 0.723626 m in Euclidean distance, primarily along map Y (`FAST - P7 = [0.035854, -0.722604, 0.013907] m`).

## Interpretation and limits

The common seed and identical map/preprocessing isolate the source cloud as the A/B variable. The FAST-LIO deskew cloud produces a 0.724 m terminal offset and changes iteration behavior, so source geometry measurably affects the optimization result. However, the offset is under one NDT resolution cell, both runs still make large corrections (1.50–1.80 m and about 16°), and FAST-LIO does not resolve the original large-correction problem. This is not evidence that either deskew implementation is intrinsically incorrect: their motion states differ (P7 starts with zero velocity while FAST-LIO estimates about 3.03 m/s), as established by the preceding preprocessing-parity report.

No multi-start or stability test was run, so this A/B neither proves nor rules out separate stable basins. The observed terminal difference alone is not sufficient evidence for a basin switch. A same-source local multi-start test can address basin structure next; it was not run in this task.

## Artifacts

Four map-aligned clouds (initial and terminal for each source) are archived at:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p8_tx666_deskew_ndt_impact_ab_r1_20261005/`

The two source clouds and preceding parity report are archived at:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p8_public_fastlio_preprocess_parity_r1_20261005_analysis_v2/`
