# Corridor01 public-config raw-map TX666 check

## Scope

One TX666 scan only; one PCL NDT alignment. No GT, normalized map, SVD, inverse-pose candidate, full replay, or production localization changes. The raw point packet adapter data were reused, and the scan was gyro-only deskewed to the recorded +67 s IMU anchor while preserving all 29,063 returns. This is a targeted public sensor/frame-configuration check, not a full frozen P7 replay: it omits within-scan translational deskew and therefore its alignment result is not an apples-to-apples replacement for the prior full SE(3) replay.

## Public files actually read

- `whu-xjh/Voxel-Pillar/config/SubT_MRS.yaml`, `# Corridor01` block. That block is commented; the active block is for Laurel Cave. Its Corridor01 extrinsic is corroborating text, not proof of an active run configuration.
- `engcang/SLAM-application/FAST_LIO/config/subt_longcorridor.yaml`
- `engcang/SLAM-application/ig_lio/config/subt_longcorridor.yaml`
- `engcang/SLAM-application/sr_lio/config/subt_longcorridor.yaml`
- `engcang/SLAM-application/ada_lio/config/subt_longcorridor.yaml`
- `engcang/SLAM-application/LIO_SAM_6AXIS/LIO_SAM_6AXIS/config/subt_longcorridor.yaml`
- `engcang/SLAM-application/direct_lidar_inertial_odometry/cfg/subt_longcorridor.yaml`

The six `subt_longcorridor` files were read from the repository's current `main` branch on 2026-10-04. FAST-LIO, iG-LIO, SR-LIO, Ada-LIO, LIO-SAM-6AXIS, and DLIO agree on the listed LiDAR-to-IMU translation and public rotation array. Voxel-Pillar's labeled Corridor01 block shows the same values but is commented out.

## Dataset parameter table

| Parameter | Official/public evidence | Existing project state before this diagnostic | Decision for this diagnostic |
|---|---|---|---|
| Start time | SuperLoc `# s 67`; task contract fixes bag-relative +67 s | Existing P8 config had 67 s and a resolved first-IMU anchor | Keep 67 s; anchor `1517157286165072000 ns` |
| LiDAR | Velodyne; dataset metadata identifies VLP-16 | VLP-16 packet decoder | Same |
| Scan lines | 16 in FAST-LIO, SR-LIO, Ada-LIO, LIO-SAM, and Voxel-Pillar's Corridor01 block | Implicit in VLP-16 packet decode; not explicit in older replay config | Explicitly record 16 |
| Scan rate | 10 Hz in SR-LIO/Ada-LIO; measured median transaction interval here is 100.859881 ms (9.915 Hz) | No explicit dataset field in older runtime config | Record public nominal 10 Hz; measured data are about 9.915 Hz |
| Point-time unit | FAST-LIO/SR-LIO/Ada-LIO use enum `3` = ns | Raw packet adapter produces absolute sensor ns then `uint32 ns` offsets from scan start; old derived ROS config called the field seconds | Keep adapter's ns contract; no seconds reinterpretation |
| LiDAR topic | Public algorithms consume `/velodyne_points`; actual bag source is `/velodyne_packets` | Existing packet→point adapter | Preserve raw bag topic and adapter; do not rename packet messages |
| IMU topic | `/imu/data` across public files and bag | `/imu/data` | Same |
| Time sync/offset | FAST-LIO: sync off, offset 0; SR-LIO: time-diff disabled; others have no Long-Corridor offset | Adapter applies no extra offset | Use `false`, `0.0 s`. Voxel-Pillar's global `0.1 s` is not Corridor01-specific and is not adopted |
| Minimum range | 0.5 m in FAST-LIO/iG-LIO/SR-LIO/Ada-LIO/LIO-SAM; Voxel-Pillar active cave setting is 0.4 m | Frozen source filter minimum 0.5 m | Same, 0.5 m |
| Maximum range | Public Long-Corridor values conflict: FAST/iG 150 m, SR/Ada 100 m, LIO-SAM 60 m | Frozen NDT source filter maximum 80 m | Preserve algorithm-specific frozen 80 m; no unique public value to copy |
| Gravity | iG-LIO/SR-LIO: 9.80665; LIO-SAM: 9.81 | Frozen frontend parameter file: 9.809 | Canonical metadata records 9.80665 (shared by two configs); not used by this gyro-only one-scan deskew |
| IMU rate | LIO-SAM config: 200 Hz | Actual CSV median period 4.992 ms = 200.321 Hz | Consistent with nominal 200 Hz |
| `T_imu_lidar` translation | All listed Long-Corridor configs: `[0.080, 0.029, 0.030] m` | Prior runtime used a projected official matrix | Use public values exactly |
| `T_imu_lidar` rotation | Public configs: `[[.999212900,-.000519121,.004],[.000516111,.999218492,-.000939132],[-.004,.000802565,.999993652]]` | Prior code applied SVD projection | Use the exact public values, no SVD |
| `T_world_imu` | SuperLoc official Corridor01 YAML | Previous loader projected the official matrix | Use official numbers directly, no projection |
| Map | This task requires the official raw Corridor01 PCD | Previous P8 replay used normalized map and `T_normalized_world` | Use `/map/corridor01.pcd` directly, no normalization transform |
| NDT | Project baseline only; not a public dataset parameter | Frozen: 0.8 m / 0.08 / 1e-5 / 80 iters | Preserve; map/source voxel filters also remain frozen |

### Matrix-source discrepancy

The public Long-Corridor configs give the last diagonal of `R_imu_lidar` as `0.999993652`; the local official `corridor01_extrinsics.yaml` gives `0.999992652`. This one-run test follows the user's explicit public matrix. No value was silently changed. Also, the supplied public matrix is not a proper rotation: `||RᵀR-I||_F = 0.0022139659`, `det(R)=0.9984426773`. Consequently `T_world_lidar` is not exactly SE(3), and an SO(3) geodesic correction cannot be reported without modifying the matrix (forbidden here). The evaluator therefore reports translation correction and explicitly returns rotation correction as undefined.

## Exact run inputs and result

Raw map PCD: 338,210 points, SHA256 `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`. After the frozen two-stage 0.15 m map/target voxel filtering, NDT target had 225,826 points. TX666 scan start/end were `1517157286155932903 / 1517157286256772352 ns`; anchor was `1517157286165072000 ns`, 9.139097 ms after scan start and 91.700352 ms before scan end. All 29,063 points were retained and gyro-only deskewed to the anchor LiDAR frame.

Static-window gyro bias used by that diagnostic deskew:

```text
[0.00830944018, -0.02382068498, 0.03699050114] rad/s
axis std = [0.03507283109, 0.04478743810, 0.04905005025] rad/s
```

`T_world_lidar = T_world_imu * T_imu_lidar`, with the exact supplied matrices:

```text
[ 0.135469425292  -0.989725170109  -0.022931760286   1.949572159000
  0.989994861125   0.135406510400   0.003975071803  -6.796086617000
 -0.000832800619  -0.023378414558   0.999731103008  -0.866581202000
  0                0                0                1              ]
```

One PCL alignment, no parameter search:

| Measurement | Before NDT | After NDT |
|---|---:|---:|
| NN overlap `<0.20 m` | 0.155714 | 0.427143 |
| NN overlap `<0.30 m` | 0.230000 | 0.557143 |
| NN overlap `<0.50 m` | 0.367857 | 0.710714 |
| NN overlap `<1.00 m` | 0.511429 | 0.861429 |
| NN mean | 1.713621 m | 0.547744 m |
| NN median | 0.941144 m | 0.258852 m |
| NN P95 | 4.915978 m | 2.231846 m |
| NN mean squared distance | 8.242830 m² | 0.988749 m² |

PCL converged in 57/80 iterations (`SUCCESS`). Translation correction was `1.525307 m`. Final PCL fitness was `0.988748666`. Relative-rotation matrix orthogonality error was `0.0022183316` with determinant `1.001560536`; rotation correction is therefore reported as `UNDEFINED_NONRIGID_RELATIVE_MATRIX`, not as an angle. The final rotation block emitted by PCL itself is near proper SO(3), but comparing it to the non-rigid initial block would not yield a valid rigid-body correction metric.

The result is better behaved than the earlier normalized-map/SVD result (80-iteration exhaustion and 1.943452 m translation change), but still requires a large `1.525 m` translation adjustment. Because this diagnostic also uses gyro-only (not the full frozen translational scan-end deskew), and because the literal public extrinsic is non-rigid, it does **not** prove that the residual is an NDT wrong-basin failure. It supports the requested status `CANONICAL_PUBLIC_CONFIG_STILL_HAS_LARGE_CORRECTION`, with those two causal limits stated.

## Runtime/verification

- `p8_public_raw_tx666_ndt` compiled against PCL 1.10.
- CTest: `p8_tx666_rviz_pcd_parser_test`, 1/1 passed.
- Python preparation completed with 29,063/29,063 finite points.
- The first process invocation failed before NDT because the environment loaded `/opt/MVS/lib/64/libusb-1.0.so.0`, which lacks `libusb_set_option`; the single successful alignment then ran with `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu`. No second NDT alignment was run.
- `SVD_USED=NO`, `NORMALIZED_MAP_USED=NO`, `GT_USED=NO`.

Artifacts are in the external dataset result folder `results/p8_public_config_raw_tx666_r1/`: preparation manifest, anchor-deskewed scan, official-initial aligned scan, and NDT-refined aligned scan.
