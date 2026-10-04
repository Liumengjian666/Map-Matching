# Corridor01 TX666 public FAST-LIO preprocessing parity

## Result

`PUBLIC_FASTLIO_PREPROCESS_PARITY = FAIL` for full deskewed-cloud parity, but the three upstream data-contract checks close:

- packet decoding: exact XYZ equality, 29,063/29,063 returns;
- point-time semantics: maximum absolute discrepancy 5 ns at the public Velodyne point-cloud input and 12 ns after FAST-LIO body publication;
- LiDAR-to-IMU transform direction/application: translation is identical and the report-only nearest-SO(3) component of the public rotation equals the P7 runtime rotation to numerical precision. No inverse/transpose/sign error was found.

The remaining failure is specifically `DESKEW_STATE_CONTRACT_MISMATCH`, not a packet adapter or timestamp defect. P7's Corridor01 +67 s re-anchor uses zero velocity and the static-window gyro bias; the public FAST-LIO run has already estimated a nonzero velocity and different online IMU biases from its 0-to-67 s history. Thus strict end-to-end deskewed-cloud parity fails, while parity of the deskew algorithms under identical state inputs remains indeterminate; this run does not isolate that algorithmic comparison.

No localization core, deskew code, NDT, map, or GT was changed. The adapter's seconds-to-nanoseconds conversion was retained because its measured result is already correct.

## Public source and configuration used

- `engcang/SLAM-application`, commit `412e162fd7a125df722aa84883541aa7a397cab1`, file `FAST_LIO/config/subt_longcorridor.yaml`.
- FAST-LIO core `hku-mars/FAST_LIO`, commit `7cc4175de6f8ba2edf34bab02a42195b141027e9`. The public repository's copied `laserMapping.cpp` differs from that core only by timing-instrumentation additions; the mapping and deskew operations are otherwise the same.
- Pinned ROS1 Velodyne decoder from the existing local source, commit `29abd0e1361cb7f5eda451d2b51c35eeca45e0d5`, using the SubT VLP16 calibration file `VLP16db.yaml` (SHA256 `171e5fbf3c17256ca1fdc4098a3d190f668d414b1212a8f90bbf0fe6956a11bc`). No source in that workspace was modified.

FAST-LIO loaded the public YAML directly. The only runtime parameter override was `publish/scan_bodyframe_pub_en=true`, because the public file defaults that diagnostic output off. The public launch's algorithm values were retained: `feature_extract_enable=0`, `point_filter_num=1`, `max_iteration=3`, `filter_size_surf=filter_size_map=0.3`, `cube_side_length=1000`, and `runtime_pos_log_enable=false`.

The packet-to-point converter receives the bag's `/velodyne_packets`, publishes the public sensor contract on `/velodyne_points`, and uses the pinned official VLP16 decoder/calibration. The driver emits `PointCloud2.time` as relative `float32` seconds, while the public FAST-LIO YAML says `timestamp_unit=3` (nanoseconds). A transparent relay changes only that field by `time_ns = float32_seconds * 1e9`; all XYZ, intensity, ring, header stamps, and frame IDs remain unchanged. FAST-LIO then applies its configured nanosecond-to-millisecond scale. Public config values remain untouched:

```text
lidar_type = 2 (Velodyne)
scan_line = 16
timestamp_unit = 3 (ns)
blind = 0.5 m
time_sync_en = false
time_offset_lidar_to_imu = 0.0 s
extrinsic_T = [0.080, 0.029, 0.030] m
extrinsic_R = [0.999212900, -0.000519121, 0.004000000,
               0.000516111, 0.999218492, -0.000939132,
              -0.004000000, 0.000802565, 0.999993652]
```

The public body publisher normally omits per-return curvature/time. In the temporary FAST-LIO clone only, the diagnostic publisher copied `pi->curvature` into the published body point. This does not feed back into preprocessing, state estimation, or map matching. No public or stable workspace was edited.

## Same-transaction proof

The bag's transaction 666 (`/velodyne_packets`) has record time `1690254179920640694 ns`, bag-relative `67.09889936447144 s`, and scan header/start `1517157286155932903 ns`. The decoded raw-point maximum is `1517157286256772352 ns`, which is the frozen TX666 scan end.

```text
OUR_TX666_SCAN_START = 1517157286155932903 ns
OUR_TX666_SCAN_END   = 1517157286256772352 ns
PUBLIC_RAW_HEADER    = 1517157286155932903 ns  (exact start match)
PUBLIC_BODY_HEADER   = 1517157286256772280 ns  (72 ns before scan_end)
PUBLIC_BODY_FRAME_ID = body
FRAME_MATCH          = PASS
```

The body header's 72 ns shortfall is float-time conversion/rounding; the body's maximum preserved point offset is `100839447 ns`, versus source `100839449 ns`. This is the same transaction, not an adjacent scan.

## Point and timestamp comparison

| Quantity | P6 source | Public decoded / FAST-LIO | Result |
|---|---:|---:|---|
| Decoded points | 29,063 | 29,063 | equal |
| FAST-LIO body points | — | 29,063 | no returns removed by `blind=0.5 m` |
| Finite XYZ | 29,063 | 29,063 | all finite |
| Minimum raw range | 0.980 m | 0.980 m | no point lies inside the 0.5 m blind radius |
| Maximum raw range | 82.752 m | 82.752 m | same raw support |
| Relative point-time range | 0–100,839,449 ns | raw: 0–100,839,451 ns; body: 0–100,839,447 ns | same scan timing |

Source-vs-public raw XYZ, index by index:

```text
mean = 0 m
median = 0 m
P95 = 0 m
max = 0 m
```

Absolute point-time error:

```text
Public raw input minus source offsets: mean 1.110 ns, median 1 ns, P95 3 ns, max 5 ns.
FAST-LIO body curvature-derived times minus source offsets: mean 1.817 ns, median 1 ns, P95 6 ns, max 12 ns.
```

The errors are consistent with the public ROS `float32` time field and its configured ns→ms conversion; they are not a scan-scale or firing-order error.

## Common-frame deskewed-cloud comparison

The public cloud is `/cloud_registered_body`, the full dense scan in FAST-LIO's scan-end IMU/body frame. The project cloud is the P7 `ScanEndProcessor` full SE(3) result in scan-end LiDAR coordinates, transformed to the same nominal scan-end IMU/body frame using P7's frozen `T_imu_lidar`.

### Bidirectional nearest-neighbor distances

| Direction | Mean | Median | P90 | P95 | Max |
|---|---:|---:|---:|---:|---:|
| Public FAST-LIO → P7 | 0.038480 m | 0.025076 m | 0.078168 m | 0.138031 m | 0.649549 m |
| P7 → Public FAST-LIO | 0.038786 m | 0.024923 m | 0.086367 m | 0.140837 m | 0.654402 m |

| NN overlap threshold | FAST-LIO → P7 | P7 → FAST-LIO |
|---|---:|---:|
| `<0.02 m` | 0.3678 | 0.3653 |
| `<0.05 m` | 0.8358 | 0.8407 |
| `<0.10 m` | 0.9132 | 0.9089 |
| `<0.20 m` | 0.9839 | 0.9826 |

Because raw XYZ and point-time indices match exactly, pointwise same-return comparison is more diagnostic than nearest-neighbor matching (which can match a different return on repeated/nearby surfaces):

```text
Same-return distance: mean 0.150149 m, median 0.147921 m,
P90 0.266361 m, P95 0.283381 m, max 0.654402 m.
```

The discrepancy decays toward scan end:

| Point offset | Count | Same-return median | Same-return P95 |
|---|---:|---:|---:|
| 0–20 ms | 5,748 | 0.266720 m | 0.304306 m |
| 20–40 ms | 5,786 | 0.207475 m | 0.233913 m |
| 40–60 ms | 5,729 | 0.148617 m | 0.174867 m |
| 60–80 ms | 5,784 | 0.095702 m | 0.122618 m |
| 80–100.839 ms | 6,016 | 0.032988 m | 0.061987 m |

For the last 1 ms (`>=100 ms`, 233 points), same-return median/P95/max are `0.001415 / 0.002724 / 0.003245 m`. The strong time dependence and near-zero terminal residual are characteristic of different within-scan motion histories, not a fixed XYZ axis swap or a constant extrinsic translation error.

The basis sample uses the exact source point index in both outputs. FAST-LIO's diagnostic body publisher loops over the undistorted cloud and writes each converted point to the same index; the analyzer additionally verifies that the public raw and body point offsets at each sampled index agree with the source point time (within 250 ns).

## Root-cause separation

### Packet decoding: PASS

The P6 timed export and public Velodyne ROS converter use the same pinned VLP16 decoder/calibration provenance. All 29,063 XYZ triples are bitwise equal when compared by source order. No packet decoding discrepancy was found.

### Point time: PASS

The seconds→nanoseconds bridge preserves every point's firing offset to at most 5 ns at FAST-LIO input and at most 12 ns in the body publisher. No frame-scale offset, sign reversal, or timestamp ordering mismatch was found.

### Extrinsic application: PASS for direction/translation and effective rotation; raw-matrix caveat retained

Both chains use `p_imu = T_imu_lidar p_lidar`; both translations are exactly `[0.080, 0.029, 0.030] m`. FAST-LIO loads the public matrix into its MTK `SO3` state through Eigen's matrix-to-quaternion path; the runtime source does not call SVD. The literal public matrix is not a proper rotation (`||RᵀR-I||F=0.002213966`, `det(R)=0.9984426773`). A report-only nearest-proper-rotation check (not fed to either algorithm) gives P7's runtime rotation to Frobenius residual `4.98e-13` and relative angle `4.17e-14 deg`. At the last 1 ms, full-cloud pointwise median/P95/max separation is only `1.4/2.7/3.2 mm`. There is no evidence of inverse, transpose, or translation-sign error.

### Deskew: end-to-end FAIL; isolated algorithm parity INDETERMINATE

The P7 +67 s official re-anchor contract sets `v_67=[0,0,0]` and uses static-window gyro bias `[0.00830944, -0.02382068, 0.03699050] rad/s` (accel-bias prior is zero). In FAST-LIO's own non-GT state log at the TX666 scan start (`67.0678 s` relative to its first LiDAR scan), its estimated velocity is `[2.99678, 0.411264, -0.232985] m/s` (norm `3.03383 m/s`), gyro bias is `[-0.00352412, -0.00125998, -0.02417500] rad/s`, and accel bias is `[-0.257519, 0.0380234, 0.0168117] m/s²`.

The velocity-contract difference alone corresponds to about `0.30593 m` of translation over this `100.839 ms` scan. The gyro-bias difference norm is `0.066259 rad/s`, or about `0.006682 rad` integrated over the scan. These are state estimates from FAST-LIO, used only to explain why its scan-end deskew differs; they are not GT and were not used to alter P7.

Therefore the full-cloud parity failure cannot be attributed to packet decoding, time units, frame direction, or a small extrinsic adapter. Changing those would corrupt already-matched data. A decisive algorithm-level deskew comparison requires a common motion-state contract. Importing FAST-LIO's estimated 67 s velocity/bias into P7 would change the agreed official re-anchor semantics and would use estimator state outside this sensor-interface test, so no such change was made. The present result is not evidence that either deskew implementation is intrinsically wrong.

## Pointwise basis sample

Thirty source points (first 10, middle 10, final 10) with raw LiDAR XYZ, firing time, P7 body-frame XYZ, and public FAST-LIO body-frame XYZ are archived in `raw_point_basis_30.csv` in the external result directory. The CSV records every component at full precision. Representative rows:

```text
begin index 1, 2.304 us:
raw=(-9.898822,-0.269583,0.172849)
P7=(-9.824585,-0.326480,0.168445)
FAST=(-10.121890,-0.234343,0.234640), delta=0.318216 m

middle index 14530, 50.407086 ms:
raw=(15.144326,0.203544,-0.264370)
P7=(15.218726,0.220664,-0.242712)
FAST=(15.072261,0.200078,-0.232277), delta=0.148272 m

end index 29062, 100.839449 ms:
raw=(-9.149556,0.172487,2.452052)
P7=(-9.059758,0.194599,2.518792)
FAST=(-9.059754,0.194593,2.518785), delta=0.000010 m
```

## Artifacts and implementation scope

Repository additions:

- `scripts/p8/public_fastlio_tx666_parity.py`: seconds→ns relay, TX666 input/body capture, scan-time checks.
- `scripts/p8/analyze_public_fastlio_tx666_parity.py`: raw/time parity, common-frame NN and same-return statistics, basis CSV, PCD overlay, PNG plot.

External HIKVISION result directory:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p8_public_fastlio_preprocess_parity_r1_20261005_analysis_v2/`

It contains both captured NPZ clouds, our P7 scan-end cloud, two separate common-frame PCDs, a colored combined overlay PCD, `raw_point_basis_30.csv`, `parity_metrics.json`, and `tx666_deskew_overlay.png`.

No GT, map, NDT, inverse transform, normalized map, or runtime SVD was used. The public FAST-LIO run was limited to bag start through the TX666 scan. No long localization replay was performed.
