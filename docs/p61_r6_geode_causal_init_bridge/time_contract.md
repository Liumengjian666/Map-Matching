# Causal time and native integration contract

`T_A_B` maps B coordinates to A. M is the unchanged R5 `camera_init` gauge; I external Xsens, L Livox Avia. ROS header is scan start, `offset_time` is per-point ns, and the author uses header+offset, not device timebase as epoch. Existing time_sync=false/time_offset=0 remain unchanged.

| Event | Exact original/logical ns |
|---|---:|
| First original LiDAR start | 1706671289400000810 |
| R5 last admitted scan2441 raw tail | 1706671533499543833 |
| R5 immutable mapping cutoff | 1706671533569869338 |
| Final bridge scan2984 start | 1706671587699897289 |
| Final bridge scan2984 raw tail | 1706671587799724734 |
| Last external IMU raw header | 1706671587824846982 |
| Logical initialization/test start | 1706671587829840123 |
| First full test scan2986 start | 1706671587900364160 |
| First full test scan2986 raw tail | 1706671588000204345 |

Scan2442 crossed the R5 map cutoff and remains excluded from the frozen map, but was legally processed by this task's separate state bridge. Scan2985 crosses the test-start cutoff and was excluded whole. No partial scan, offset shift or synthetic LiDAR message was used.

The bridge-only bag contains scans1–2984 and29844 raw Xsens messages, all <=test start. The original2441-scan prefix is byte-identified by inherited serialized SHA checks. At ACK2441 the direct native scan-end pose gate was checked before publishing scan2442; all2441 prefix poses ultimately matched R5 exactly. No SE(3)/GT alignment was applied.

Additional bridge LiDAR count543 comprises542 fully middle-window scans plus boundary-crossing scan2442. Strict middle-window raw IMU count is5426. Newly published IMUs after the R5 scan-end gate number5432, because some remaining first-window measurements precede the strict middle boundary. These are different counting definitions, not skipped or fabricated data.

## Terminal IMU-only propagation

Native final scan-end state is recorded as double seconds `1706671587.7997248`; exact raw point tail is separately retained above. To logical test start, the raw-tail interval is30.115389ms. Three real pending Xsens measurements are processed, followed by the author's existing held pair-average input for the final4.993141ms after the last raw measurement. No future IMU was used for interpolation.

The forward integration block was extracted without changing its arithmetic into `PropagateImu`, shared by original Deskew and the terminal no-LiDAR wrapper. A regression test compares the integration block literally against R5; the only variable substitution is `last_imu_=v_imu.back()` for the identical last `meas.imu.back()` sample. Native `kf.predict`, noise, bias and gravity handling are unchanged. No CV model or CSV filter restoration was introduced. No terminal LiDAR update/map insertion occurs.

Author `imu_cbk` intrinsically converts header `toSec()` through double and back with `fromSec()`, even when offset=0. For the final sample this produces internal ns `1706671587824846983`, +1ns from the original header. The service therefore checks an observation-only **raw integer arrival watermark under the original buffer mutex**, not equality between raw and internally rounded times. Both times are saved. Existing native conversion was not changed or relaxed. `test_native_time.cpp` reproduces the original failed equality and verifies the raw-watermark fix before the sole real run.

The exact logical target is retained as integer ns; the existing native filter works at double seconds `1706671587.8298402`. This is not a claim that double epoch seconds retain nanosecond arithmetic precision. The final native and raw measurement times are both strictly before the logical target.

Initialization SHA freeze preceded opening scan2986. That scan begins70.524037ms after the initial pose, with raw points through170.364222ms after it. The subsequent one-align probe **does not deskew or propagate the initial pose through this interval**. Its NN/score/correction are raw geometric diagnostics, not synchronized pose-error measurements. The NDT output never replaces the earlier initializer.

## Actual covariance

`initial_covariance.csv` is the native filter's actual23×23 P at the propagated state. State order: position3, rotation3, LiDAR-to-IMU rotation3, LiDAR-to-IMU translation3, velocity3, gyro bias3, acceleration bias3, gravity S2 tangent2. It is not a LiDAR6×6 covariance or an inverse NDT Hessian. Pose rotation uses the native MTK manifold chart; no covariance coordinate conversion is implied.

Velocity is the IMU origin velocity in M; biases are in I, gravity in M. RPY is explicitly SciPy `Rotation.from_matrix(T_M_L[:3,:3]).as_euler('xyz',degrees=True)`. Recorded covariance symmetry error1.70e-17; minimum eigenvalue1.39147e-7. These are numerical checks, not independent uncertainty-calibration evidence.
