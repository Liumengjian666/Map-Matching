# P8 Corridor01 67 s full motion-state diagnostic

## Decision

`FINAL_RESULT = MOTION_STATE_INITIALIZATION_NOT_PRIMARY_CAUSE`

At the fixed official 67 s pose, injecting the FAST-LIO-derived motion state changed the scan-end LiDAR seed by 0.273374 m and 0.348126 deg. It reduced the NDT translation correction by only 0.054687 m (3.04%) and rotation correction by 0.008702 deg. Both runs still reached the 80-iteration cap and terminated at nearly the same pose (0.066871 m / 0.656300 deg apart). This single-frame test does not support zero velocity/bias initialization as the primary explanation for the large TX666 correction. It does not establish that either terminal is globally correct.

## Frozen inputs and provenance

- Anchor: `1517157286165072000 ns`; official `T_world_imu` from Corridor01 YAML. P7 represents this with its SO(3) quaternion state. Its decoded matrix is shown below; no SVD was used.
- Scan: TX666, `[1517157286155932903, 1517157286256772352] ns`, 29,063 original timed points. It starts 9.139097 ms before the anchor and ends 91.700352 ms after it. The existing causal scan processor reconstructs the pre-anchor portion backward from the anchored state and propagates forward through the scan; the last observed IMU sample before scan end is held to the exact end time.
- Map: official raw Corridor01 map; SHA256 `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`.
- P7 IMU CSV SHA256 `7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa`.
- Timed scan manifest SHA256 `41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d`; timed scan index SHA256 `e018558448ffa2a34c7bbe4f44d2219c22074193c91057765cea5d175544b1af`.
- Packed timed-point source SHA256 `6195878e0d0a68891b392c490078e7ed3be185cd6425d1738ccc13847df31916`.
- Frozen P7 parameters: `corridor01_params_official_calibration.txt`, SHA256 `7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d`.
- Fixed public `T_imu_lidar` (same in A/B; no SVD or inverse):

  ```text
  R = [ 0.999212900  -0.000519121   0.004000000
        0.000516111   0.999218492  -0.000939132
       -0.004000000   0.000802565   0.999993652 ]
  t = [0.080, 0.029, 0.030] m
  ```

- P7 registration settings were unchanged: map/target voxel 0.15 m, source voxel 0.25 m, source cap 1,400, range 0.5–80 m; NDT resolution 0.8 m, step 0.08, epsilon `1e-5`, maximum 80 iterations. Both cases loaded the same raw map and used the same preprocessing implementation.
- FAST-LIO reference: `hku-mars/FAST_LIO` commit `7cc4175de6f8ba2edf34bab02a42195b141027e9`; dataset-config repository `engcang/SLAM-application` commit `412e162fd7a125df722aa84883541aa7a397cab1`; `FAST_LIO/config/subt_longcorridor.yaml` SHA256 `8b4d5b58819e0a2203785d3b9975614417d17299b33d29710ed9e393465682dd`. State-only instrumentation sampled the filter immediately after IMU prediction and before the TX666 LiDAR update; it did not feed data back into FAST-LIO. The 20-row state capture is archived as `fastlio_tx666_anchor_states.csv`, SHA256 `b12f4ded6e5874f622f308bc278c3af8cc184c51332aa61895171b0558078ef5`.

FAST-LIO state was bracketed by `1517157286165071964 ns` and `1517157286170063972 ns`, i.e. 36 ns before and 4.991972 ms after the requested anchor. Rotation used quaternion SLERP; vector/scalar states used linear interpolation. The FAST-LIO absolute position and covariance were not imported. The FAST-LIO scratch checkout also contained the prior diagnostic-only copy-through of per-point `curvature` in `publish_frame_body`; this function is only used for the published body cloud. The state dump is after `kf_state.predict` and before the corresponding scan's LiDAR update. Neither instrumentation change feeds back into the filter state.

## State conventions and conversion

FAST-LIO `R_fast_world_imu` maps IMU/body vectors into its own world frame. Its velocity and gravity are in that world frame; gyro and accelerometer biases are in IMU/body axes and subtracted from measurements. P7 position, velocity, and gravity use the official raw-map/world frame; P7 biases use the same IMU/body axes and subtractive convention.

Only the FAST-LIO world-axis relation was used to transform velocity/gravity while retaining the official pose as the P7 anchor:

```text
R_map_fast_world = R_world_imu_official * transpose(R_fast_world_imu)
v_map = R_map_fast_world * v_fast_world
g_map = R_map_fast_world * g_fast_world, then normalized to P7 gravity magnitude 9.809 m/s^2
bg_p7 = bg_fast
ba_p7 = ba_fast / accel_input_scale
accel_input_scale = 9.81 / mean(||raw_acceleration||) = 0.998969232593
```

The accelerometer-bias scaling is necessary because FAST-LIO scales each raw accelerometer input before estimating `ba`, while P7 consumes raw m/s². Both cases retained the same fresh P7 initialization covariance; FAST-LIO covariance was unavailable and not transferred. The current case uses the frozen static-window gyro bias and P7's configured zero accelerometer bias. The full-state case uses FAST-LIO gyro/accelerometer biases, velocity, and transformed gravity. Thus this is a comparison of the requested full state pack, not a velocity-only ablation.

The official YAML rotation is rounded and not exactly orthonormal. Since P7's state is SO(3)-valued, the runner decodes it through a normalized Eigen quaternion (not SVD); both A/B anchors are identical. The resulting P7 anchor rotation is:

```text
R_world_imu =
[ 0.135990212695  -0.990409520525  -0.0244058129144
  0.990705073191   0.136027344131   0.00014000527676
  0.00318119535307 -0.0241980020171  0.999702123982 ]
p_world_imu = [1.968147, -6.879292, -0.896125] m
```

## State values at anchor

Current P7 state:

```text
R = the common R_world_imu above
p = [1.968147, -6.879292, -0.896125] m
v_map = [0, 0, 0] m/s
bg_imu = [0.008309440176, -0.023820684976, 0.036990501135] rad/s
ba_imu = [0, 0, 0] m/s^2
g_map = [-8.29389651046, 1.45065657871, -5.03223182738] m/s^2
```

The static calibration gate passed on the frozen 200-sample window `[1517157224023904000, 1517157225018848000] ns`, using the unchanged limits `acceleration std <= 0.5 m/s^2` and `gyro std <= 0.05 rad/s`.

FAST-LIO reference state at the interpolated anchor (its `p` is shown only to make clear it was deliberately not used as the map anchor):

```text
R_fast_world_imu =
[ 0.993509683620  -0.113416278116   0.008675045303
  0.113740010059   0.991429304308  -0.064273981287
 -0.001310978392  0.064843522553    0.997894583069 ]
p_fast_world_imu = [166.114621839, 12.915910167, -11.235917856] m  (NOT USED)
v_fast_world = [2.947699634, 0.385449990, -0.247048107] m/s
bg_imu = [-0.003524117073, -0.001259977133, -0.024174974464] rad/s
ba_scaled_imu = [-0.257519285597, 0.038023434212, 0.016811715102] m/s^2 in FAST-LIO scaled-input convention
g_fast_world = [-1.044150502, 0.413894971, -9.744481601] m/s^2
raw mean acceleration norm = 9.820122262 m/s^2
```

Injected P7 full-motion state:

```text
R = the common R_world_imu above
p = [1.968147, -6.879292, -0.896125] m
v_map = [0.378755116, 2.949394357, -0.236970638] m/s
bg_imu = [-0.003524117073, -0.001259977133, -0.024174974464] rad/s
ba_raw_imu = [-0.257785001975, 0.038062667970, 0.016829061951] m/s^2
g_map = [0.207364641234, -0.983826584322, -9.75733396772] m/s^2
```

The transformed full-state velocity is not copied componentwise: `R_map_fast_world` is

```text
[ 0.247224433286  -0.964884834992  -0.0887543508141
  0.968848583286   0.247535301452   0.00766140972048
  0.014577456930  -0.0878836147231   0.996024080037 ]
```

The two gravity directions differ by 61.470470 deg. This reflects the difference between the frozen gyro-transported static gravity and FAST-LIO's online gravity estimate; it is reported as part of the composite state comparison, not hidden as an invariant.

## Scan-end predictions

The IMU anchor pose is identical in A/B. Predicted scan-end `T_world_imu`:

```text
CURRENT p = [1.932442425, -6.867325747, -0.876862834] m
R =
[ 0.140144450824  -0.989690592727  -0.029530722500
  0.990126776263   0.139993677956   0.007122995096
 -0.00291544678268 -0.030237407306  0.999538493190 ]
anchor->end: translation 0.042297032 m, rotation 0.547376 deg

FULL_STATE p = [2.001284748, -6.605581669, -0.917446419] m
R =
[ 0.134533873321  -0.990491494732  -0.028761707051
  0.990908496243   0.134447991962   0.004908108673
 -0.00099448613703 -0.029160526754  0.999574246705 ]
anchor->end: translation 0.276532201 m, rotation 0.381031 deg
```

Predicted scan-end `T_world_lidar`:

```text
CURRENT p = [1.914067033, -6.783842098, -0.847986799] m
R =
[ 0.139747204402  -0.989788175543  -0.028108475906
  0.990162959150   0.139488776697   0.010963370949
 -0.006930598011 -0.029364072121  0.999544755416 ]

FULL_STATE p = [1.982460353, -6.522262755, -0.888384406] m
R =
[ 0.134133179334  -0.990585510555  -0.027361222193
  0.990950653540   0.133940758214   0.008756456907
 -0.005009236490 -0.028288152419  0.999587258814 ]
```

Difference between scan-end LiDAR seeds: 0.273374041 m and 0.348126 deg.

## Deskew cloud comparison

All 29,063 timed points were retained in both cases and deskewed into the scan-end LiDAR frame. The same-index XYZ difference is larger than the nearest-neighbor difference because motion changes where individual returns land, particularly at the scan beginning.

```text
same-index distance: mean 0.150157 m, median 0.147927 m, P95 0.283401 m, max 0.653250 m
CURRENT -> FULL NN: mean 0.038776 m, median 0.024914 m, P95 0.140847 m, max 0.653251 m
FULL -> CURRENT NN: mean 0.038471 m, median 0.025073 m, P95 0.138030 m, max 0.648465 m

scan time bin       count   median (m)   P95 (m)
0–20 ms             5748    0.266728     0.304246
20–40 ms            5786    0.207476     0.233916
40–60 ms            5729    0.148619     0.174868
60–80 ms            5784    0.095700     0.122616
80 ms–end           6016    0.032988     0.061985
```

This temporal decay is consistent with the two initial motion states producing different early-scan deskew, while they agree more closely near scan end.

## NDT A/B results

| Quantity | CURRENT | FULL_STATE |
|---|---:|---:|
| Full timed points retained | 29,063 / 29,063 | 29,063 / 29,063 |
| Prepared source points | 1,400 | 1,400 |
| Target points | 225,826 | 225,826 |
| Initial overlap `<0.2 / 0.3 / 0.5 / 1.0 m` | 0.1564 / 0.2214 / 0.3579 / 0.5050 | 0.1629 / 0.2407 / 0.3700 / 0.5229 |
| Initial NN mean / median / P90 / P95 (m) | 1.7180 / 0.9869 / 4.1157 / 4.9981 | 1.7027 / 0.9100 / 4.0911 / 4.8497 |
| PCL converged flag | true | true |
| Iterations / wrapper status | 80 / `ITERATION_LIMIT_EXHAUSTED` | 80 / `ITERATION_LIMIT_EXHAUSTED` |
| Prediction-to-terminal translation correction | 1.801025 m | 1.746338 m |
| Prediction-to-terminal rotation correction | 0.285636 rad (16.365758 deg) | 0.285484 rad (16.357056 deg) |
| Final fitness | 0.892480 | 0.959085 |
| Final overlap `<0.2 / 0.3 / 0.5 / 1.0 m` | 0.4343 / 0.5693 / 0.7300 / 0.8764 | 0.4314 / 0.5643 / 0.7200 / 0.8721 |
| Final NN mean / median / P90 / P95 (m) | 0.5134 / 0.2360 / 1.3689 / 2.0914 | 0.5308 / 0.2472 / 1.4279 / 2.1856 |
| Alignment runtime | 209.65 ms | 208.43 ms |

Both raw terminal poses:

```text
CURRENT T_world_lidar terminal:
p = [0.424755812, -5.772893906, -0.908196449] m
R =
[-0.128070027368 -0.990740382127  0.045072866701
  0.990207388544 -0.130282956925 -0.0501565430256
  0.0555643389543 0.0382079357923 0.997723788370]

FULL_STATE T_world_lidar terminal:
p = [0.381851643, -5.824179649, -0.909038901] m
R =
[-0.131462689225 -0.989758903100  0.055631601413
  0.989835729674 -0.134129793535 -0.047269723362
  0.0542474847562 0.048851941826  0.997331789415]
```

Terminal difference: 0.066871 m and 0.656300 deg. The lower FULL_STATE correction did not yield better final NN/overlap; CURRENT was slightly better on those final geometry measures. `PCL converged=true` coexists with 80/80 iterations, so the wrapper correctly labels both results as iteration-limited; neither should be described as a normally terminated registration.

## Interpretation and scope

The full state materially changes scan-end prediction (0.273 m seed shift) and same-index deskew geometry, but barely changes the NDT correction and converges to a nearby terminal. The motion-state mismatch is therefore not the primary cause of TX666's roughly 1.8 m / 16.4 deg NDT correction under this experiment. This is a one-frame diagnostic, not proof that motion state never matters or that the NDT terminal is correct. No GT, map fitting, normalized map, SVD, inverse extrinsic, NDT tuning, multi-start, Dual-U, or post-TX666 tracking was used.

## Code and verification

- Added offline diagnostic executable source: `scripts/p8/corridor01_full_motion_state_tx666.cpp`.
- Added a standalone CMake target in `src/dog_prior_map_fastlio2_frontend_exp/CMakeLists.txt`; no P7 baseline runtime source, NDT parameters, EKF, or stable dog workspace changed.
- Build: `p8_corridor01_full_motion_state_tx666` PASS.
- Existing relevant standalone tests: `fastlio2_propagation_contract_test`, `scan_end_deskew_runtime_test`, `ndt_pose_update_runtime_test`, `frontend_runtime_end_to_end_test` all PASS.
- CTest itself reports `Total Tests: 0`; these test programs are standalone executables, so the four binaries above were invoked directly.
- `git diff --check`: PASS before commit.
