# Corridor01 Dataset-Specific Initial Velocity R1

## Decision

`FINAL_RESULT = DATASET_INITIAL_VELOCITY_NOT_SUFFICIENT`

The fixed reference-derived velocity made TX666's frozen NDT call finish in 44 iterations and be accepted, whereas the reproduced zero-velocity initialization exhausted 80 iterations and was not applied. It also improved TX667. The 100-frame run did not enter a stable localization regime: only 92/99 post-bootstrap updates were `SUCCESS`, correction magnitudes remained large, velocity and accelerometer bias drifted to implausible values, and covariance contracted while those estimates drifted. Do not proceed to full Corridor01 or tune velocity/bias covariance on these data.

This is evidence that `v0=0` was a consequential startup mismatch, not evidence that the proposed initialization is sufficient or that the trajectory-derived velocity is an unbiased initialization truth. The map/world and exact official YAML epoch relationship remains a project-level caveat; this run follows the task's frozen `T_world_imu` at 67 s contract.

## Reference-derived velocity

Source: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt`, SHA256 `3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03`. It contains 1,385 TUM-like rows: decimal-second absolute sensor timestamps parsed to nanoseconds, XYZ and quaternion. The newer project reference-lineage closure identifies this exact challenge-lineage trajectory as IMU-origin poses in a fixed/world frame (`src/dog_prior_map_localization/docs/superloc_corridor01/P3_R3B_GT_LINEAGE.md`); it supersedes the older P2A audit's `UNKNOWN` pose-origin statement. The map-to-GT fixed transform and the official YAML's exact parser/epoch binding remain not officially confirmed (`P3_R3B_MAP_FRAME_CLOSURE.md`).

Anchor: `t0 = 1517157286165072000 ns`.

The nearest bracketing samples are:

- `t- = 1517157286155932903 ns`, `p- = [156.580726, 54.339934, -12.846159] m`
- `t+ = 1517157286357652903 ns`, `p+ = [157.132107, 54.552800, -12.889235] m`
- `dt = 0.201720 s`

Thus, in the reference fixed/world frame:

`v_ref = (p+ - p-) / (t+ - t-) = [2.7333977791, 1.0552548086, -0.2135435257] m/s`,

`|v_ref| = 2.9377928735 m/s`.

As a local check, a linear least-squares fit to the three available samples in `t0 ± 0.3 s` gives `v_fit = [2.8051282765, 1.0693030576, -0.2287568892] m/s`; `||v_fit-v_ref|| = 0.0746596544 m/s`. There are only three fit samples, so this is a coarse sanity check rather than a strong estimate of acceleration or velocity uncertainty. The central difference was retained.

Under the task's same-epoch pose/frame contract, velocity is transformed as a vector, without translation:

`R_map_ref = R_map_imu(t0) R_ref_imu(t0)^T`

`v_map = R_map_ref v_ref = [0.340226167731, 2.913118327290, -0.169158899609] m/s`.

The implied rotation, conditional on pairing the reference IMU orientation with the official `T_world_imu` orientation at the same physical instant, is:

```text
[ 0.459897049974  -0.884567655165  -0.077683761893 ]
[ 0.887003054756   0.461720029047  -0.006340002400 ]
[ 0.041476309854  -0.065989985704   0.996957891542 ]
```

`|v_map| = 2.9377928735 m/s`. The algebraic velocity-frame transform is defined under this task's frozen assumption; it is not independent proof that the released YAML matrix is the map pose at this exact sensor timestamp. The map↔GT rotation is inferred from the paired orientations under that assumption. Report `VELOCITY_FRAME_PARITY = CONDITIONAL_PASS`, not an independently official frame closure.

The offline derivation is archived in `velocity_derivation.json` and implemented in `scripts/p8/derive_corridor01_initial_velocity.py`. It is not called by the runtime wrapper.

## Runtime contract and implementation

The Corridor01 data configuration now contains:

```yaml
initial_state:
  anchor_timestamp_ns: 1517157286165072000
  first_transaction_id: 666
  replay_frame_count: 100
  use_initial_velocity: true
  velocity_frame: raw_map_world
  velocity_m_s: [0.340226167731097, 2.913118327289612, -0.169158899608618]
  velocity_source: reference_trajectory_offline_finite_difference
  velocity_frame_rotation_source: paired_same_anchor_imu_pose_orientation
  reference_sha256_for_provenance_only: 3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03
  use_initial_biases: true
  gyro_bias_rad_s: [0.0, 0.0, 0.0]
  accel_bias_m_s2: [0.0, 0.0, 0.0]
  use_covariance_overrides: true
  velocity_std_m_s: 0.5
  gyro_bias_std_rad_s: 0.05
  accel_bias_std_m_s2: 0.5
```

The initial tangent-covariance diagonal blocks are set to standard deviation squared at the existing IKFoM ordering:

- velocity, tangent indices 12–14: `0.25 (m/s)^2` each;
- gyro bias, indices 15–17: `0.0025 (rad/s)^2` each;
- accelerometer bias, indices 18–20: `0.25 (m/s^2)^2` each.

Other covariance blocks retain the frozen defaults. The navigation biases start at zero. The accepted static IMU window is still used by the existing startup path to establish gravity; its estimated gyro bias is not copied into this profile. Gravity initialization remains `STATIC_FORCE_CAUSAL_GYRO_TRANSPORT` and is not part of this velocity experiment.

The 200-sample variance gate passed (`accel_std=[0.3560,0.3168,0.2634] m/s^2`; `gyro_std=[0.03507,0.04479,0.04905] rad/s`). Its mean specific force was `[1.5783,0.2218,9.6988] m/s^2`, and the frozen gravity transport initialized `gravity_map=[-8.2939,1.4507,-5.0322] m/s^2`. This was not changed because gravity initialization was frozen by the task, but it is an unresolved common startup-model caveat; passing a variance gate alone does not establish that the window is truly motionless.

Implementation points:

- `frontend_types.hpp`: `InitialStateOverrides` optional mean/covariance values.
- `fastlio2_frontend_ikfom.cpp`: overload of `initializeFromStaticCalibration`; applies velocity/bias means and physical std-squared covariance blocks only at initialization. Existing overload retains its prior semantics.
- `p7_single_state_runner.cpp`: explicit `BASELINE` and `DATASET_VELOCITY` diagnostic profiles; a single `INITIAL_VELOCITY_INJECTION` log is emitted at the re-anchor. Later velocity changes are filter state evolution, not repeated injection.
- `scripts/p8/run_corridor01_dataset_initial_velocity_replay.py`: reads the dataset YAML and launches the frozen 100-frame diagnostic; rejects reference/GT paths in configured runtime paths. The command contains no reference trajectory path. Runtime reports `REFERENCE_USED_AT_RUNTIME=NO`, `GT_UPDATES_AFTER_INITIALIZATION=0`.

The generic frontend update path permits pose measurements to correct velocity and both biases through propagated cross-covariance. This is verified in the new synthetic regression test, but the real Corridor01 run shows that an active update path does not imply that the resulting estimates are well-conditioned or physically valid.

## TX666 paired replay

Same official raw map, official pose input, P7 runtime parameter file, full scan, preprocessing and frozen NDT configuration were used for both profiles. The full TX666 scan was retained: `29,063 / 29,063`, zero dropped points. The scan starts `9.139097 ms` before the re-anchor timestamp; the existing causal IMU pre-roll supplied the bracket and the runner logged `FIRST_SCAN_COMPLETE=PASS`.

Both profiles loaded `T_imu_lidar` from `corridor01_params_official_calibration.txt` through `readP7Parameters`; the diagnostic runner did not pass the separate `T_imu_lidar_row_major` field in the dataset YAML. Runtime logs show translation `[0.080, 0.029, 0.030] m` and this P7 unit-quaternion rotation:

```text
[ 0.999991860  -0.000516138   0.004001761 ]
[ 0.000519624   0.999999486  -0.000870145 ]
[-0.004001310   0.000872217   0.999991614 ]
```

It was identical between A/B and is the existing P7 SO(3) runtime representation, not the literal slightly non-orthonormal public row-major matrix. This remains a paired sensitivity test under the current frozen P7 extrinsic representation; it does not newly establish that P7 applies the literal raw matrix.

| TX666 quantity | Zero-velocity baseline profile | Dataset-velocity profile |
|---|---:|---:|
| Initial scan-end seed XYZ (m) | `[1.914067, -6.783842, -0.847987]` | `[1.944992, -6.516862, -0.863305]` |
| Seed separation | — | `0.269201 m`, `0.2353 deg` |
| Initial overlap `<0.2 / 0.3 / 0.5 / 1.0 m` | `0.1564 / 0.2214 / 0.3579 / 0.5050` | `0.1493 / 0.2229 / 0.3529 / 0.5164` |
| NDT status / iterations | `ITERATION_LIMIT_EXHAUSTED / 80` | `SUCCESS / 44` |
| Seed-to-terminal translation / rotation | `1.801025 m / 0.285636 rad (16.3658 deg)` | `1.471728 m / 0.292000 rad (16.7304 deg)` |
| Final overlap `<0.2 / 0.3 / 0.5 / 1.0 m` | `0.4343 / 0.5693 / 0.7300 / 0.8764` | `0.4193 / 0.5400 / 0.6929 / 0.8579` |
| Fitness | `0.892480` | `1.014182` |
| Measurement applied | `NO` | `YES` |

The new velocity reduced the TX666 translation correction by `18.3%` and changed the optimizer outcome to success, but did not reduce rotation correction; its final overlap and fitness were not better. So TX666 alone is not sufficient evidence of correct localization.

## 100-frame tracking result

Both profiles processed TX666–TX765 (100 transactions). For the requested post-bootstrap statistics, TX667–TX765 gives 99 frames:

| Metric | Reproduced zero-v profile | Dataset-velocity profile |
|---|---:|---:|
| `SUCCESS` / iteration-limit / zero-iteration passthrough | `34 / 6 / 59` | `92 / 7 / 0` |
| Effective pose updates | `34/99 (34.34%)` | `92/99 (92.93%)` |
| Translation correction median / P95 / max | `0.000004 m / 3.155404 m / 5.195602 m` | `0.560628 m / 1.245336 m / 1.938870 m` |
| Rotation correction median / P95 / max | `0.0000016 deg / 24.5438 deg / 77.6545 deg` | `4.5637 deg / 13.2739 deg / 15.0223 deg` |
| Max frame-to-frame translation increment | `5.2728 m` | `1.6421 m` |
| Frame increments `>1 m` / `>10 deg` | `72 / 1` | `28 / 0` |
| Mean / P95 frame time | `113.40 / 445.54 ms` | `83.98 / 205.97 ms` |

The nominal success rate improved over the weak zero-velocity profile, but remained below the specified 95% target. Corrections were still much larger than the proposed stable envelope (`median <0.25 m`, `P95 <0.50 m`, and rotational `median <3 deg`, `P95 <5 deg`).

### First ten transactions, dataset-velocity profile

Bias and speed columns are the posterior state after processing each transaction; the initialized means before TX666 are exactly `v=[0.340226,2.913118,-0.169159] m/s`, `bg=[0,0,0]`, `ba=[0,0,0]`.

| TX | Speed m/s | gyro bias rad/s | accel bias m/s^2 | correction m / deg | iters / status |
|---:|---:|---|---|---:|---|
| 666 | 3.145 | `[-0.000033, 0.000013, -0.000059]` | `[0.000090, -0.001387, -0.000006]` | `1.472 / 16.730` | `44 SUCCESS` |
| 667 | 3.573 | `[-0.000101, 0.000099, -0.000104]` | `[-0.001887, -0.004079, 0.001675]` | `0.085 / 0.438` | `13 SUCCESS` |
| 668 | 4.216 | `[-0.000141, 0.000323, -0.000171]` | `[-0.011307, -0.003773, 0.014766]` | `0.138 / 0.341` | `15 SUCCESS` |
| 669 | 4.909 | `[-0.000141, 0.000323, -0.000171]` | `[-0.011307, -0.003773, 0.014766]` | `1.572 / 0.747` | `80 ITERATION_LIMIT_EXHAUSTED` |
| 670 | 5.708 | `[-0.000141, 0.000323, -0.000171]` | `[-0.011307, -0.003773, 0.014766]` | `1.395 / 4.276` | `80 ITERATION_LIMIT_EXHAUSTED` |
| 671 | 6.031 | `[-0.002713, 0.000597, -0.006329]` | `[0.023641, 0.080105, 0.207417]` | `0.564 / 5.344` | `19 SUCCESS` |
| 672 | 6.540 | `[-0.003213, 0.001522, -0.011400]` | `[-0.025950, 0.219516, 0.360319]` | `0.621 / 3.891` | `36 SUCCESS` |
| 673 | 6.467 | `[-0.001363, 0.001136, -0.016082]` | `[0.168272, 0.450156, 0.510854]` | `0.950 / 3.285` | `60 SUCCESS` |
| 674 | 7.294 | `[-0.001363, 0.001136, -0.016082]` | `[0.168272, 0.450156, 0.510854]` | `1.231 / 2.774` | `80 ITERATION_LIMIT_EXHAUSTED` |
| 675 | 7.260 | `[0.001010, 0.000301, -0.021237]` | `[0.098052, 1.022042, 0.901118]` | `1.399 / 3.270` | `48 SUCCESS` |

At TX765, speed reached `16.1693 m/s`, position was `[-15.6554, 75.9917, -0.4802] m`, and the post-TX666 displacement was `84.075 m`. Biases did not converge to a bounded stable state: final `ba=[0.7159,7.6004,3.9302] m/s^2` (`||ba||=8.5863 m/s^2`) while `P_ba` had shrunk to `[0.004864,0.002996,0.004567]`. `P_bg` shrank from `[0.0025]*3` to approximately `[7.0e-7,5.1e-6,4.8e-6]`. This is an unstable/inconsistent filter evolution, not successful online bias calibration.

No stabilization frame occurred within TX666–TX765. Do not extrapolate this result to the entire dataset.

## Verification and source changes

Directly executed tests passed:

- `fastlio2_propagation_contract_test`
- `ndt_pose_update_runtime_test`
- `frontend_runtime_end_to_end_test`
- `dataset_initial_state_override_test`
- `static_imu_reanchor_contract_test`
- `scan_preroll_contract_test`

The new override test confirms the configured covariance values and a synthetic pose update producing nonzero `delta_v`, `delta_bg`, and `delta_ba`. Build targets compiled. `ctest` itself reported no registered tests in the queried build directories, so the above are direct binary executions, not a CTest suite pass. Python byte-compilation and `git diff --check` were also run.

Full raw logs, CSV trajectories/registrations/runtime, command lines, and environment captures are archived under this directory's `smoke/` and `replay/` subfolders. Runtime ran without reading the reference trajectory: `REFERENCE_USED_AT_RUNTIME=NO`; `GT_UPDATES_AFTER_INITIALIZATION=0`.

The 100-frame runtime profile used a workspace system-library path prepend to avoid loading a conflicting `libusb-1.0.so.0` from `/opt/MVS`; this only affects dynamic linking and is recorded in each run's `runtime_environment.txt`.
