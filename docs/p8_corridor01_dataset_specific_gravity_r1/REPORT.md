# Corridor01 dataset-specific gravity R1

Result: `GRAVITY_INITIALIZATION_SIGNIFICANT_BUT_NDT_ISSUE_REMAINS`.

Changing only the initial gravity substantially reduced the subsequent speed and accelerometer-bias drift, improved NDT success, and removed translation jumps above 1 m. It did **not** resolve the TX666 NDT correction, and the new profile still reaches 7.656 m/s by TX765. Freeze this gravity for the next diagnosis; do not call the 100-frame run stable.

## Reference and derivation

Reference file: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt`, SHA256 `3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03`. It is an 8-field TUM trajectory (timestamp, xyz, quaternion xyzw). The official [ICCV 2023 challenge trajectory description](https://superodometry.com/iccv23_challenge_VI) identifies the trajectory as IMU pose in a fixed frame and gives the IMU body axes as forward/left/up. Its samples at `1517157286155932903` and `1517157286357652903` bracket anchor `t0=1517157286165072000 ns`; the anchor attitude was interpolated by quaternion SLERP.

Under the task's frozen same-anchor frame relation:

```text
R_map_ref = R_map_imu_official(t0) * R_ref_imu(t0)^T
g_ref_world = [0, 0, -9.809] m/s^2
g_map_raw = R_map_ref * g_ref_world
g_map = 9.809 * g_map_raw / ||g_map_raw||
```

The matrices used were:

```text
R_ref_imu(t0) =
[ 0.941431868171  -0.335833390811   0.030363978803
  0.336925904566   0.940487602314  -0.044317092927
 -0.013673786033   0.051951934611   0.998555969421 ]

R_map_imu_official (P7 quaternion-decoded pose representation) =
[ 0.135990212695  -0.990409520525  -0.024405812914
  0.990705073191   0.136027344131   0.000140005276
  0.003181195353  -0.024198002017   0.999702123982 ]

R_map_ref =
[ 0.459897049974  -0.884567655165  -0.077683761893
  0.887003054756   0.461720029047  -0.006340002400
  0.041476309854  -0.065989985703   0.996957891542 ]

g_map = [0.762000020412, 0.062189083540, -9.779159958135] m/s^2
||g_map|| = 9.809 m/s^2
```

Frozen causal static calibration in both runs passed unchanged: 200 samples over `[1517157224023904000, 1517157225018848000] ns`; accel std `[0.356038, 0.316754, 0.263350] m/s²`, gyro std `[0.035073, 0.044787, 0.049050] rad/s`. Runtime gravity magnitude is 9.809 m/s².

The official near-rotation was decoded through the existing normalized-quaternion pose representation; no SVD was used. The resultant gravity was magnitude-normalized as required. A 200-sample frozen static window independently gives rotated mean specific force `[1.884294, -0.083884, 9.647412] m/s²`, 11.0623° from +Z (cosine 0.9814): evidence consistent with, but not an official explicit declaration of, world z-up.

Important frame caveat: the official challenge defines the reference trajectory's fixed frame, but does not itself prove that its world axes coincide with the Corridor01 YAML/map world. This derivation therefore uses the previous same-anchor `R_map_ref` relation as explicitly requested; it is not a new official parser proof. Treat the constant as conditional on that relation.

Comparisons:

```text
old static/gyro-transport gravity = [-8.293896510460, 1.450656578710, -5.032231827375] m/s^2
new gravity vs old: angle 63.46694 deg; vector difference 10.31845 m/s^2
new gravity vs FAST-LIO sanity vector [0.207364641234, -0.983826584322, -9.757333967720]:
    angle 6.92108 deg; vector difference 1.18416 m/s^2
```

The earlier `61.4705°` was the old gravity versus the FAST-LIO vector; it is not the angle between old gravity and this newly derived vector.

## Code and runtime contract

- Dataset constant is in `src/dog_prior_map_localization/config/datasets/superloc_corridor01_public_raw_canonical.yaml`, under `initial_state.use_initial_gravity`, frame `raw_map_world`.
- `InitialStateOverrides` adds an opt-in gravity vector, default-off. `initializeFromStaticCalibration()` validates finiteness and the frozen 9.809 m/s² magnitude, then initializes `state.grav`; the legacy path still uses the caller's existing gravity when the override is disabled.
- The P7 diagnostic runner accepts the optional override only with the dataset-velocity profile. It logs the effective state gravity at initialization and emits `INITIAL_GRAVITY_INJECTION_COUNT=1` for the new profile. A stale diagnostic field was corrected to report `gravity_map_effective`, not the unused legacy candidate.
- `run_corridor01_dataset_specific_gravity_replay.py` reads the dataset YAML and supplies only that constant to the runner. It rejects GT/reference paths under runtime `paths`; no reference pose or updates are passed after initialization. Logs report `REFERENCE_USED_AT_RUNTIME=NO`, `GT_UPDATES_AFTER_INITIALIZATION=0`.
- A read-only `propagated_state` snapshot is now written before NDT alignment/update, separate from the existing post-update state fields. This allows scan-end propagation velocity/gravity to be distinguished from corrected state values.

Both profiles used the same official map, official pose, public `T_imu_lidar`, source/target preprocessing, initial velocity `[0.340226167731, 2.913118327290, -0.169158899609] m/s`, zero initial biases, covariance stds `(0.5 m/s, 0.05 rad/s, 0.5 m/s²)`, and NDT `(resolution 0.8 m, step 0.08, epsilon 1e-5, max 80)`. Only gravity changed. Both used complete TX666: scan `[1517157286155932903, 1517157286256772352] ns`, `29063/29063` points, zero dropped points.

## TX666 result

| Measurement | Old gravity | Dataset gravity |
|---|---:|---:|
| Scan-end propagated IMU velocity (m/s) | `[-0.460156, 3.167583, 0.278937]` | `[0.370272, 3.040260, -0.156358]` |
| Scan-end propagated speed (m/s) | 3.21296 | 3.06671 |
| Anchor-to-scan-end IMU translation norm (m) | 0.27913 | 0.27577 |
| Anchor-to-scan-end rotation | 0.38221° | 0.38221° |
| NDT status / iterations | SUCCESS / 44 | SUCCESS / 45 |
| NDT translation correction (m) | 1.47173 | 1.55321 |
| NDT rotation correction | 16.73036° | 16.80523° |
| Initial overlap `<0.2/0.3/0.5/1.0 m` | `.1493/.2229/.3529/.5164` | `.1671/.2400/.3721/.5207` |
| Initial NN median / P95 (m) | `.9551 / 4.7968` | `.9158 / 4.7414` |
| Fitness | 1.01418 | 1.00563 |
| Final overlap `<0.2/0.3/0.5/1.0 m` | `.4193/.5400/.6929/.8579` | `.4207/.5564/.7057/.8571` |

The two scan-end NDT seeds differ by `0.04105 m` and `0°` rotation. The new gravity did not reduce the first-frame NDT correction; it was slightly larger (+0.08148 m, +0.07487°).

Scan-end transforms, `p_A = T_A_B p_B` (the propagated IMU pose is pre-NDT; the LiDAR pose is the NDT initial seed):

```text
OLD T_world_imu_predicted =
[ 0.136727371975 -0.990181476635 -0.029090704377  1.963641360526
  0.990608374135  0.136644003052  0.004844122543 -6.600219101975
 -0.000821490115 -0.029479819510  0.999565038102 -0.892371192647
  0              0              0              1 ]

NEW T_world_imu_predicted =
[ 0.136727371975 -0.990181476635 -0.029090704377  1.999665998600
  0.990608374135  0.136644003052  0.004844122543 -6.605742468858
 -0.000821490115 -0.029479819510  0.999565038102 -0.911254620400
  0              0              0              1 ]

OLD T_world_lidar_seed =
[ 0.136328137645 -0.990276911723 -0.027681708664  1.944991566330
  0.990651931004  0.136136867270  0.008689359449 -6.516862432280
 -0.004836370942 -0.028607542332  0.999579020407 -0.863304875479
  0              0              0              1 ]

NEW T_world_lidar_seed =
[ 0.136328137645 -0.990276911723 -0.027681708664  1.981016204405
  0.990651931004  0.136136867270  0.008689359449 -6.522385799163
 -0.004836370942 -0.028607542332  0.999579020407 -0.882188303232
  0              0              0              1 ]
```

Terminal LiDAR poses (position, quaternion xyzw): old `[0.476361, -6.423987, -0.885253]`, `[0.044667, 0.006578, 0.750998, 0.658759]`; new `[0.432350, -6.403741, -0.887045]`, `[0.043918, 0.007236, 0.751650, 0.658058]`.

## 100-frame causal A/B, TX666–TX765

TX667–TX765 statistics exclude the bootstrap TX666 row:

| Metric | Old gravity | Dataset gravity |
|---|---:|---:|
| NDT SUCCESS | 92/99 (92.93%) | 97/99 (97.98%) |
| Iteration-limit count | 7 | 2 |
| Translation correction median / P95 / max (m) | `.5606 / 1.2453 / 1.9389` | `.3514 / .9605 / 1.3778` |
| Rotation correction median / P95 / max | `4.5637 / 13.2739 / 15.0223°` | `6.6002 / 10.4495 / 11.2677°` |
| Correction >1 m / >10° | 16 / 11 | 4 / 8 |
| Corrected speed initial / TX666 / final=max (m/s) | `2.9378 / 3.1453 / 16.1693` | `2.9378 / 3.0052 / 7.6563` |
| Maximum corrected frame translation increment | 1.6421 m (28 increments >1 m) | 0.7739 m (0 increments >1 m) |
| Maximum corrected frame rotation increment | 9.8212° (0 >10°) | 9.7467° (0 >10°) |
| Final accel bias vector / norm | `[.7159, 7.6004, 3.9302]`, `8.5863 m/s²` | `[-.3111, -.7897, -.0416]`, `0.8498 m/s²` |
| Final gyro bias | `[.00396, -.00298, .01139] rad/s` | `[.01233, -.04246, -.00441] rad/s` |
| `diag(P_ba)` initial → final | `[.25,.25,.25]` → `[.004864,.002996,.004567]` | `[.25,.25,.25]` → `[.005564,.009858,.000104]` |
| `diag(P_bg)` initial → final | `[.0025,.0025,.0025]` → `[6.998e-7,5.145e-6,4.805e-6]` | `[.0025,.0025,.0025]` → `[8.509e-7,4.753e-7,1.143e-5]` |

Relative to old gravity, new gravity reduced maximum/final speed by 52.65% and final `||ba||` by 90.10%. It did not keep speed bounded near the initial 2.94 m/s: it still rose to 7.656 m/s. Translation correction improved, while median rotation correction worsened; therefore this is a significant inertial improvement, not a closed localization result.

Accel-bias compensation check: define `delta_g_map = g_new - g_old`. The expected old-gravity compensation in the IMU frame is `-R_map_imu^T delta_g_map`. Cosine similarity between old-profile `ba` and that expected compensation was `0.9019` at TX675, `0.9979` at TX695, `0.9981` at TX715, and `0.9972` at TX765. Its norm is 10.31845 m/s²; observed old-profile final `||ba||` was 8.58633 m/s². This strongly supports the “bias absorbs gravity mismatch” mechanism, but is a diagnostic association, not a proof that gravity is the only error source.

First-ten-frame values (scan-end propagated speed is before NDT; NDT corrections/status are from the same transaction):

| TX | Old speed / `||ba||` / NDT Δt, ΔR / status-iters | New speed / `||ba||` / NDT Δt, ΔR / status-iters |
|---:|---|---|
| 666 | `3.213 / .001`; `1.472 m, 16.73°`; SUCCESS-44 | `3.067 / .001`; `1.553 m, 16.81°`; SUCCESS-45 |
| 667 | `3.559 / .005`; `.085 m, .44°`; SUCCESS-13 | `3.047 / .013`; `.294 m, 1.11°`; SUCCESS-29 |
| 668 | `4.193 / .019`; `.138 m, .34°`; SUCCESS-15 | `3.202 / .023`; `.108 m, .67°`; SUCCESS-10 |
| 669 | `4.909 / .019`; `1.572 m, .75°`; LIMIT-80 | `3.263 / .054`; `.178 m, .60°`; SUCCESS-7 |
| 670 | `5.708 / .019`; `1.395 m, 4.28°`; LIMIT-80 | `3.293 / .054`; `1.378 m, .95°`; LIMIT-80 |
| 671 | `6.513 / .224`; `.564 m, 5.34°`; SUCCESS-19 | `3.339 / .278`; `.626 m, .25°`; SUCCESS-55 |
| 672 | `6.815 / .423`; `.621 m, 3.89°`; SUCCESS-36 | `3.763 / .521`; `.683 m, 2.31°`; SUCCESS-38 |
| 673 | `7.262 / .701`; `.950 m, 3.29°`; SUCCESS-60 | `4.375 / .816`; `.834 m, 1.83°`; SUCCESS-67 |
| 674 | `7.294 / .701`; `1.231 m, 2.77°`; LIMIT-80 | `5.173 / .972`; `.383 m, 2.06°`; SUCCESS-38 |
| 675 | `8.020 / 1.366`; `1.399 m, 3.27°`; SUCCESS-48 | `5.534 / 1.075`; `.233 m, 2.03°`; SUCCESS-31 |

Bias state evolution after NDT update (vectors are `[x,y,z]`; indices correspond to TX666 bootstrap, frame 10/TX675, frame 30/TX695, frame 50/TX715, and final/TX765):

```text
OLD bg rad/s:
TX666 [-0.000033,  0.000013, -0.000059]
TX675 [ 0.001010,  0.000301, -0.021237]
TX695 [ 0.002243, -0.005150, -0.073305]
TX715 [ 0.023712, -0.006151,  0.003185]
TX765 [ 0.003963, -0.002982,  0.011386]

OLD ba m/s^2:
TX666 [ 0.000090, -0.001387, -0.000006]
TX675 [ 0.098052,  1.022042,  0.901118]
TX695 [ 1.355752,  7.040370,  3.455655]
TX715 [ 0.785016,  7.626978,  3.727441]
TX765 [ 0.715909,  7.600420,  3.930154]

NEW bg rad/s:
TX666 [-0.000033,  0.000012, -0.000059]
TX675 [-0.008810,  0.014717,  0.000822]
TX695 [ 0.001713,  0.078052, -0.024263]
TX715 [ 0.015823, -0.018384, -0.017665]
TX765 [ 0.012335, -0.042456, -0.004411]

NEW ba m/s^2:
TX666 [ 0.000076, -0.001466, -0.000024]
TX675 [-0.880759, -0.615517,  0.017812]
TX695 [ 0.910186, -0.557241, -0.050059]
TX715 [ 0.934505, -0.618915,  0.057522]
TX765 [-0.311121, -0.789721, -0.041581]
```

At TX765 `diag(P_bg)` was old `[6.998e-7, 5.145e-6, 4.805e-6]`, new `[8.509e-7, 4.753e-7, 1.143e-5]`; `diag(P_ba)` was old `[.004864,.002996,.004567]`, new `[.005564,.009858,.000104]`. Biases/covariances for every transaction are in `trajectory.csv`.

## Verification and artifacts

- `dataset_initial_state_override_test`: PASS. It checks gravity override norm/insertion, disabled-override legacy fallback, configured velocity/bias covariance, and that a pose update can still alter velocity and both biases through covariance cross terms.
- `p7_single_state_runner` rebuilt successfully. `ctest --test-dir build -N` reports zero registered CTest cases, so the direct standalone test executable was run explicitly.
- `git diff --check`: PASS; Python derivation, runner, and analysis scripts compile; dataset YAML parses; A/B wrapper verified one velocity injection, the selected effective gravity, and exactly one optional gravity injection in the new profile.
- Per-frame raw CSVs: `replay/{old_gravity,dataset_specific_gravity}/{trajectory,registration,runtime}.csv`; runtime logs and exact commands are alongside them.
- Plot: `replay/gravity_ab_dynamics.png` (speed, ba xyz, bg xyz, translation/rotation corrections, `diag(P_ba)`). Summary: `replay/summary.json`; derivation: `gravity_derivation.json`.
