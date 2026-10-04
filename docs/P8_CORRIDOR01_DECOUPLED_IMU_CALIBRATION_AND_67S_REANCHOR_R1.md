# P8 Corridor01 decoupled IMU calibration and 67 s re-anchor R1

## Result

The new startup mode successfully passes the frozen early static-IMU gate and
creates a fresh navigation state at the bag-relative 67 s epoch using the
conditional official DIRECT pose. It does not reuse the early static-window
pose, velocity, covariance, or timestamp, and it does not integrate position
or velocity for the intervening 61.146224 s.

The localization result is not stable. TX666 still needs a large NDT change
(`1.708451 m`, `0.294450 rad`), close to the previous offline TX665 result
(`1.758785 m`, `0.292661 rad`). In the 100-transaction replay, only 25 pose
measurements were accepted; 13 alignments exhausted 80 iterations and 62
later scans became zero-iteration passthrough. By TX765, the IMU position had
moved about `304.012 m` from TX666 and scan/map overlap was zero. The
conditional official-world-to-map relation therefore remains suspect, but
this replay does **not** isolate map-frame mismatch as the sole cause: gravity
orientation was transferred from the static epoch using 61.146224 s of
gyro-only integration, and the first scan prefix was clipped because it
starts 9.139097 ms before the state timestamp.

```text
STATIC_CALIBRATION = PASS
67S_REANCHOR = PASS
CAUSAL_FIRST_SCAN = PARTIAL (TX666 used; 2,603 pre-anchor points clipped)
FLOOR01_DEFAULT_PATH_PARITY = PASS (100 frames)
FINAL_RESULT = 67S_REANCHOR_WORKS_BUT_MAP_FRAME_NOT_CLOSED
```

No GT, inverse pose, map search, or NDT parameter change was used.

## State/calibration contract

`FastLio2IkfomFrontend::initializeStatic()` previously computed both IMU
initialization quantities and a complete navigation state at one epoch. Its
static computations have now been factored into `calibrateStaticImu()`; the
original default startup still applies the same gravity alignment, state
creation, covariance, and timestamp rules.

The static-window values reusable across epochs are:

- sample mean angular velocity, used as gyro-bias estimate `b_g`;
- mean acceleration and `mean_specific_force = mean_acceleration - b_a_prior`;
- configured accelerometer-bias prior `b_a_prior` (the current initializer
  does **not** estimate accelerometer bias from this window);
- axis-wise sample standard deviations, used only by the unchanged gate;
- configured gravity magnitude (`9.809 m/s^2` here).

The static window does not produce an independent navigation covariance. The
re-anchor deliberately creates a fresh copy of the existing fixed initial
covariance: unit diagonal for position, attitude, and velocity; `1e-5` for
gyro/accelerometer bias blocks; `1e-4` for fixed extrinsic rotation; `1e-3`
for fixed extrinsic translation; and `1e-5` for the gravity S2 tangent block.
No early pose, attitude, velocity, covariance, or navigation timestamp is
copied into the new epoch.

For the gravity direction, the static specific-force vector is expressed in
the IMU body frame at the calibration epoch. The implementation estimates
the relative body rotation from that epoch to the re-anchor by midpoint
integration of `(gyro - b_g)` with right multiplication:

```text
R_map_imu(anchor) = R_map_imu(calibration) * R_calibration_to_anchor
R_map_imu(calibration) = R_map_imu(anchor) * R_calibration_to_anchor^T
g_map = -R_map_imu(calibration) * normalize(mean_specific_force) * 9.809
```

This is rotation-only; it does not integrate position or velocity. It is also
a material uncertainty in this test: the attitude bridge lasts `61.146224 s`
and is not an independently validated attitude estimate.

The navigation state is freshly initialized as:

```text
pose      = supplied official T_normalized_imu
velocity  = [0, 0, 0]  (ZERO_VELOCITY_START_ASSUMPTION)
gyro bias = static-window mean
accel bias= configured prior [0, 0, 0]
gravity   = static specific-force direction transported to map as above
timestamp = first IMU header after the bag-record cutoff
covariance= fresh existing makeInitialCovariance()
```

## Frozen input and transforms

Bag-relative startup was taken from the exact record-time cutoff equivalent
to `rosbag play <bag> -s 67`:

```text
bag record-time cutoff = 1690254179.821741343
first post-cut IMU header = 1517157286165072000 ns
first LiDAR transaction = TX666
```

The accepted conditional hypothesis remains `T_raw_officialWorld = I`; it is
not an authoritative frame closure. The raw-to-normalized transform used by
the baseline is:

```text
T_normalized_raw =
[-0.143643605112   0.989002291350  -0.035227580323   7.363630829145
  0.987183929718   0.140696851380  -0.075314573074  -0.644385552171
 -0.069529875709  -0.045594557972  -0.996537371435   0.346859433882
  0                0                0                1]
```

The fresh navigation anchor is the supplied conditional DIRECT pose:

```text
T_normalized_imu =
[ 0.960163465743   0.277649560977  -0.031572778492   0.308851863228
  0.273396627048  -0.956755336447  -0.099365539793   0.398134085225
 -0.057796222809   0.086775269917  -0.994549973184   1.416694747472
  0                0                0                1]
```

The runtime loads the existing P7 calibration parameter file
`src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt`.
Its `T_imu_lidar` is translation `[0.080, 0.029, 0.030] m` and quaternion
`[0.000435591554, 0.002000771806, 0.000258941125, 0.999997870059]` in
`xyzw` order. This is the rigid SO(3) calibration representation already
used by the P7 runtime parameter set; no extrinsic was edited in this task.
Using `T_map_lidar = T_map_imu * T_imu_lidar`, the corresponding re-anchor
LiDAR pose is:

```text
T_normalized_lidar =
[ 0.960426255632   0.277126303099  -0.027971764738   0.392769594401
  0.273294840590  -0.956982623848  -0.097438122721   0.389278944438
 -0.053771159543   0.085937592376  -0.994848426957   1.384751033279
  0                0                0                1]
```

No inverse interpretation was tested.

## Static calibration and re-anchor evidence

The 200-sample interval was frozen before execution and passed the original
per-axis limits (`accel std <= 0.5 m/s^2`; `gyro std <= 0.05 rad/s`):

```text
STATIC_WINDOW = [1517157224023904000, 1517157225018848000] ns
acceleration std = [0.356038078170, 0.316753910102, 0.263350216811] m/s^2
gyro std         = [0.0350728310874, 0.0447874380996, 0.0490500502501] rad/s
gyro bias        = [0.00830944017588, -0.0238206849755, 0.0369905011350] rad/s
mean acceleration / specific force
                 = [1.57831102461, 0.221833535522, 9.69877770424] m/s^2
accel bias prior = [0, 0, 0] m/s^2 (configured; not estimated)
gravity magnitude= 9.809 m/s^2 (configured)
```

The transported gravity vector installed at the new epoch was
`[2.80333882336, -7.60449832715, 5.52533959444] m/s^2` (norm `9.809`). The
corresponding 61.146224 s rotation-only bridge was:

```text
R_calibration_to_anchor =
[-0.619548298818   0.256457412453   0.741882403774
 -0.779969382180  -0.307519299303  -0.545050129271
  0.088361011164  -0.916330440482   0.390551988283]
```

This large accumulated rotation is reported as evidence, not treated as a
validated gravity-direction transfer.

## TX666 first scan

```text
scan_start = 1517157286155932903 ns
scan_end   = 1517157286256772352 ns
original point count = 29,063
scan_start precedes re-anchor by 9,139,097 ns
points clipped before the committed state time = 2,603
points passed to NDT after preprocessing/cap = 1,400
```

The anchor pose at the re-anchor timestamp is given above. After the first
scan's approximately 0.1 s propagation, NDT received this predicted LiDAR
pose:

```text
translation = [0.409324480088, 0.354551176665, 1.368131425171] m
quaternion xyzw = [0.141350156659, -0.0172685667524,
                   0.0488590032583, 0.988602411294]
```

TX666 result:

```text
PCL convergence flag = YES; iterations = 80 / 80
wrapper status = ITERATION_LIMIT_EXHAUSTED; measurement accepted = NO
reported PCL fitness = 0.450234931574
transformation probability = 1.529756308990
translation correction = 1.708451128074 m
rotation correction = 0.294449717287 rad = 16.870726 deg
refined T_map_lidar translation = [1.442624688149, -0.999058485031,
                                   1.505378723145] m
initial overlap <[0.20,0.30,0.50,1.00] m
  = [0.138571, 0.217143, 0.345714, 0.514286]
final overlap <[0.20,0.30,0.50,1.00] m
  = [0.449286, 0.605714, 0.783571, 0.942857]
initial NN mean / median / P95 = 1.558555 / 0.958156 / 4.421408 m
final NN mean / median / P95   = 0.378132 / 0.236488 / 1.090167 m
```

The correction is only `0.050334 m` smaller in translation than the offline
TX665 correction, while its rotation correction is `0.001789 rad` larger.
The direct pose is therefore not a close local initialization under the
current conditional map-frame relation.

## 100-transaction short causal replay

The replay used TX666–TX765, 100 LiDAR transactions spanning `10.076232470 s`
of sensor timestamps after the anchor. NDT parameters and map were unchanged:
resolution `0.8 m`, step `0.08`, epsilon `1e-5`, maximum iterations `80`.

```text
effective NDT measurements = 25 / 100
ITERATION_LIMIT_EXHAUSTED = 13 / 100
ZERO_ITERATION_PASSTHROUGH = 62 / 100
first wrapper SUCCESS = TX672; last wrapper SUCCESS = TX703
maximum correction among all alignments = 5.508306 m / 0.453526 rad
maximum correction among accepted SUCCESS results = 2.906276 m / 0.397024 rad
TX704 onward had zero overlap at all four reported thresholds
max displacement from TX666 corrected pose to TX765 = 304.012 m
TX765 IMU position = [79.092742, -260.321630, -133.739004] m
```

The trajectory is finite but not continuous/stable in the localization sense;
after map support is lost it continues by IMU prediction only. Frame-time mean
was `71.627 ms`, P95 `205.203 ms`; NDT alignment mean was `40.916 ms`, P95
`190.167 ms`. Peak RSS measured on the same re-anchor path was `64,640 KiB`
(`63.125 MiB`).

## Floor01 default-path regression

The default initialization mode remains the original `initializeStatic()`
path. A 100-frame Floor01 smoke was run after the final runner change:

```text
frames = 100; lidar updates = 100; prediction-only = 0; finite = true
trajectory rows = byte-identical to the frozen Run1 first 100 rows
registration header = byte-identical
all non-timing registration fields = identical (alignment_ms excluded)
```

Overlap evaluation and its extra CSV columns are now conditional on the
explicit `--official-pose-reanchor` mode. The default baseline runner does
not perform the extra nearest-neighbor diagnostic queries.

## Code and tests

Changed files are confined to `dog_loc_paper_ws`:

- `frontend_types.hpp`: `StaticImuCalibration` separates calibration outputs
  from navigation state.
- `fastlio2_frontend.hpp` and `fastlio2_frontend_ikfom.cpp`:
  `calibrateStaticImu()` and `initializeFromStaticCalibration()`; the old
  `initializeStatic()` semantics remain unchanged.
- `p7_replay_io.hpp/.cpp`: causal, right/body-frame gyro-only relative
  rotation integration for transporting static gravity direction.
- `p7_single_state_runner.cpp`: explicit optional official-pose re-anchor
  mode, TX start selection, per-frame diagnostics; default mode retains its
  original CSV and matching path.
- `current_frame_ndt.hpp/.cpp`: diagnostic overlap evaluator using the
  preprocessed target cloud; no change to NDT objective/parameters.
- tests and P7 CMake target for the calibration/re-anchor and overlap
  contracts.

Focused build succeeded. CTest:

```text
current_frame_ndt_test                 PASS
p7_replay_io_test                      PASS
static_imu_reanchor_contract_test      PASS
3/3 passed
```

These are software/geometry contract tests, not evidence that Corridor01
localization is stable. GT was not read. `/home/jian/livox_ws/dog_visual_loc_ws`
was not touched.

## Artifacts

- `docs/p8_corridor01_decoupled_imu_calibration_r1/corridor01_67_77/`:
  trajectory, registration, runtime CSVs, run log, resource capture.
- `docs/p8_corridor01_decoupled_imu_calibration_r1/floor01_100_frame_parity/`:
  100-frame Floor01 trajectory, registration, and runtime CSVs.
- `docs/P8_CORRIDOR01_START_FROM_67S_SHORT_CAUSAL_REPLAY_R1.md`:
  preceding blocked-start evidence (retained unchanged).

## Decision

The decoupled API and fresh 67 s state creation work, but the actual
short replay does not maintain map localization. The strongest direct
evidence still points to an unresolved official-world-to-baseline-map spatial
relation because the very first NDT correction is essentially as large as
the earlier offline TX665 correction. However, the later divergence is not
clean evidence of map-frame mismatch alone: the long gyro-only gravity
orientation transfer and the clipped first-scan prefix remain confounders.

```text
CAUSAL_INIT = PASS
CAUSAL_FIRST_SCAN = PARTIAL
TRAJECTORY_CONTINUITY = FAIL
MAP_FRAME = STILL CONDITIONAL / SUSPECTED, NOT PROVEN SOLE CAUSE
FINAL_RESULT = 67S_REANCHOR_WORKS_BUT_MAP_FRAME_NOT_CLOSED
```
