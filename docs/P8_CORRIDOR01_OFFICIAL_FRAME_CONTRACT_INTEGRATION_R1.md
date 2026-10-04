# P8 Corridor01 official frame-contract integration R1

## Outcome

The dataset frame/configuration path is implemented and the causal 67 s
startup runs end to end, including every point in the first scan. The replay
does **not** support stable localization: the first NDT alignment reaches the
80-iteration limit with a `1.943 m / 0.296 rad` correction, and map support is
lost later in the 99-transaction segment.

```text
DATASET_CONFIG = config/datasets/superloc_corridor01.yaml
START_TIME = 67.0 s from rosbag record-time bag start
DARPA_FRAME = IMU
IMU_PREROLL = PASS
67S_REANCHOR = PASS
FIRST_SCAN_COMPLETE = PASS (29,063 / 29,063 points retained)
TRAJECTORY_CONTINUITY = FAIL
FLOOR01_REGRESSION = PASS (first 100 frames)
CTEST = PASS (5/5, with inherited LD_LIBRARY_PATH unset)
FINAL_RESULT = OFFICIAL_SENSOR_FRAME_CONTRACT_CLOSED_BUT_MAP_FRAME_STILL_INCONSISTENT
```

This is a symptom-level result: the initial mismatch is consistent with an
unresolved official-world/normalized-map relation, but this replay does not
prove map-frame mismatch is the sole cause. Gravity direction is transported
from the static window with 61 s of gyro-only rotation integration; that
estimate is itself an uncertainty.

## Dataset and transform contract

The dataset config retains official pose/calibration paths and SHA-256
identities, frozen start time, IMU calibration interval, map-frame convention,
and NDT parameters. The runner reads `laser_to_imu` and `rgb_camera_to_imu`
from the official calibration YAML. Column-vector composition is:

```text
p_A = T_A_B p_B
T_world_lidar  = T_world_imu T_imu_lidar
T_world_camera = T_world_imu T_imu_camera
T_normalized_imu = T_normalized_world T_world_imu
T_normalized_lidar = T_normalized_world T_world_imu T_imu_lidar
```

Serialized official rotation blocks are not perfectly in SO(3). The runner
applies deterministic nearest-proper-SO(3) SVD projection to rotations only;
translations are kept exactly as serialized. `laser_to_imu` raw rotation has
determinant `0.9984426773`, orthogonality error `0.0022139659`, and projection
Frobenius change `0.0011074165`. This is the full official extrinsic, not an
identity/simplified extrinsic. Camera projection is `8.21e-9`; pose rotation
projection is `6.19e-7`.

The raw map/world transform is configured as identity under the task's stated
frame contract. The normalized map is in the recorded `camera_init` frame and
uses the already-established raw-to-normalized transform below. The raw PCD
has no embedded frame ID, so identity is an explicit dataset contract here,
not a frame ID read from the PCD header.

```text
T_WORLD_IMU =
[ 0.135989979134 -0.990409550441 -0.024405900304  1.968147000000]
[ 0.990705105525  0.136027108544  0.000140097550 -6.879292000000]
[ 0.003181110098 -0.024198101899  0.999702121836 -0.896125000000]
[ 0.000000000000  0.000000000000  0.000000000000  1.000000000000]

T_IMU_LIDAR =
[ 0.999991859723 -0.000516138108  0.004001760674  0.080000000000]
[ 0.000519624185  0.999999486419 -0.000870145088  0.029000000000]
[-0.004001309504  0.000872217416  0.999991614344  0.030000000000]
[ 0.000000000000  0.000000000000  0.000000000000  1.000000000000]

T_WORLD_LIDAR = T_WORLD_IMU T_IMU_LIDAR =
[ 0.135571886943 -0.990500518648 -0.022999696288  1.949572144359]
[ 0.990767163313  0.135515820220  0.003986297786 -6.796086602484]
[-0.000831607318 -0.023327773761  0.999727524579 -0.866581192492]
[ 0.000000000000  0.000000000000  0.000000000000  1.000000000000]

T_IMU_CAMERA =
[-0.020500346639 -0.000372770849  0.999789776318  0.179675500000]
[-0.999585604484 -0.020204402176 -0.020503693361  0.047270310000]
[ 0.020207797911 -0.999795800739  0.000041580874 -0.019855360000]
[ 0.000000000000  0.000000000000  0.000000000000  1.000000000000]

T_WORLD_CAMERA = T_WORLD_IMU T_IMU_CAMERA =
[ 0.986718097952  0.044360856413  0.156267429725  1.946248688956]
[-0.156277706537 -0.003257721334  0.987707783553 -6.694859302910]
[ 0.044324638904 -0.999010261043  0.003718160302 -0.916546729753]
[ 0.000000000000  0.000000000000  0.000000000000  1.000000000000]

T_NORMALIZED_WORLD =
[-0.143643602700  0.989002287400 -0.035227581860  7.363630829145]
[ 0.987183928500  0.140696853400 -0.075314566490 -0.644385552171]
[-0.069529883560 -0.045594558120 -0.996537387400  0.346859433882]
[ 0.000000000000  0.000000000000  0.000000000000  1.000000000000]

T_NORMALIZED_IMU =
[ 0.960163462153  0.277649558088 -0.031572780088  0.308851896524]
[ 0.273396628904 -0.956755335125 -0.099365533181  0.398134063031]
[-0.057796224074  0.086775278059 -0.994549988952  1.416694747345]
[ 0.000000000000  0.000000000000  0.000000000000  1.000000000000]

T_NORMALIZED_LIDAR =
[ 0.960426252047  0.277126300211 -0.027971766345  0.392769627278]
[ 0.273294842421 -0.956982622521 -0.097438116103  0.389278922629]
[-0.053771160741  0.085937600504 -0.994848442737  1.384751032815]
[ 0.000000000000  0.000000000000  0.000000000000  1.000000000000]
```

The camera chain is computed and recorded only; camera data does not enter
localization.

## Time, static calibration, and fresh navigation anchor

The rosbag record-time cutoff is `1690254179821741342 ns`, computed as
`bag.get_start_time() + 67.0 s`, matching `rosbag play <bag> -s 67`. The
first post-cut IMU record is `1517157286165072000 ns`; the first post-cut
LiDAR transaction is TX666. TX666 scan interval is
`[1517157286155932903, 1517157286256772352] ns`, with 29,063 points.

The unchanged 200-sample static gate passed on
`[1517157224023904000, 1517157225018848000] ns`:

```text
acceleration std = [0.356038078170, 0.316753910102, 0.263350216811] m/s^2
gyro std         = [0.0350728310874, 0.0447874380996, 0.0490500502501] rad/s
gyro bias        = [0.00830944017588, -0.0238206849755, 0.0369905011350] rad/s
accel bias prior = [0, 0, 0] m/s^2 (initializer does not estimate it)
mean specific force = [1.57831102461, 0.221833535522, 9.69877770424] m/s^2
gravity magnitude = 9.809 m/s^2 (frozen P7 parameter)
```

At the anchor, the implementation creates a fresh state with pose
`T_NORMALIZED_IMU`, zero velocity (`ZERO_VELOCITY_START_ASSUMPTION`), static
gyro bias and configured accelerometer-bias prior, fresh baseline covariance,
and first post-cut IMU header timestamp. It does not reuse early position,
attitude, velocity, covariance, or timestamp and does not integrate
position/velocity from the static epoch to 67 s.

Gravity direction is derived from static mean specific force and transported
to the anchor by bias-corrected midpoint gyro integration from calibration
end to anchor. It uses only causal IMU before the anchor, but spans
`61.146224 s`; accumulated attitude error is not independently bounded. The
resulting normalized-map gravity is
`[2.80333881972, -7.60449832690, 5.52533959662] m/s^2` (norm `9.809`). This
direction uncertainty is a confound in the localization diagnosis.

TX666 begins `9,139,097 ns` before the fresh state timestamp. A causal IMU
sample at `1517157286155072000 ns` brackets before scan start, and the next
sample at `1517157286160064000 ns` brackets after it. The opt-in first-scan
path back-propagates only the IMU pose history over that bracket from the
fresh anchor, then predicts forward to scan end. It retained all `29,063`
points: `dropped=0`, instead of the prior clipped prefix. No pre-cut LiDAR
transaction entered localization.

Adversarial review found that the first-scan helper previously permitted an
anchor value interpolated from either side if the exact anchor IMU sample were
absent. The actual Corridor01 IMU input contains the exact anchor sample
`1517157286165072000 ns` (row 13,396); the helper and replay input validator
now fail closed unless it is present. A regression test removes that sample
and verifies the preroll is rejected.

## 67–77 s replay

NDT remained frozen at resolution `0.8 m`, step `0.08`, epsilon `1e-5`, and
80 maximum iterations. The runner processed 99 transactions (TX666–TX764)
whose bag record times fall in the 10 s playback interval. GT was not read.

TX666:

```text
PCL converged flag = YES; iterations = 80 / 80
wrapper status = ITERATION_LIMIT_EXHAUSTED; effective EKF update = NO
initial overlap <[0.20, 0.30, 0.50, 1.00] m = [0.154286, 0.221429, 0.361429, 0.505714]
final overlap   <[0.20, 0.30, 0.50, 1.00] m = [0.445714, 0.587143, 0.742143, 0.879286]
translation correction = 1.943452 m
rotation correction = 0.296462 rad = 16.988 deg
PCL fitness = 0.8835764
```

The first correction remains in the earlier large-error regime, not a small
local refinement. Across the segment:

```text
effective NDT updates = 25 / 99
iteration-limit results = 13 / 99 (TX666–671, TX695–696, TX703)
zero-iteration passthrough = 61 / 99 (TX704–764)
all serialized states finite = YES
first zero-iteration passthrough = TX704
first frame with <1.00 m overlap = TX706
maximum raw NDT correction = 5.508306 m / 0.453526 rad (TX698)
final corrected IMU position = [75.918271, -237.298944, -131.478967] m
displacement from first corrected position = 282.584941 m
mean / P95 frame time = 78.494 / 223.053 ms
mean / P95 NDT alignment time = 41.849 / 201.443 ms
```

This is a finite but failed localization trajectory: after TX704 the system
continues by prediction with zero map overlap. Sensor frame composition is
exercised, but replay stability is not established.

## Floor01 regression and tests

The default runner mode was replayed on frozen Floor01 inputs for 100
transactions after the exact-anchor guard change. It was compared directly to
the first 100 rows of archived frozen `Run1` (4,127 rows):

```text
frames = 100; LiDAR updates = 100; prediction-only = 0; all states finite
trajectory: all 29 fields identical on 100/100 rows; header identical
registration: all 30 non-timing fields identical on 100/100 rows; header identical
new smoke trajectory SHA-256 = 6aae278fd4996aff76d2d168f9332414f9e1bd9e768b664d412de8bd13a839b7
runner executable SHA-256 = cfee3e0e00b8e205eef2e111b1d0b931dbfff51e705a19e997b6c27b1230173e
peak RSS = 137,332 KiB (this smoke only)
```

CTest was run from `/tmp/p7-baseline-v2-build` with inherited
`LD_LIBRARY_PATH` unset. All five tests passed:

```text
current_frame_ndt_test                 PASS
p7_replay_io_test                      PASS
static_imu_reanchor_contract_test      PASS
scan_preroll_contract_test             PASS
p8_corridor01_frame_contract_test      PASS
5/5 passed
```

An initial CTest invocation inherited an incompatible library path and failed
`current_frame_ndt_test` with unresolved `libusb_set_option`; rerunning with
`env -u LD_LIBRARY_PATH` passed without source changes.

## Code scope and limitations

- `config/datasets/superloc_corridor01.yaml`: official pose/calibration
  provenance, frame/time contract, normalized-map composition, frozen NDT.
- `scripts/p8/run_corridor01_official_frame_replay.py`: hash checks, sensor
  transform composition, rosbag-relative record selection, IMU preroll
  validation, and opt-in runner invocation.
- `p7_single_state_runner.cpp`: opt-in dataset-contract re-anchor and
  calibration-derived gravity direction; default Floor01 startup unchanged.
- `scan_processor.cpp/.hpp`: opt-in causal pre-anchor scan bracket that keeps
  the entire first scan while preserving the default scan path.
- `scan_preroll_contract_test.cpp` and
  `p8_corridor01_frame_contract_test.py`: scan/time/frame contracts; both are
  registered in P7 CMake.

No `dog_visual_loc_ws` files were touched. No GT, inverse pose, map
registration/calibration, visual data, recovery, or NDT parameter tuning was
used. Further work should investigate map-frame and gravity-direction
consistency from evidence; this task does not isolate which one causes the
failed replay.

The read-only adversarial review identified that the first-scan helper
previously permitted the anchor value to be interpolated from either side if
the exact anchor IMU sample were absent. The actual Corridor01 IMU input
contains the exact anchor sample `1517157286165072000 ns` (row 13,396); the
helper and replay input validator now fail closed unless it is present. A
regression test removes the sample and verifies rejection. After this guard,
the 67–77 s replay was repeated into
`docs/p8_corridor01_official_frame_contract_r1/post_review/`: it records
`anchor_imu_sample_present=true` and again processes TX666–TX764 with 25
accepted updates, 13 iteration-limit results, and 61 passthrough frames. The
first-frame correction and later map-support loss are unchanged.

The review found no transform-composition or replay-counter inconsistency. Two
scientific caveats remain: raw-map/world identity is a declared assumption,
not a PCD-embedded frame fact; and the static gravity direction is transported
over 61.146224 s without an independent attitude-error bound. These prevent
attributing the large replay correction to map-frame mismatch alone.

## Git

```text
WORKSPACE = /home/jian/livox_ws/dog_loc_paper_ws
BRANCH = research/p8-challenge-failure-harvest
START_SHA = 66e6c715285ab5266c32c59ba712bed46803c867
dog_visual_loc_ws touched = NO
push = NOT REQUESTED
```
