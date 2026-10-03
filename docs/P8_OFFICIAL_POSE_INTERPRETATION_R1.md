# P8 Official Corridor01 Pose Interpretation R1

## Decision

`OFFICIAL_INITIAL_POSE_NOT_DIRECTLY_USABLE` for the current Corridor01 map and
baseline startup contract. Here “not directly usable” means **not admissible
as a frozen initialization for this baseline under the presently documented
map, epoch, and IMU-startup contract**. It does **not** prove the official YAML
transform is wrong. Under the one-scan raw-map screening hypothesis, the tested
inverse interpretations fit more poorly than the forward ones, but this is
not enough evidence to reject their official semantics. The forward candidates
require large NDT corrections, and the map used by the frozen baseline is a
separately normalized PCD whose origin transform is not documented.

No candidate was frozen. No GT was read. No 5-frame baseline run was attempted.

## S67 transaction and provisional cross-clock mapping

The original rosbag was queried by integer bag-record timestamps. The first
raw bag record is `1690254112821741400 ns`. The `/velodyne_packets` message
corresponding to transaction 665 has:

- bag record time `1690254179813390480 ns`, i.e. `66.991649080 s` after the
  first raw bag record;
- header/first-packet sensor stamp `1517157286055073023 ns`;
- 76 packets.

The ROS API's floating `get_start_time()` is
`1690254112.821741342545556 s`; converting that double back to nanoseconds
does not reproduce the first raw bag record. The earlier report therefore
had a 71 ns error in the mapped sensor reference. The corrected value below
uses integer raw record stamps. This correction changes neither the selected
transaction nor any candidate result at the reported precision.

Transaction 665 is the nearest `/velodyne_packets` record to bag-relative
`67.000000000 s`; the adjacent records are at `66.892573309 s` and
`67.098899294 s`. The official meaning of YAML comment `# s 67` and any exact
bag-clock-to-sensor-clock mapping remain undocumented. For *screening only*,
the evaluator assumes unit-rate clocks over this short interval and carries
the local offset from transaction-665 bag record time to its first packet
sensor stamp forward by `8,350,920 ns`. This yields the assumed sensor
reference `1517157286063423943 ns`. It is not an official epoch contract.

The adjacent record/header intervals show why this carry-forward is only a
screening assumption: tx664→tx665 is `99.075771 ms` in bag-record time versus
`100.790024 ms` between first-packet sensor stamps; tx665→tx666 is
`107.250214 ms` versus `100.859880 ms`. The event-record timing is not a
constant-rate proxy for sensor time at millisecond precision.

| Field | Value |
|---|---:|
| Transaction | 665 |
| Scan start | `1517157286055073023 ns` |
| Scan end | `1517157286155912472 ns` |
| Point count | 29,067 |
| Point offset min / max | 0 / 100,839,449 ns |
| Point timestamp min / max | `1517157286055073023` / `1517157286155912472 ns` |
| Scan duration | 100.839449 ms |
| Relative start / end, local unit-rate mapping assumption | 66.991649080 / 67.092488529 s |
| Position of assumed s=67 in scan | +8.350920 ms after start; 92.488529 ms before end |
| Finite XYZ | 29,067 / 29,067 |

Thus, under the stated local unit-rate mapping assumption, bag-relative
`67.000000000 s` falls inside transaction 665. Without that assumption, only
the selected nearest bag-record transaction and the sensor-clock scan bounds
are established; the exact sensor-time instant represented by `# s 67` is
unresolved.

## Frozen extrinsic convention

The configuration states `p_imu = T_imu_lidar * p_lidar`; the frontend
initialization code computes `T_map_imu = T_map_lidar * inverse(T_imu_lidar)`.
Thus `T_imu_lidar` maps LiDAR coordinates to IMU coordinates:

```text
T_imu_lidar =
[  0.999991859723 -0.000516138108  0.004001760674  0.080
   0.000519624185  0.999999486419 -0.000870145088  0.029
  -0.004001309504  0.000872217416  0.999991614345  0.030
   0                0                0                1     ]
```

No candidate changed this convention. Candidate formulas were:

```text
A: T_map_lidar = T_yaml
B: T_map_lidar = inverse(T_yaml)
C: T_map_imu = T_yaml;          T_map_lidar = T_yaml * T_imu_lidar
D: T_map_imu = inverse(T_yaml); T_map_lidar = inverse(T_yaml) * T_imu_lidar
```

The official calibration/configuration extrinsic is approximately 9.02 cm and
0.24 degrees from identity. Consequently A and C place this one scan almost
identically. Their separation is small relative to the 0.8 m target-grid
length scale, so this one scan has weak power to distinguish the two; it does
not prove the grid makes them mathematically indistinguishable.

The tested YAML rigid transform uses the supplied translation and converts
the rounded rotation matrix to a normalized quaternion, then back to a proper
rotation. This is a small numerical repair of the serialized matrix, not a
claim that this operation is the exact Frobenius-nearest rotation:

```text
T_yaml =
[ 0.135990 -0.990409 -0.024406  1.968147
  0.990705  0.136027  0.000140 -6.879292
  0.003181 -0.024198  0.999702 -0.896125
  0        0         0         1        ]
```

The four actual `T_map_lidar` candidate translations and ZYX RPY values were:

| Candidate | x / y / z (m) | roll / pitch / yaw (rad) | roll / pitch / yaw (deg) |
|---|---|---|---|
| A | 1.968147 / −6.879292 / −0.896125 | −0.0242005 / −0.0031812 / 1.4343827 | −1.3866 / −0.1823 / 82.1841 |
| B | 6.550552 / 2.863359 / 0.944855 | −3.1414526 / 3.1171844 / 1.7072501 | −179.9920 / 178.6015 / 97.8182 |
| C | 1.949572 / −6.796087 / −0.866581 | −0.0233298 / 0.0008315 / 1.4348054 | −1.3367 / 0.0476 / 82.2083 |
| D | 6.590257 / 2.787345 / 0.972898 | −3.1405677 / 3.1131832 / 1.7077705 | −179.9413 / 178.3723 / 97.8480 |

Full candidate transforms evaluated (columns are LiDAR-frame basis vectors and
translation, rows are map-frame components):

```text
A:
[ 0.135990212695 -0.990409520525 -0.024405812914  1.968147000000
  0.990705073191  0.136027344131  0.000140005277 -6.879292000000
  0.003181195353 -0.024198002017  0.999702123982 -0.896125000000
  0              0               0               1             ]
B:
[ 0.135990212695  0.990705073191  0.003181195353  6.550551503910
 -0.990409520525  0.136027344131 -0.024198002017  2.863358912290
 -0.024405812914  0.000140005277  0.999702123982  0.944855430504
  0              0               0               1             ]
C:
[ 0.135572120168 -0.990500488776 -0.022999607991  1.949572166530
  0.990767131472  0.135516055742  0.003986205179 -6.796086601010
 -0.000831522020 -0.023327673922  0.999727526979 -0.866581182711
  0               0               0              1              ]
D:
[ 0.136491171066  0.990637149347  0.002863311809  6.590256603910
 -0.990233951525  0.136517356447 -0.028279544497  2.787345003570
 -0.028405659104  0.001024559579  0.999595952777  0.972898089343
  0               0               0               1             ]
```

## Maps and preprocessing

Primary interpretation test used the released raw map:

- `map/corridor01.pcd`, SHA-256
  `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`.
- YAML SHA-256
  `0670732e26f0d9ee19e6115110e29d0aeca5d26f62ab4848d1629c5f9a6eb62b`.
- raw bag SHA-256
  `c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.
- exact P8 timed-point stream SHA-256
  `6195878e0d0a68891b392c490078e7ed3be185cd6425d1738ccc13847df31916`.
- IMU CSV SHA-256
  `7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa`;
- P7 parameter vector SHA-256
  `fc7bb4c8758da222771aa0c9d18bb56e554a758a5bf181ea129ef9b86e3b4f48`;
- P8 filter manifest / timed-scan index SHA-256
  `41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d` /
  `e018558448ffa2a34c7bbe4f44d2219c22074193c91057765cea5d175544b1af`.

The current baseline configuration instead loads
`map/derived/corridor01_map_normalized.pcd` (SHA-256
`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`). Its
origin relation to the released raw map is not established, so it was only a
cross-check and its scores are not compared numerically with raw-map scores.

The evaluator rotationally deskewed the timed cloud to the assumed s=67
reference using gyro integration and an evaluation-only 200-sample
low-variance gyro reference window,
then applied the baseline `preprocessRegistrationCloud()` source filtering,
0.25 m source voxelization, and deterministic 1,400-point cap. This is **not**
full baseline SE(3) deskew: translational deskew is unavailable without a
valid trajectory seed at this epoch. The primary quantitative results must
therefore be interpreted as direct-pose screening, not a full runtime replay.
The gyro integration uses the packet/IMU shared-clock assumption documented
for this dataset; no local fixed clock offset was estimated in this stage.

The evaluation-only causal low-variance window was
`1517157221789088000`–`1517157222784032000 ns` (about 64 s before the assumed
reference):

- gyro reference mean `[−0.0011057451, 0.0079905404, −0.0009064381] rad/s`;
- maximum-axis accel std `0.498354 m/s²`;
- maximum-axis gyro std `0.037283 rad/s`.

These standard-deviation checks alone do not prove the window was stationary,
and the 64 s separation does not establish gyro-bias stability to the scan.
Subtracting this mean is only a screening assumption; the reported cloud
alignment statistics are conditional on this gyro-only deskew choice.

Target preprocessing is the baseline's finite filtering plus two 0.15 m voxel
passes. The evaluator build pins PCL `1.10.0` exactly. The PCL NDT target grid was verified as
`0.800000012 × 0.800000012 × 0.800000012 m`. Frozen registration parameters:
source voxel 0.25 m, range 0.5–80 m, source cap 1,400, resolution 0.8 m,
step 0.08, epsilon `1e-5`, maximum 80 iterations.

The exact initial PCL NDT score was evaluated through PCL 1.10's own
`computeDerivatives()` at the supplied pose, reproducing its per-align
Gaussian constants and active radius-search support. Its final-pose score
divided by source count matched PCL's reported transformation probability to
within `2.83e-8` on all eight map/candidate cases. `initial_nn_mse_m2` is a separate nearest-
neighbor diagnostic, not the PCL NDT objective.

## Primary results: released raw map

Overlap columns are fractions of the 1,400 prepared source points whose nearest
target point lies within 0.20 / 0.30 / 0.50 / 1.00 m. Initial score is PCL's
NDT objective sum at the supplied pose; the adjacent initial PCL value is that
score divided by 1,400. The reported PCL `transformation_probability` is an
implementation score (higher-is-better), not a calibrated Bayesian
probability. `fitness` is PCL's final nearest-neighbor fitness in m²; it is a
different quantity from the NDT score.

| Candidate | Initial overlap (.2/.3/.5/1m) | NN mean / median / P90 / P95 (m) | Initial NN-MSE (m²) | Initial PCL score / probability | PCL flag; wrapper status, iters | Final fitness / probability | Correction (m / rad) | Align ms | Final overlap (.2/.3/.5/1m) |
|---|---|---|---:|---:|---|---:|---:|---:|---|
| A | .1579/.2493/.3600/.5250 | 1.672/.886/4.097/4.651 | 7.7450 | 799.319 / .57094 | YES; `ITERATION_LIMIT_EXHAUSTED`, 80 | .82591 / 1.26962 | 1.703 / .28901 | 206.642 | .4257/.5636/.7229/.8729 |
| B | .0471/.0907/.1300/.2464 | 3.276/3.391/6.153/6.565 | 15.9426 | 273.302 / .19522 | YES; `ITERATION_LIMIT_EXHAUSTED`, 80 | 6.80890 / .55949 | 2.556 / .13154 | 119.736 | .1729/.2450/.2786/.3257 |
| C | .1571/.2393/.3593/.5257 | 1.670/.888/4.099/4.617 | 7.7178 | 788.759 / .56340 | YES; `ITERATION_LIMIT_EXHAUSTED`, 80 | .81961 / 1.26991 | 1.661 / .28981 | 187.050 | .4271/.5650/.7250/.8743 |
| D | .0471/.0907/.1350/.2479 | 3.292/3.428/6.210/6.607 | 16.1704 | 274.362 / .19597 | YES; `SUCCESS`, 78 | 6.82195 / .55968 | 2.522 / .13194 | 120.331 | .1721/.2450/.2786/.3257 |

PCL's `hasConverged()` flag is YES for all four. The frozen wrapper separately
rejects A/B/C because they reach the 80-iteration cap; D has wrapper status
`SUCCESS`, but still has poor fit and a large correction.

Initial points inside the axis-aligned target-map bounding box were A/B/C/D:
raw map `[0.9971, 1.0000, 0.9971, 1.0000]`; normalized map has the same
fractions. The map AABB is broad, so this is only a containment diagnostic,
not evidence of local map overlap.

Raw-map refined positions:

```text
A: [ 0.430335, -6.147207, -0.896948 ] m
B: [ 6.175194,  0.886876, -0.631981 ] m
C: [ 0.429352, -6.128388, -0.896268 ] m
D: [ 6.177385,  0.885707, -0.631861 ] m
```

A/C have the highest initial overlap on the raw map, but they
move by 1.66–1.70 m and about 16.6 degrees, and both exhaust the 80-iteration
limit. Their refined positions are only 1.9 cm apart. The initial A↔C poses are
about 9.02 cm / 0.24 degrees apart. This does not support either direct pose as
a close scan-time initialization, nor does it resolve the LiDAR-vs-IMU frame.
This one-scan gyro-only screen is not sufficient to reject the official
inverse interpretation as a matter of frame semantics.

## Cross-check: current normalized baseline map

| Candidate | Initial overlap (.2/.3/.5/1m) | NN mean / median / P90 / P95 (m) | Initial NN-MSE (m²) | Initial PCL score / probability | Wrapper status, iters | Final fitness / probability | Correction (m / rad) | Align ms | Final overlap (.2/.3/.5/1m) |
|---|---|---|---:|---:|---|---:|---:|---:|---|
| A | .0486/.0750/.1257/.2807 | 1.809/1.641/3.489/4.120 | 5.3377 | 237.116 / .16937 | `SUCCESS`, 23 | 5.82794 / .32535 | 1.303 / .08636 | 57.473 | .1057/.1407/.2129/.4236 |
| B | .0679/.0993/.1843/.2771 | 2.119/1.991/4.135/4.592 | 6.9717 | 370.903 / .26493 | `SUCCESS`, 57 | 16.11505 / .38556 | 1.528 / .23149 | 81.813 | .1314/.1621/.2043/.3057 |
| C | .0507/.0771/.1293/.2857 | 1.785/1.607/3.446/4.089 | 5.2244 | 244.006 / .17429 | `SUCCESS`, 22 | 5.81061 / .32511 | 1.270 / .08419 | 31.790 | .1057/.1407/.2121/.4229 |
| D | .0679/.1071/.1829/.2700 | 2.148/2.022/4.171/4.622 | 7.1731 | 360.569 / .25755 | `SUCCESS`, 60 | 16.18947 / .38573 | 1.603 / .23079 | 88.472 | .1329/.1614/.2036/.3071 |

None of the four directly overlays the current normalized runtime map well.
This is consistent with (but does not independently prove) a map-origin
normalization mismatch. On this map the inverse candidates B/D have better
initial overlap and higher PCL probabilities than A/C, the opposite of the
raw-map ranking. This map-dependent reversal prevents using the raw-map result
to select the official transform direction. No raw-map-to-normalized-map
transform was guessed.

## 5-frame validation and GT

5-frame EKF validation was not run. A/C remain the better-overlapping pair on
the released raw map, but their score difference is weak, both require large
corrections, and the current runtime map has the opposite candidate ranking;
therefore no candidate could be frozen from this evidence. In addition, the
frozen P7 runner selects the last 200 IMU
samples at or before its initialization timestamp. At the *assumed* reference
`1517157286063423943 ns`, that exact runner-selected window is
`1517157285065152000`–`1517157286060064000 ns`; its last sample is 3.359943 ms
before the initialization time. Its sample standard deviations are:

- acceleration axes: `[0.998484, 1.615300, 1.056049] m/s²`;
- gyro axes: `[0.140706, 0.117640, 0.068108] rad/s`.

Both max-axis values exceed the frozen P7 gates of `0.50 m/s²` and
`0.05 rad/s`, so the exact baseline static initializer rejects this epoch.
The separate earlier low-variance window used only for this evaluator's gyro
deskew is not a substitute for the runner-selected initialization window; it
has no established map pose at its own epoch. Moving the s=67 pose to that
earlier time would be unsupported. The first scan also begins 8.350920 ms
before the assumed epoch; the screen uses all scan points, whereas a real
baseline replay would clip pre-initialization points.

GT was not read. Since no interpretation was selected and frozen, post-hoc GT
error was not computed.

## Code and reproducibility

Only an offline evaluator was added under
`src/dog_prior_map_fastlio2_frontend_exp/scripts/p8/`. No baseline, EKF, NDT
configuration, deskew production path, map, GT, or stable robot workspace was
modified. Corrected integer-clock CSV outputs are stored externally under
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p8_official_pose_interpretation_r1/`
as `candidates_raw_map_integer_clock_v4.csv` and
`candidates_normalized_map_integer_clock_v4.csv`. Both have 48 fields in each
header and data row. A writer/header order defect in older CSVs was fixed
before these v4 files were emitted; the initial `bbox_fraction` column now
follows the PCL score-check fields as declared by the header.

The emitted CSV SHA-256 values are
`605e0fefd39952178ccdd84e5372b3b9b3dc377e97fa92f4fc3c034ef99fb0ff` (raw map)
and `39610e9f291205ff85a88aeefcb9335edee0af40451e4b62aea91f8a6142891c`
(normalized map).

The P8 baseline dataset map discrepancy and unresolved initialization history
remain documented in `docs/P8_CHALLENGE_FAILURE_HARVEST_R1.md`.
