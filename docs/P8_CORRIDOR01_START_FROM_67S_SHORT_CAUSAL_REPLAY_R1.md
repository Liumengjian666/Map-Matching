# P8 Corridor01 start-from-67s causal short replay R1

## Result

`CAUSAL_INIT_BLOCKED`. The raw rosbag-relative `-s 67` slice was resolved
directly from bag record time. Its first LiDAR transaction is TX666. The
frozen P7 initializer was then invoked at the first post-cut IMU header time;
it stopped before loading or processing a scan because the last 200 causal
IMU samples violate the unchanged static-variance gate. No NDT alignment,
state update, or trajectory was produced. No threshold was changed and no GT
was read.

The allowed earlier-IMU warm-up was checked. A qualifying 200-sample window
exists around sensor timestamps `1517157224023904000`–
`1517157225018848000 ns` (about 61.15 s before the start epoch), but the
existing frozen initializer creates its full navigation state at that older
timestamp. It has no existing mode that transfers only IMU calibration to a
new state anchored at the official pose at 67 s. Propagating that old full
state would require a valid earlier global pose; using the official 67 s pose
as if it belonged to the earlier window would be temporally wrong. Therefore
the warm-up option cannot be used without adding a distinct initialization
path, which was outside this run's scope.

## Bag-relative cut and first transaction

Source bag SHA-256:
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.

`rosbag info` and the bag index report:

```text
bag start record time = 1690254112.821741343
rosbag play -s 67 cut = 1690254179.821741343
```

Reading the raw bag index at that exact record-time cutoff gives:

```text
first /imu/data record after cut:
  record time relative to bag start = 67.002860308 s
  header stamp = 1517157286165072000 ns

first /velodyne_packets record after cut:
  record time relative to bag start = 67.098899364 s
  header stamp = 1517157286155932903 ns
  matched raw timed catalog transaction = TX666
  packet count = 76
```

The TX666 point-time catalog row is:

```text
scan_start_ns = 1517157286155932903
scan_end_ns   = 1517157286256772352
point_count   = 29063
scan duration = 0.100839449 s
```

The first scan's header-based start precedes the first post-cut IMU header by
`9.139097 ms`. Pre-cut IMU could provide that bracketing data, but it does not
resolve the initialization gate. The transaction identity is based on the
raw bag record-time cutoff and exact header/catalog match, not the earlier
provisional sensor-time-to-bag-time mapping.

## Frozen initialization gate evidence

The first post-cut IMU header stamp was supplied to the current P7 runner as
the initialization epoch. The precise 200-sample window selected by the
runner spans:

```text
1517157285170144000 .. 1517157286165072000 ns
200 samples; elapsed span 0.994928 s
```

Sample standard deviations by axis `[ax, ay, az, gx, gy, gz]` are:

```text
[1.000273005671, 1.638271435395, 1.064113876268,
 0.144148977232, 0.117641014317, 0.068557758353]
```

Frozen limits from `corridor01_params_official_calibration.txt` are maximum
per-axis acceleration standard deviation `0.5 m/s^2` and gyro standard
deviation `0.05 rad/s`. Both fail:

```text
max_accel_std = 1.638271435395 m/s^2 > 0.5
max_gyro_std  = 0.144148977232 rad/s > 0.05
```

The current baseline runner (`/tmp/p7-baseline-v2-build/p7_single_state_runner`)
returned:

```text
FIRST_BAD_TX=0
error=static_initialization_failed:static_imu_variance_exceeds_gate
```

This is before the scan loop; `NDT alignments = 0`, `processed frames = 0`.
An earlier stale binary in `build/p7_b` had a different CLI and was not used
for the result. The current executable required the documented system-library
override because inherited `/opt/MVS` libusb lacks `libusb_set_option`.

## Direct initialization pose candidate in normalized-map coordinates

The official direct hypothesis remains `T_world_imu = T_yaml`, with the
provided LiDAR-to-IMU extrinsic `p_imu = T_imu_lidar p_lidar`. The known raw
map to normalized map transform is:

```text
T_normalized_raw =
[-0.143643602700   0.989002287400  -0.035227581860   7.363630829145
  0.987183928500   0.140696853400  -0.075314566490  -0.644385552171
 -0.069529883560  -0.045594558120  -0.996537387400   0.346859433882
  0                0                0                1]
```

If raw-map coordinates are the YAML `world_darpa` coordinates, the
conditional composition used for the normalized baseline map is:

```text
T_normalized_imu = T_normalized_raw * T_yaml =
[ 0.960163358670   0.277649368081  -0.031572857953   0.308851896524
  0.273396642948  -0.956754814684  -0.099365636149   0.398134063031
 -0.057796110997   0.086775143190  -0.994549856159   1.416694747345
  0                0                0                1]

T_normalized_lidar = T_normalized_imu * T_imu_lidar =
[ 0.959677203415   0.276908602645  -0.027992721927   0.392769611153
  0.273085123311  -0.956228776463  -0.097372800376   0.389278935756
 -0.053727634647   0.085939139991  -0.994855225964   1.384751041933
  0                0                0                1]
```

For rigid-pose use, the P8 evaluator applies its existing nearest-SO(3)
projection to the stored map transform, official YAML rotation, and calibration
rotation. The effective map transform is therefore:

```text
T_normalized_raw (effective) =
[-0.143643605112   0.989002291350  -0.035227580323   7.363630829145
  0.987183929718   0.140696851380  -0.075314573074  -0.644385552171
 -0.069529875709  -0.045594557972  -0.996537371435   0.346859433882
  0                0                0                1]
```

The actual calibration file
`calibration/corridor01_extrinsics.yaml` (SHA-256
`59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d`) stores
the third-row diagonal value `0.999993652`; this is the value used here and
by the existing P8 evaluator. It differs by `1e-6` from the rounded
`0.999992652` printed in the task prompt.

Using that actual calibration and the same rigid-pose projection convention,
the conditional candidate transforms are:

```text
T_normalized_imu = T_normalized_raw * T_yaml =
[ 0.960163465743   0.277649560977  -0.031572778492   0.308851863228
  0.273396627048  -0.956755336447  -0.099365539793   0.398134085225
 -0.057796222809   0.086775269917  -0.994549973184   1.416694747472
  0                0                0                1]
xyz = [0.308851863, 0.398134085, 1.416694747] m
RPY = [3.054562263, 0.057828448, 0.277398366] rad
    = [175.013526, 3.313326, 15.893756] deg

T_normalized_lidar = T_normalized_imu * T_imu_lidar =
[ 0.960426255632   0.277126303099  -0.027971764738   0.392769594401
  0.273294840590  -0.956982623848  -0.097438122721   0.389278944438
 -0.053771159543   0.085937592376  -0.994848426957   1.384751033279
  0                0                0                1]
xyz = [0.392769594, 0.389278944, 1.384751033] m
RPY = [3.055423959, 0.053797105, 0.277228259] rad
    = [175.062897, 3.082347, 15.884009] deg
```

These are computed candidate initialization transforms only; because the
static IMU gate failed, they were not installed into a running frontend.
`T_normalized_raw` is established by exact raw/normalized map regeneration.
The additional identity assumption `T_raw_officialWorld = I` is not
authoritatively established; hence these normalized initial poses are
conditional, not a closed official-world-to-baseline-map contract. This run
did not use NDT to infer or alter that relation.

## Scope and decision

- Start method: exact raw rosbag record-time cutoff equivalent to `rosbag
  play <bag> -s 67`.
- Initial pose direction: DIRECT only; no inverse candidate.
- Map: existing normalized baseline map; no map modification.
- NDT parameters and static thresholds: unchanged.
- LiDAR localization before 67 s: none.
- GT: not read.
- Estimator/core source: unchanged.
- Short replay frames / NDT successes / iteration limits: `0 / 0 / 0`.
- First-frame predicted pose, NDT correction, overlap, and trajectory
  continuity: not produced because causal initialization failed first.

The failure layer is **IMU initialization**, before map registration or state
update. This result does not establish whether the official map frame is
spatially consistent; it only shows that the frozen P7 static initializer
cannot initialize at the 67 s playback start from its required latest
200-sample window. The requested next action is a separately authorized
initialization design that reuses pre-67 IMU calibration while anchors the
navigation state to the official pose at the actual playback epoch.
