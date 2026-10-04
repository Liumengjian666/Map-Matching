# P8 Corridor01 Map-Frame Closure R1

## Decision

The raw-to-normalized map transform is closed and exactly reproducible. The
stronger transform from the YAML's `world_darpa` frame to the released raw-map
frame is still not authoritatively specified. Thus `T_normalized_raw` is known,
but `T_baselineMap_officialWorld` remains conditional on the unproven identity
`T_raw_officialWorld = I`.

Applying the known map transform removes the earlier raw-versus-normalized
candidate-ranking reversal: direct YAML interpretations A/C remain the
better-supported geometric hypotheses on both maps, while inverse B/D remain
poor. A/C still require about 1.76–1.80 m and 0.292 rad of NDT correction and
exhaust the 80-iteration wrapper limit. No unique pose interpretation is
selected, and no EKF or IMU initialization was run.

## Raw and normalized map evidence

The released raw asset is
`map/corridor01.pcd`, SHA-256
`4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`, with
338,210 points. Its PCD stores XYZ among other fields and a `VIEWPOINT`, but
does not contain an explicit frame ID. It is the official released map file;
that fact alone does not prove its coordinate frame is identical to the
`world` in `world_darpa`.

The baseline target is
`map/derived/corridor01_map_normalized.pcd`, SHA-256
`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`, also
338,210 points. The saved PCD `VIEWPOINT` is byte-for-byte the same as the raw
map header even though XYZ was transformed. It is copied metadata, not a
reliable declaration of the normalized point coordinates' frame. The baseline
map loader consumes XYZ and does not use this PCD viewpoint to transform the
map.

The generation record is
`/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_map_normalized.meta.yaml`
(SHA-256
`8920aa92964da4b8f434d5882748b21e4fb4410f1008595eb7a0604452ba6c22`). It
records:

- source map SHA equal to the released raw-map SHA above;
- normalized-map SHA equal to the current baseline map SHA above;
- initializer `FIRST_SEGMENT_WITH_OFFICIAL_PRIOR`, selected candidate 2,
  50/50 first-segment scans, no GT, and no data after the five-second
  initialization interval;
- `T_ML0` translation `[1.71798264980316, -7.17616987228394,
  0.556529641151428] m` and rotation shown below.

The generator source is
`/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_map_normalizer.cpp`,
SHA-256
`5c38a755fc015abffabafab7ed71f343716ca29028b9f93b4ce35fba24abe808` (same as
the archived `P2B_ADAPTER_SOURCE_MANIFEST.md`). It loads the raw map as
`PointXYZ`, constructs an `Eigen::Affine3f T_ML0` from `corridor01_init.yaml`,
computes `T_ML0.inverse()`, applies `pcl::transformPointCloud`, then saves a
binary PCD. Its implemented relationship is:

```text
p_normalized = T_normalized_raw * p_raw
T_normalized_raw = inverse(T_ML0)
```

Numerically, the metadata's inverse (`T_LM`) is:

```text
T_normalized_raw =
[-0.1436436027   0.9890022874  -0.03522758186   7.363630829145
  0.9871839285   0.1406968534  -0.07531456649  -0.644385552171
 -0.06952988356 -0.04559455812 -0.9965373874    0.346859433882
  0              0              0               1]
```

This is the fixed transform from the raw PCD coordinate numbers into the
baseline normalized PCD coordinate numbers. The normalized frame is the
first-segment initializer's local frame, not an independently documented
official world frame.

## Independent regeneration check

The archived normalizer was rerun on the raw map and archived
`corridor01_init.yaml`. The generated PCD SHA-256 was exactly
`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`, identical
to the current normalized map. Both files were 4,058,751 bytes and contained
338,210 points. Therefore all serialized point records match exactly; the
index-matched pointwise residual is exactly 0 m (mean/median/P95/max all 0 m).
This is stronger than an ICP fit and uses no trajectory or GT.

The first invocation of the archived executable failed because the runtime
loader selected an incompatible `/opt/MVS` libusb. Rerunning with
`LD_LIBRARY_PATH=/lib/x86_64-linux-gnu` succeeded. This affected only the
verification environment, not either PCD.

## Frozen candidate composition after map-frame closure

For the existing four hypotheses only, a normalized-map candidate is formed
as `T_normalized_lidar = T_normalized_raw * T_raw_lidar`. The offline P8
candidate evaluator now accepts an optional row-major 3x4 `MAP_T_RAW` argument
and left-composes it before overlap and baseline-NDT evaluation. The default
is identity, so the previous raw-map mode is unchanged. No production
localization file was modified.

Both map runs used the same TX665 source, the same deterministic 1,400-point
prepared scan, and frozen NDT configuration: PCL 1.10.0; actual target grid
`0.800000011921 m` on each axis; target preprocessing 0.15 m twice; source
voxel 0.25 m; range 0.5–80 m; source cap 1,400; step 0.08; epsilon `1e-5`;
maximum iterations 80. The source remained the previous one-scan screening
cloud: rotational gyro-only deskew at the provisional s≈67 reference, no
translation deskew. These are not a full runtime replay or a proof of `# s 67`
epoch semantics. GT was not read.

Overlap columns are the fractions within 0.20 / 0.30 / 0.50 / 1.00 m. The
normalized-map column uses each candidate after left-composition by
`T_normalized_raw`.

| Candidate | Map | Initial overlap (.2/.3/.5/1.0 m) | NDT correction (m / rad) | PCL converged | Iterations / wrapper result |
|---|---|---:|---:|---|---|
| A direct as map-LiDAR | raw | .1579/.2493/.3600/.5250 | 1.7032 / .28901 | yes | 80 / iteration limit |
| A direct as map-LiDAR | normalized, composed | .1586/.2507/.3614/.5264 | 1.7994 / .29246 | yes | 80 / iteration limit |
| B inverse as map-LiDAR | raw | .0471/.0907/.1300/.2464 | 2.5561 / .13154 | yes | 80 / iteration limit |
| B inverse as map-LiDAR | normalized, composed | .0464/.0900/.1300/.2457 | 2.6561 / .16126 | yes | 80 / iteration limit |
| C direct as map-IMU | raw | .1571/.2393/.3593/.5257 | 1.6607 / .28981 | yes | 80 / iteration limit |
| C direct as map-IMU | normalized, composed | .1607/.2371/.3614/.5271 | 1.7588 / .29266 | yes | 80 / iteration limit |
| D inverse as map-IMU | raw | .0471/.0907/.1350/.2479 | 2.5223 / .13194 | yes | 78 / success |
| D inverse as map-IMU | normalized, composed | .0450/.0893/.1343/.2471 | 2.6660 / .16456 | yes | 80 / iteration limit |

The initial NN mean/median/P90/P95 for normalized composed A/B/C/D were,
respectively:

```text
A: 1.6725 / 0.8947 / 4.1022 / 4.6505 m
B: 3.2766 / 3.3913 / 6.1583 / 6.5650 m
C: 1.6708 / 0.8883 / 4.0939 / 4.6195 m
D: 3.2923 / 3.4279 / 6.2130 / 6.6067 m
```

The broad candidate ordering is now consistent across raw and normalized
coordinates: direct A/C fit substantially better than inverse B/D. The
remaining small numerical differences are expected because the baseline
applies axis-aligned voxel filters and builds axis-aligned NDT cells after
normalization; those discretizations are not exactly equivariant to a rigid
rotation. In particular, A/C are still not acceptable as a close frozen
initialization under this provisional scan-time evaluation: corrections remain
large and the wrapper reaches its 80-iteration limit.

## Spatial conclusion and remaining edge

```text
RAW_TO_NORMALIZED_MAP_TRANSFORM = CLOSED
T_normalized_raw = inverse(T_ML0), as above
YAML_DIRECT_A_OR_C_BETTER_THAN_INVERSE_B_OR_D = SUPPORTED_BY_TX665_GEOMETRY
RAW_MAP_FRAME_ID = NOT_ENCODED_IN_PCD
T_raw_officialWorld = NOT_OFFICIALLY_CLOSED
T_baselineMap_officialWorld = CONDITIONAL, not yet authoritative
GT_USED = false
```

If (and only if) raw PCD coordinates are confirmed to be the YAML's `world`
coordinates, then `T_baselineMap_officialWorld = T_normalized_raw`. Under that
condition, the existing direct candidates become:

```text
A: T_baselineMap_lidar = T_normalized_raw * T_yaml
[ 0.960163393622   0.277649783268 -0.0315728839754  0.308851896524
  0.273396848502  -0.956755279969 -0.0993654600561  0.398134063031
 -0.0577963237993  0.0867751657013 -0.994549992960  1.416694747350
  0                0                0                1]

C: T_baselineMap_lidar = T_normalized_raw * T_yaml * T_imu_lidar
[ 0.960426184049   0.277126525335 -0.0279718707017  0.392769625209
  0.273295061753  -0.956982567414 -0.0974380421475  0.389278943990
 -0.0537712605075  0.0859374881952 -0.994848447046  1.384751021460
  0                0                0                1]
```

A versus C remains a LiDAR-versus-IMU/body interpretation question; their
initial poses differ by only the calibrated LiDAR–IMU extrinsic (about 9 cm),
and this scan does not uniquely resolve it. The official frame label and epoch
are not inferred from GT here.

The previously recorded frozen P7 200-sample startup gate near provisional
s≈67 remains unsatisfied (acceleration-axis standard deviation up to about
1.6153 m/s²; gyro-axis standard deviation up to about 0.1407 rad/s). It was not
changed or rerun in this map-only task.

## Reproduction artifacts

External results directory:
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p8_map_frame_closure_r1/`

- `candidates_raw_identity.csv`, SHA-256
  `72cda9782be198011af621e2379f98803743089e142bd83299b97042ac1b2a57`
- `candidates_normalized_composed.csv`, SHA-256
  `9d74b0326076e64e281741f8006897900befcb213c46e202e9841cb54b915aa7`
- `raw_identity.log`, `normalized_composed.log`

The PCD exact-regeneration output was created in a fresh `/tmp` directory; the
source and destination map assets were not overwritten. The P8 tool build was
under `/tmp/p8-map-frame-eval-build`.
