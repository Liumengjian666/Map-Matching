# P8 Corridor01 Official Pose Direct Test R1

## Scope and decision

This is a one-scan, four-hypothesis test at transaction TX665 against the
released raw Corridor01 PCD. It does not infer the official `# s 67` epoch,
run an EKF, read GT, or alter the localization baseline. The official YAML's
direction is supported as **direct rather than inverse** by map overlap and
the final NDT basin. A versus C remains unresolved. Neither direct candidate
is acceptable as a frozen baseline `T_map_lidar` without further evidence:
both require about 1.7 m / 0.29 rad correction and reach the 80-iteration
limit.

## Frozen inputs and evaluation contract

- Scan: TX665; scan start/end
  `1517157286055073023` / `1517157286155912472 ns`; 29,067 points.
- Provisional reference: `1517157286063423943 ns`, the prior unit-rate
  bag-to-sensor mapping assumption for bag-relative 67 s. It is not an
  official clock/epoch proof.
- Source: same timed-point stream and preprocessing as the previous
  TX665 screening run: gyro-only rotational deskew to the provisional
  reference, then baseline preprocessing; 1,400 points enter NDT. No
  translational deskew is applied.
- Map: released raw `map/corridor01.pcd`, 338,210 stored points; SHA-256
  `4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a`.
  After the baseline's two 0.15 m target voxel passes, 225,826 points remain.
- NDT: PCL 1.10.0; configured resolution 0.8 m; actual target grid
  `0.800000011921 m` per axis; step 0.08; epsilon `1e-5`; maximum 80
  iterations. Other source/map preprocessing is unchanged.
- GT was not read (`GT_USED=false`).

Input hashes: timed points
`6195878e0d0a68891b392c490078e7ed3be185cd6425d1738ccc13847df31916`;
IMU CSV
`7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa`;
scan filter manifest
`41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d`;
timed-scan index
`e018558448ffa2a34c7bbe4f44d2219c22074193c91057765cea5d175544b1af`.

## Rigid-pose handling

The requested convention was held fixed:

```text
p_imu = T_imu_lidar * p_lidar
T_map_lidar = T_map_imu * T_imu_lidar
```

The task text transcribed the extrinsic's last diagonal as `0.999992652`, but
the local official file
`calibration/corridor01_extrinsics.yaml` (SHA-256
`59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d`)
specifies `0.999993652`. The authoritative file value was used for the final
run; the first run using the transcribed value is retained separately and is
superseded.

With the authoritative value, the supplied `T_imu_lidar` rotation has
`||RᵀR-I||F = 0.00221396591760` and `det(R) = 0.998442677325`, so it is not a
rigid rotation as serialized. The YAML rotation's corresponding values are
`1.2379704e-6` and `0.99999921617`. Since the runtime registration interface
represents orientation as a unit quaternion, both input rotations were
projected once to the closest proper rotation by SVD; translations and
transform directions were preserved. The authoritative extrinsic's
rotational projection change has Frobenius norm `0.00110741654004`. The
effective projected extrinsic is:

```text
[ 0.999991859723 -0.000516138108  0.004001760674  0.080
  0.000519624185  0.999999486419 -0.000870145088  0.029
 -0.004001309504  0.000872217416  0.999991614344  0.030
  0               0               0               1     ]
```

Its rotational change from the supplied block has Frobenius norm
`0.00110741654004`. The same resulting proper candidate transform is used
for overlap, NDT initialization, CSV provenance, and PCD export. This
projection is explicitly part of the evaluation contract; C/D are not tests
of the malformed non-rigid block as an affine warp.

## Effective initial candidates

The following matrices are the effective rigid transforms actually tested
after the above projection:

```text
A: direct YAML as T_map_lidar
[ 0.135989979134 -0.990409550441 -0.024405900304  1.968147000000
  0.990705105525  0.136027108544  0.000140097550 -6.879292000000
  0.003181110098 -0.024198101899  0.999702121836 -0.896125000000
  0                0               0              1             ]

B: inverse YAML as T_map_lidar
[ 0.135989979134  0.990705105525  0.003181110098  6.550552109624
 -0.990409550441  0.136027108544 -0.024198101899  2.863357261000
 -0.024405900304  0.000140097550  0.999702121836  0.944856235353
  0                0               0              1             ]

C: direct YAML as T_map_imu, then multiply T_imu_lidar
[ 0.135571886943 -0.990500518648 -0.022999696288  1.949572144359
  0.990767163313  0.135515820220  0.003986297786 -6.796086602484
 -0.000831607318 -0.023327773761  0.999727524579 -0.866581192492
  0                0               0              1             ]

D: inverse YAML as T_map_imu, then multiply T_imu_lidar
[ 0.136490937865  0.990637181727  0.002863225591  6.590257189318
 -0.990233981163  0.136517120789 -0.028279644292  2.787343340055
 -0.028405746436  0.001024651896  0.999595950201  0.972898889813
  0                0               0              1             ]
```

## Results

Initial overlap is the fraction of the 1,400 NDT-prepared source points with
nearest target distance below 0.20 / 0.30 / 0.50 / 1.00 m. “PCL converged” is
PCL's flag; the wrapper status separately marks iteration-limit exhaustion.
The PCL score/probability columns are implementation scores, not calibrated
Bayesian probabilities; PCL final fitness is a separate nearest-neighbor
quantity in m².

| Candidate | Initial overlap (.2/.3/.5/1.0 m) | Initial NN mean / median / P95 (m) | Initial PCL score / probability | PCL flag; iterations; wrapper status | Correction (m / rad) | Final PCL score / probability; fitness (m²) | Final overlap (.2/.3/.5/1.0 m) | Refined XYZ (m) |
|---|---|---|---:|---|---:|---:|---|---|
| A direct LiDAR | .1579/.2493/.3600/.5250 | 1.6719 / .8864 / 4.6505 | 799.319 / .570942 | YES; 80; `ITERATION_LIMIT_EXHAUSTED` | 1.70318 / .289006 | 1777.475 / 1.269625; .825909 | .4257/.5636/.7221/.8729 | .430335, −6.147207, −.896948 |
| B inverse LiDAR | .0471/.0907/.1300/.2464 | 3.2757 / 3.3909 / 6.5650 | 273.301 / .195215 | YES; 80; `ITERATION_LIMIT_EXHAUSTED` | 2.55613 / .131540 | 783.286 / .559490; 6.808896 | .1729/.2450/.2786/.3257 | 6.175194, .886876, −.631982 |
| C direct IMU | .1571/.2393/.3593/.5257 | 1.6701 / .8883 / 4.6173 | 788.759 / .563399 | YES; 80; `ITERATION_LIMIT_EXHAUSTED` | 1.66065 / .289805 | 1777.871 / 1.269908; .819612 | .4271/.5650/.7250/.8743 | .429352, −6.128388, −.896268 |
| D inverse IMU | .0471/.0907/.1350/.2479 | 3.2918 / 3.4279 / 6.6067 | 274.362 / .195973 | YES; 78; `SUCCESS` | 2.52229 / .131940 | 783.553 / .559681; 6.821955 | .1721/.2450/.2786/.3257 | 6.177386, .885707, −.631861 |

Direct A/C have about 2.6–2.8× the 0.30 m initial overlap of B/D and refine into
the same high-overlap corridor basin. Inverse B/D refine to a quantitatively
poorer basin. D's wrapper success is not evidence that it is
the correct map pose: its final fitness and overlap are poor.

A/C cannot be distinguished by this scan. Their initial translations differ
by only about 0.10 m; their refinements end within about 0.02 m; their final
overlap and fitness differences are small; the initial raw PCL score slightly
favors A, while correction magnitude and final fitness slightly favor C. These
conflicting small differences do not support choosing YAML-as-LiDAR versus
YAML-as-IMU. Both direct candidates also start far from the refined solution
and exhaust the iteration limit.

## PCD and reproducibility artifacts

The evaluator saves the full 29,067-point gyro-deskewed TX665 cloud under
each effective initial and refined transform:

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/
  p8_official_pose_direct_test_r1_authoritative_calib/pcd_raw_map/
    candidate_{A,B,C,D}_{aligned,refined}.pcd
```

All eight PCDs contain 29,067 points (348,976 bytes each). They are intended
for CloudCompare/RViz inspection; no GUI visual inspection was performed in
this run. The complete 60-column CSV, including all 12 elements of the exact
evaluated initial `T_map_lidar`, is
`.../p8_official_pose_direct_test_r1_authoritative_calib/tx665_raw_map.csv`
(SHA-256 `114ec1e92de04618a63dad029f9a2dfb18f4e87384dd53fcf464bed2186d0992`).

The offline evaluator changed is
`src/dog_prior_map_fastlio2_frontend_exp/scripts/p8/official_pose_candidate_eval.cpp`.
It uses the explicit official extrinsic, proper-rotation projection, one
effective transform for every comparison path, exact matrix CSV columns, and
initial/refined PCD export. No baseline frontend, EKF, NDT parameters, map,
GT, or stable dog workspace was changed.

## Result

```text
BEST_DIRECTION = DIRECT
BEST_FRAME_INTERPRETATION = STILL_AMBIGUOUS (A vs C)
SELECTED_T_MAP_LIDAR = NONE
OFFICIAL_POSE_USABLE = NO, not as a frozen direct NDT initialization under
                       this TX665 screening contract
```

This resolves the direction ranking on the released raw map, but not a
production initialization. The 67 s sensor-time mapping remains provisional,
deskew is gyro-only with no translation, the official extrinsic rotation is
not rigid as serialized, and no multi-frame continuity or GT check was used.

The prescribed 5-frame A-vs-C continuation was not run. The frozen P7 runner
requires a causal 200-sample static-IMU window at its initialization epoch;
for the provisional TX665 reference `1517157286063423943 ns`, the exact
runner-selected window is `1517157285065152000`–`1517157286060064000 ns`, with
acceleration standard deviations `[0.998484, 1.615300, 1.056049] m/s²` and
gyro standard deviations `[0.140706, 0.117640, 0.068108] rad/s`. These exceed
the frozen limits `0.50 m/s²` and `0.05 rad/s`. The gate was not relaxed, and
no candidate was selected by substituting an ad-hoc filter initialization.
Thus the TX665 test supports DIRECT over INVERSE but does not close the
LiDAR-vs-IMU frame interpretation.
