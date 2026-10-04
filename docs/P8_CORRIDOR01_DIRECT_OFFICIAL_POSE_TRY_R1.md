# P8 Corridor01 Direct Official Pose Try R1

## Decision

The primary interpretation was tested first, followed by only the three
fallback interpretations requested after the primary failed. None is accepted
as a usable, near-terminal `T_map_lidar` initialization for the frozen NDT on
TX665. The map-overlap evidence strongly favors the direct-YAML family over
the inverse-YAML family, but both direct candidates require about 1.8 m and
0.292 rad of NDT correction and exhaust the 80-iteration limit. This does not
show that the official YAML is wrong; the tested scan-time, map-frame, and
point-deskew assumptions remain conditional.

## Frozen test contract

- Transaction: TX665, 29,067 original points; scan interval
  `1517157286055073023`–`1517157286155912472 ns`.
- Provisional `s≈67` reference: `1517157286063423943 ns`; this remains an
  assumed reference, not proof of the official epoch.
- Source: existing timed adapter, gyro-only rotational deskew, baseline source
  preprocessing; 1,400 points passed to NDT. Translational deskew was not used.
- Target: current normalized baseline map
  `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/map/derived/corridor01_map_normalized.pcd`,
  226,164 preprocessed points; SHA-256
  `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`.
- The runner composed each official/raw-map candidate with the frozen raw-to-
  normalized map transform from `corridor01_map_normalized.meta.yaml` before
  overlap, NDT, and PCD evaluation. The metadata transform used was:

  ```text
  T_normalized_raw =
  [-0.143643605112  0.989002291350 -0.0352275803234  7.36363082915
    0.987183929718  0.140696851380 -0.0753145730741 -0.644385552171
   -0.0695298757091 -0.0455945579719 -0.996537371435 0.346859433882
    0                0               0               1]
  ```

- NDT: PCL 1.10; configured and actual target-grid resolution
  `0.8 m` (`0.800000011921 m` actual); step `0.08`; epsilon `1e-5`;
  maximum `80` iterations. No other parameter was changed.
- The supplied candidate rotations are not all exactly in SO(3). The runner
  projected each candidate rotation once to its nearest proper rotation, then
  used that same effective transform for overlap, NDT seed, and PCD export.
  For the primary input matrix the projection Frobenius change was
  `0.00110787219`; the official extrinsic projection change was
  `0.00110741654`.
- GT was not read. No candidate was frozen, so no GT post-hoc error was
  computed.

## Primary candidate

The user-specified raw/official-frame primary was used verbatim as input:

```text
T_map_lidar (primary input) =
[ 0.135469425 -0.989725170 -0.022931736  1.949572160
  0.989994861  0.135406510  0.003975072 -6.796086620
 -0.000832801 -0.023378415  0.999730103 -0.866581202
  0           0            0            1           ]
```

After the runner's rigid projection and map-frame composition, the effective
initial pose used against the normalized baseline map was:

```text
T_normalized_lidar =
[ 0.960426255788  0.277126302782 -0.0279717625442  0.392769575166
  0.273294840501 -0.956982623945 -0.0974381220168  0.389278958130
 -0.0537711572205 0.0859375923166 -0.994848427087 1.38475104247
  0               0               0              1]
```

Effective initial XYZ: `(0.392770, 0.389279, 1.384751) m`; RPY:
`(175.0629°, 3.08235°, 15.8840°)`.

- Initial overlap at NN thresholds `<0.2 / 0.3 / 0.5 / 1.0 m`:
  `0.160714 / 0.237143 / 0.361429 / 0.527143`.
- NN distance mean / median / P95: `1.670754 / 0.888310 / 4.619541 m`.
- Initial PCL NDT score / reported probability: `798.717871 / 0.570513`.
- PCL convergence flag: `YES`; iterations: `80`; wrapper status:
  `ITERATION_LIMIT_EXHAUSTED` (therefore not accepted as converged within the
  frozen iteration budget).
- Correction: `1.758785 m`, `0.292661 rad` (`16.771°`).
- Final PCL score check / probability check: `2002.097259 / 1.430069`;
  final nearest-neighbor fitness: `0.808469 m²`.
- Final overlap: `0.429286 / 0.567857 / 0.737143 / 0.875714`.
- Final median NN distance: `0.247474 m`.
- Alignment time: `36,206.89 ms`.

PCL's score/probability values are implementation-specific scores, not
calibrated likelihoods. The final alignment improves overlap, but its large
pose displacement and iteration-limit status mean this is a substantial
relocalizing correction, not a local refinement of the official initial pose.

## Fallback candidates

All rows use the same scan, map, preprocessing, NDT settings, and map-frame
composition. “User backup C” is the runner's `D` candidate
`inverse(T_yaml) * T_imu_lidar`.

| Candidate | Initial overlap `<.2/.3/.5/1.0 m` | NN mean / median / P95 (m) | PCL flag; iterations; wrapper status | Correction (m / rad) | Final overlap `<.2/.3/.5/1.0 m` | Final fitness (m²) | Align ms |
|---|---|---|---|---:|---|---:|---:|
| Primary `T_yaml * T_imu_lidar` | .1607 / .2371 / .3614 / .5271 | 1.6708 / .8883 / 4.6195 | YES; 80; iteration limit | 1.7588 / .29266 | .4293 / .5679 / .7371 / .8757 | .8085 | 36,206.89 |
| Backup A `T_yaml` | .1586 / .2507 / .3614 / .5264 | 1.6725 / .8947 / 4.6505 | YES; 80; iteration limit | 1.7994 / .29246 | .4229 / .5671 / .7379 / .8743 | .8141 | 35,472.36 |
| Backup B `inverse(T_yaml)` | .0464 / .0900 / .1300 / .2457 | 3.2766 / 3.3913 / 6.5650 | YES; 80; iteration limit | 2.6561 / .16126 | .1750 / .2429 / .2771 / .3321 | 6.3795 | 13,645.79 |
| Backup C `inverse(T_yaml)*T_imu_lidar` | .0450 / .0893 / .1343 / .2471 | 3.2923 / 3.4279 / 6.6067 | YES; 80; iteration limit | 2.6660 / .16456 | .1757 / .2436 / .2793 / .3336 | 6.3889 | 13,732.43 |

The direct candidates end in nearly the same high-overlap basin. Direct versus
inverse is strongly distinguished by initial and refined overlap, but the
LiDAR-versus-IMU interpretation within the direct family is not distinguished
by this one scan. Inverse candidates remain in a much poorer map match after
refinement.

## Artifacts and code

The archived run directory contains per-candidate CSVs and aligned/refined
PCDs. In particular:

- `tx665_primary_official_pose.pcd` and
  `tx665_primary_official_pose_refined.pcd` are the primary initial/refined
  clouds.
- `candidate_A_*`, `candidate_B_*`, and `candidate_D_*` are the three
  fallbacks; `candidate_D_*` is user backup C.
- Output directory:
  `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p8_corridor01_direct_pose_try_r1/`.
- Primary CSV SHA-256:
  `feaca2f905edcf90e3868521197592c54ada376a07b79b53df6fb518483ba9f3`.
- Primary aligned PCD SHA-256:
  `aeb6a1a66b1bc84f0d22f5d3c6cb27e25693f699473a2e90225d62c2f329dd52`.
- Primary refined PCD SHA-256:
  `c6aa1d656f1c3dc470a6d233482da7eb7a30b077b0665f80ce9986f8d84c159b`.

The offline runner change is limited to accepting a candidate selector and the
provided primary matrix, plus using primary-specific PCD names. No baseline
localization, EKF, NDT parameters, map, GT, or stable dog workspace was
modified. Build passed with:

```text
cmake --build /tmp/p8-map-frame-eval-build --target p8_official_pose_candidate_eval -j2
```

No GUI/CloudCompare inspection was performed; map consistency above is judged
from the computed overlap and NN diagnostics.

## Final result

```text
PRIMARY_RESULT = FAIL as a directly usable near-terminal initialization
BEST_DIRECTION = DIRECT (map-overlap evidence only)
BEST_FRAME_INTERPRETATION = LIDAR-vs-IMU STILL AMBIGUOUS
SELECTED_T_MAP_LIDAR = NONE
GT_POSTHOC_ERROR = NOT RUN (no candidate frozen)
FINAL_RESULT = OFFICIAL_POSE_CHAIN_NOT_SUPPORTED for direct TX665 initialization
```

This is a scoped failure of the tested direct initialization contract, not a
claim that the official pose asset is erroneous. The remaining uncertainty is
which frame/epoch/map-coordinate interpretation is intended and whether the
assumed s≈67 scan-time relation and gyro-only source deskew match the official
data protocol.
