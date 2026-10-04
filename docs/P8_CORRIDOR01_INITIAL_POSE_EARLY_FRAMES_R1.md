# P8 Corridor01 Initial Pose: Early-Frame Contrast R1

## Result

Using the same direct primary transform, normalized map, source preprocessing,
gyro-only deskew procedure, and NN overlap evaluator as the TX665 test, the
first point-time/IMU-bracketed scan (TX2, reference at assumed bag-relative
0.133906 s) has better overlap at all four thresholds than TX665. Its median
NN distance is lower, although its mean and P95 are worse due to a long tail.
Because the overlap gain is clear but the distance distribution is mixed, the
frozen NDT was run once on TX2: it returned `SUCCESS` after 11 iterations with
a `0.3162 m / 0.07249 rad` correction. TX10 and TX20, near 1 s and 2 s, have
stronger overlap and were evaluated overlap-only.

## Frame selection and time contract

The sensor-time bag-zero anchor is inherited from the prior provisional
mapping: `assumed_s67_reference_ns - 67 s = 1517157219063423943 ns`. This is
not an independently verified rosbag-record-time epoch. To match TX665, each
scan was deskewed to `scan_start + 8,350,920 ns`, the same within-scan offset
as the prior TX665 reference.

TX1 is complete in the adapter (28,866 points) but not usable for this
gyro-only deskew: the IMU stream starts at `1517157219159216000 ns`, about
71.097 ms after TX1 scan start, leaving early point times unbracketed. TX2 is
the first complete transaction with full IMU point-time coverage (28,863
points). TX10's scan spans assumed bag-relative `[0.932436, 1.033214] s`, so
it contains nominal 1 s. TX20 spans `[1.940915, 2.041754] s`, containing
nominal 2 s. TX665 is retained as the prior 67 s comparator.

The same fixed gyro bias reference as the TX665 run was reused:

```text
window: 1517157221789088000 .. 1517157222784032000 ns
mean gyro: [-0.00110574512284, 0.00799054038949, -0.000906438059505] rad/s
```

That bias-reference window is later than TX2, TX10, and TX20. Reusing it keeps
the deskew calculation matched to TX665, but is noncausal for these early
frames; these are offline map-overlap comparisons, not an online-startup
validation.

## Frozen inputs and processing

- Candidate: the exact primary matrix used for the previous TX665 test,
  `T_map_lidar = T_yaml * T_imu_lidar`; same nearest-proper-rotation
  projection and same `T_normalized_raw` map composition.
- Map: normalized baseline Corridor01 map, SHA-256
  `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`.
- Each scan used the same timed adapter, gyro-only rotational deskew, and
  baseline source preprocessing; each yielded 1,400 prepared source points.
- Map target preprocessing yielded 226,164 points; actual PCL target grid
  leaf size `0.800000011921 m` per axis.
- NDT was run only for TX2, with frozen resolution `0.8 m`, step `0.08`,
  epsilon `1e-5`, maximum iterations `80`.
- GT was not read (`GT_USED=false`). No inverse candidate or full replay was
  run.

## Initial overlap comparison

Overlap is fraction of the 1,400 prepared source points with nearest target
distance below each threshold.

| Approximate bag-relative frame | Transaction | Scan interval (assumed s) | Deskew reference (assumed s) | Overlap `<.2/.3/.5/1.0 m` | NN mean / median / P95 (m) |
|---|---:|---|---:|---|---|
| 0 s (first usable) | 2 | .125555–.226394 | .133906 | .260714 / .350714 / .467143 / .641429 | 1.808978 / .581103 / 7.143826 |
| 1 s | 10 | .932436–1.033214 | .940787 | .460000 / .716429 / .835000 / .919286 | .397568 / .208408 / 1.328755 |
| 2 s | 20 | 1.940915–2.041754 | 1.949266 | .472857 / .631429 / .812857 / .929286 | .412546 / .213787 / 1.226725 |
| 67 s comparator | 665 | 66.991649–67.092489 | 67.000000 | .160714 / .237143 / .361429 / .527143 | 1.670754 / .888310 / 4.619541 |

## TX2 frozen NDT result

- PCL convergence flag: `YES`; iterations `11`; wrapper status `SUCCESS`.
- Translation correction: `0.316212545 m`.
- Rotation correction: `0.0724849996 rad` (`4.154°`).
- Initial/final PCL score: `1141.441072 / 1300.032249`.
- Initial/final reported PCL probability: `0.815315 / 0.928594`.
- Final nearest-neighbor fitness: `14.444083 m²`.
- Final overlap `<.2/.3/.5/1.0 m`: `.312857 / .387857 / .477857 / .635714`.
- Alignment time: `3817.75 ms`.

PCL score/probability values are implementation diagnostics, not calibrated
likelihoods. NDT improved the three tighter overlap thresholds, while the
1.0 m overlap decreased slightly (`.641429` to `.635714`); the long-tail NN
statistics remain poor.

## Decision

At the first usable frame, the direct official-pose candidate is more
map-consistent than at TX665 by every overlap threshold and by median NN
distance; frozen NDT also succeeds with a modest correction rather than the
large iteration-limited TX665 correction. Therefore the requested contrast
supports the interpretation that this pose is more compatible with the
sequence start than with the 67 s scan, under the provisional sensor-time
mapping. It does not prove the official `# s 67` semantics or establish an
official epoch.

TX10/TX20 overlap is stronger still. No GT, inverse candidate, EKF, or full
sequence replay was used.

## Artifacts and code scope

CSV sidecars are archived under
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p8_corridor01_initial_pose_0_1_2_r1/`:
`tx2_0s.csv`, `tx2_0s_ndt.csv`, `tx10_1s.csv`, and `tx20_2s.csv`.
Their SHA-256 values, in that order, are:

```text
d39d349a61975a63bf30338c4f7b8511b6f089fd4a925f27dcbe0a7b49a13472
b0b3df649401bf1cc90902ecbf50b3e021845fdbabc38217fd83f6fe196ab4ce
a8a9244d906e8879227c02b95fabf464429d8192273b2734742cca32632c6eac
f34408ca7b2bc65ca61582dee257e796780179a3e0941bd85021be36b5bc76e4
```

The only code change is to the offline
`official_pose_candidate_eval.cpp`: select a transaction, use the same scan-
relative reference offset, and support overlap-only mode. The frozen baseline
localization path was not modified. The evaluator rebuilt successfully and
all four requested frame evaluations completed.
