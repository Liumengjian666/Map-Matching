# Corridor01 moving-start velocity-prior sweep (R1)

## Scope and interpretation

Four independent 100-transaction replays (TX666–TX765) changed only the initial
velocity standard deviation. Initial velocity, official pose, gravity, zero gyro
and accelerometer bias means, bias covariance, map, sensor data, preprocessing,
and NDT parameters were held fixed. The replay starts at the Corridor01 67 s
anchor; runtime reference/GT access is disabled.

The results are conditional on the fixed reference-derived initial velocity
`[0.340226167731, 2.913118327290, -0.169158899609] m/s` and gravity
`[0.762000020412, 0.062189083540, -9.779159958135] m/s²`. They do not establish
accuracy against GT or performance with independently estimated initial means.

## Provenance boundary

The authoritative archive is `replay_verified2/`. Its shared-config identity
matches across profiles and its before/after runtime input fingerprints match.
The three archived runs have identical trajectories and identical registration
fields except `alignment_ms`.

One setup caveat remains: the dataset YAML declares a public raw
`T_imu_lidar` matrix, but the `DATASET_VELOCITY` runner loads the extrinsic from
the frozen P7 runtime-parameter file instead. The runtime log reports:

```text
R = [ 0.999991859723  -0.000516138108003   0.00400176067421
      0.000519624185203 0.999999486419     -0.000870145087925
     -0.00400130950395  0.000872217416333  0.999991614344 ]
t = [0.080, 0.029, 0.030] m
```

The declared public matrix has `R00=0.999212900` and `R11=0.999218492`.
This is constant across all profiles and therefore does not confound the
velocity-prior comparison, but these runs must not be described as having used
the YAML's public raw rotation matrix. No runtime extrinsic change or rerun was
made for this task.

## Profiles and outcome

| Profile | velocity std (m/s) | variance (m²/s²) | TX667–765 NDT applied | iteration limit | zero-iteration passthrough |
|---|---:|---:|---:|---:|---:|
| V0 | 0.5 | 0.25 | 97/99 (97.98%) | 2 | 0 |
| V1 | 2 | 4 | 89/99 (89.90%) | 10 | 0 |
| V2 | 10 | 100 | 96/99 (96.97%) | 3 | 0 |
| V3 | 100 | 10,000 | 16/99 (16.16%) | 5 | 78 |

Raw accepted NDT pose innovation and actual EKF pose correction are distinct.
The following pairs are median / P95 / max; translation is metres and rotation
is degrees.

| Profile | accepted raw NDT Δt | accepted raw NDT ΔR | actual EKF pose Δt | actual EKF pose ΔR |
|---|---|---|---|---|
| V0 | 0.343 / 0.928 / 1.066 | 6.614 / 10.466 / 11.268 | 0.078 / 0.247 / 0.337 | 0.272 / 0.982 / 1.238 |
| V1 | 0.288 / 0.794 / 1.200 | 5.299 / 13.584 / 14.323 | 0.057 / 0.219 / 0.872 | 0.267 / 0.779 / 1.731 |
| V2 | 0.230 / 1.062 / 1.279 | 3.966 / 6.406 / 6.743 | 0.065 / 0.224 / 0.772 | 0.192 / 0.611 / 1.312 |
| V3 | 1.366 / 3.729 / 3.898 | 9.259 / 28.014 / 28.513 | 0.581 / 1.721 / 1.808 | 1.813 / 4.091 / 4.121 |

TX666 was identical across profiles before the EKF update: the same source hash,
1,400 prepared source points, 225,826 target points, 45 NDT iterations, and
1.553 m / 16.805° raw NDT pose change. Its initial overlap at `<0.2/0.3/0.5/1.0`
m was `0.1671/0.2400/0.3721/0.5207`; final overlap was
`0.4207/0.5564/0.7057/0.8571`; fitness was `1.005625`.

The TX666 NDT update changed velocity by `0.068`, `0.470`, `7.440`, and
`16.511 m/s` for V0–V3 respectively. Post-update speed was `3.005`, `3.023`,
`7.923`, and `16.675 m/s`. Thus V2/V3's larger covariance allows a large
startup velocity impulse; this is not automatically beneficial.

Corrected speed (initial YAML → TX675 → TX695 → TX715 → TX765; m/s):

| Profile | Speed sequence | final accel-bias norm (m/s²) | final gyro-bias norm (rad/s) |
|---|---|---:|---:|
| V0 | 2.938 → 5.648 → 2.166 → 3.170 → 7.656 | 0.850 | 0.0444 |
| V1 | 2.938 → 5.422 → 2.296 → 3.218 → 9.429 | 0.907 | 0.0549 |
| V2 | 2.938 → 1.924 → 2.997 → 1.947 → 2.585 | 1.254 | 0.0193 |
| V3 | 2.938 → 9.116 → 14.430 → 31.864 → 68.658 | 7.253 | 0.0763 |

For V2, `P_v` contracts from approximately `[100,100,100]` before TX666 NDT,
to `[55.30,55.30,55.29]` after TX666, `[6.374,6.374,6.372]` at TX667,
`[0.398,0.398,0.390]` at TX670, and `[0.113,0.113,0.096]` at TX675. By TX765
it is `[0.00474,0.00464,0.00182]`. V3 initially contracts from approximately
10,000 to 122.17 after TX666 and 6.80 after TX667, but later grows to
`[425.52,372.39,65.61]` by TX765 while the trajectory becomes unstable.

V2 offers mixed evidence: lower final speed and substantially lower rotational
NDT corrections than V0, but a large TX666 velocity kick, worse translation
P95/max, more accepted >1 m raw translation corrections (6 vs 3), and a larger
final accelerometer-bias norm. Without an accuracy metric, it is not selected as
a better localization profile. V3 is clearly unstable: 68.66 m/s final speed,
7.25 m/s² final accelerometer-bias norm, 78/99 zero-iteration passthroughs,
and 83/99 frames without an effective NDT update.

## Decision

`VERY_WEAK_VELOCITY_PRIOR_DESTABILIZES_EKF` applies to V3 (`σ_v=100 m/s`,
`P_v=10,000 (m/s)²`). V0 remains the conservative profile; no larger prior is
adopted. V2 is a mixed diagnostic result, not an accuracy winner. This is a
conditional comparison under the actual frozen P7 extrinsic described above.

The profile contract and artifact checks are software/provenance checks; they
are not GT accuracy validation. The standalone initialization test passed, and
the analyzer now defaults to `replay_verified2/` and rejects archives whose
profile hashes or runtime-input fingerprint gates are missing/false.
