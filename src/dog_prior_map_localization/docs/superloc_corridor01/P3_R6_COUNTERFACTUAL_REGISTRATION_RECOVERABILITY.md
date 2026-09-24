# PAPER-P3-R6 Counterfactual Registration Recoverability

## Scope and protection boundary

This stage is an offline counterfactual diagnostic only.  It does not change
the frozen baseline, paper runtime NDT, EKF, predictor, step limiter, deskew,
map, derived bag, or runtime configuration.  Ground truth is used only to
construct diagnostic initial guesses; it is not used by runtime localization
or by any proposed method.

The offline probe mirrors the online PCL 1.10 lifecycle and parameters:

- source: `/superloc_adapter/points_deskewed`, range `[0.5, 80] m`, voxel `0.25 m`, max `1400` points;
- target: normalized Corridor01 map, target size `226164` points;
- NDT resolution parameter `0.8 m`, effective target grid leaf `1.0 m`;
- step size `0.08`, transformation epsilon `0.001`, maximum iterations `40`.

The accepted input is:

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/derived/corridor01_adapted_full_se3_v2.bag
SHA256: 7c52b3703f2f5f9b7fe291e579187c67c0181e015df6a9a8398cf4795b547ba0
```

The formal baseline is Run A under:

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/baseline_full_se3_deskew_20260923_170936/runA
```

## Oracle definition

No absolute map-to-GT transform is used.  Let the first valid Run-A NDT pair
after the evaluation start be `(T_map_est0, T_gt0)`.  For each later frame:

```text
Delta_T_gt(k) = inverse(T_gt0) * T_gt(k)
T_oracle(k)   = T_map_est0 * Delta_T_gt(k)
```

This is a **FIRST-PAIR-ANCHORED RELATIVE ORACLE**.

The experiment evaluates three independent initializations for every frame:

1. `ONLINE_BASELINE`: logged online `initial_guess`;
2. `ORACLE_RELATIVE`: `T_oracle(k)`;
3. `PERFECT_INCREMENT`: `T_final(k-1) * Delta_T_gt(k-1 -> k)`, which retains the previous carried state but replaces only the current motion increment.

The three NDT runs use identical source cloud, target grid, point order and
PCL parameters.  No counterfactual output is propagated to another frame.

## Online replay gate

The baseline initialization was replayed before counterfactual analysis over
all `65` frames from `0` to `6.455 s` relative to the first valid pair.

```text
translation max difference: 4.76837e-07 m
rotation max difference:    3.13531e-05 deg
fitness max difference:     2.87834e-06
filtered source hashes:     65/65
status:                      PASS
```

The hard gate thresholds (`1e-3 m`, `0.01 deg`, `1e-4`) all pass.

## Segment results

Errors below are translation distance to the anchored oracle endpoint after
NDT.  Values are mean / P95.

| segment | frames | online baseline | oracle-relative | perfect increment |
|---|---:|---:|---:|---:|
| W0 `(0, 3.068] s` | 30 | 0.128701 / 0.279913 m | 0.077960 / 0.160522 m | 0.123910 / 0.291786 m |
| W1 `(3.068, 5.590] s` | 25 | 0.409695 / 0.525600 m | 0.117205 / 0.267560 m | 0.410676 / 0.515431 m |
| W2 `(5.590, 6.5] s` | 9 | 0.645214 / 0.859228 m | 0.199119 / 0.632452 m | 0.676573 / 0.979220 m |

Aggregate mean translation error is `0.306312 m` for the online baseline,
`0.108697 m` for oracle-relative initialization, and `0.308875 m` for the
perfect-current-increment initialization.

## Anchor frames

| anchor | selected frame | baseline raw | oracle raw | perfect-increment raw | baseline/oracle endpoint separation |
|---|---|---:|---:|---:|---:|
| formal 0.25 m (`3.068314 s`) | `R6_0081` (`3.025627 s`) | 0.305658 m | 0.091259 m | 0.300668 m | 0.330520 m / 2.902 deg |
| separate LiDAR 0.5 m (`5.488832 s`) | `R6_0105` (`5.446132 s`) | 0.528497 m | 0.094298 m | 0.516806 m | 0.606595 m / 2.798 deg |
| formal 0.5 m (`5.589692 s`) | `R6_0106` (`5.546992 s`) | 0.538212 m | 0.091678 m | 0.537981 m | 0.621507 m / 2.992 deg |

## Objective comparison

The fixed-pose evaluator uses the same PCL objective implementation as the
alignment run.  Transformation probability is higher-is-better; `getFitnessScore`
is lower-is-better.

- Oracle raw endpoint is closer to the anchored reference in aggregate: **YES**.
- Oracle raw transformation probability is at least the baseline raw value in `15/65` frames; baseline is higher in `50/65`.
- Oracle raw `getFitnessScore` is lower or equal in `54/65` frames.
- Therefore objective preference is **MIXED**, with transformation probability usually preferring the baseline endpoint even when the oracle endpoint is closer to the reference.

This supports the cautious statement:

```text
REFERENCE-CONSISTENT POSE EXISTS,
BUT OBJECTIVE PREFERENCE IS AMBIGUOUS.
```

It does not establish a second mode or a multimodal NDT landscape.

## Frame-level classification

Using a diagnostic near-oracle box of `0.15 m / 2 deg` and a stricter endpoint
clustering tolerance of `0.10 m / 1 deg`, the segment counts are:

| segment | A same endpoint | B oracle preserved / baseline differs | C oracle moves away | D perfect increment recovers | E all differ | F unresolved |
|---|---:|---:|---:|---:|---:|---:|
| W0 | 20 | 8 | 1 | 0 | 0 | 1 |
| W1 | 2 | 19 | 4 | 0 | 0 | 0 |
| W2 | 1 | 6 | 2 | 0 | 0 | 0 |

The W1/W2 result is dominated by `B_ORACLE_PRESERVED_BASELINE_DIFFERS`.  The
perfect-current-increment initialization never provides an aggregate recovery
in W1/W2, so correcting only the current increment is insufficient.

## Oracle local perturbation

At each of the three anchor frames, independent NDT runs were initialized at
oracle pose perturbations of ±0.02/±0.05 m along x/y/z and ±0.2/±0.5 deg around
roll/pitch/yaw.  The near-oracle diagnostic box is `0.15 m / 2 deg`.

- formal 0.25 m anchor: `16/24` remained near oracle — **MIXED/UNSTABLE**;
- LiDAR 0.5 m anchor: `0/24` remained near oracle — **MIXED/UNSTABLE**;
- formal 0.5 m anchor: `0/24` remained near oracle — **MIXED/UNSTABLE**.

This is only a local recoverability check.  No stationarity, gradient,
Hessian, basin, DBSCAN, mode-count or multimodality claim is made.

## Answers to the P3-R6 questions

1. **Does GT-relative-near initialization preserve a reference-consistent pose?**
   **PARTIAL.**  `52/65` oracle-initialized frames remain inside the diagnostic near-oracle box, compared with `20/65` baseline-initialized frames.  Preservation is strongest in W1 but is not universal.
2. **Do baseline and oracle initializations produce different endpoints?**
   **YES** in a substantial early-failure subset, especially W1/W2; the formal 0.5 m anchor separation is `0.621507 m / 2.992 deg`.
3. **Is a perfect current increment sufficient to recover?**
   **NO.**  Its aggregate error (`0.308875 m`) is essentially the baseline (`0.306312 m`), and it does not recover W1/W2.
4. **Is the oracle endpoint objective score better?**
   **MIXED.**  It is closer to the reference, but transformation probability usually prefers the baseline endpoint; `getFitnessScore` often prefers the oracle endpoint.
5. **What best describes early onset?**
   The evidence supports **initialization dependence**, while also showing that local objective preservation is not universal.  Current-increment error alone is not sufficient.

## Scientific boundary and decision

Formal classification for this diagnostic:

```text
A — INITIALIZATION_DEPENDENCE_SUPPORTED
```

This stage does **not** confirm:

- multimodality;
- wrong mode;
- stationary mode;
- Hessian degeneracy;
- unstable feedback;
- a unique physical root cause.

```text
RESULT: PAPER-P3-R6-PARTIAL
NEXT_DECISION: NEEDS_MORE_VALIDATION
P4_ALLOWED = NO
```

A second independent sequence is required before generalizing initialization
dependence beyond Corridor01.

## Outputs

All external outputs are under:

```text
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p3_counterfactual_recoverability/
```

Required CSV files:

- `p3_r6_online_replay_gate.csv`
- `p3_r6_initialization_comparison.csv`
- `p3_r6_objective_comparison.csv`
- `p3_r6_endpoint_separation.csv`
- `p3_r6_segment_statistics.csv`
- `p3_r6_anchor_frames.csv`
- `p3_r6_oracle_local_perturbation.csv`

Plots:

- `early_counterfactual_errors.png`
- `endpoint_separation_timeline.png`
- `objective_by_initialization.png`
- `anchor_frame_comparison.png`
- `oracle_local_perturbation.png`
