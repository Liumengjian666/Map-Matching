# Final delivery acceptance — 2026-09-22

Classification: **FINAL-DELIVERY-PASS-WITH-USER-DIRTY-DEFERRED**

The canonical prior-map delivery chain passed two complete loop2 replays.
The FAST-LIVO2 trajectory used below is a comparison reference, not ground
truth.  Metrics align the reference by interpolation at each output header
timestamp and subtract both trajectories at their first common sample.

## Git

```text
Repository: https://github.com/Liumengjian666/fuxianFASTLIVO2
Branch: feature/visual-factor-window
Stage C commit: 3175f0849d4de81dce9334834f5e2c0e65bf7529
Release commit: recorded by the parent commit of tag delivery-v1.0-20260922
Tag: delivery-v1.0-20260922
Remote before release: 1c038b5349b0698bf7cfffe4f91df18969304e41
Push: FAILED — fatal: could not read Username for 'https://github.com'
```

The working tree also contains 58 original user-staged files and the
user-owned dirty launch change setting `map/load_in_ekf=true`.  They were
preserved and were not included in cleanup or release commits.  Historical
staged evaluation scripts remain explicitly deferred for this reason.

## Builds and contract

Both Release configurations passed:

```text
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=OFF --pkg dog_prior_map_localization
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=ON --pkg dog_prior_map_localization
OOSM_REPLAY_PLANNER_CONTRACT_PASS
```

## Full replay protocol

Input:

```text
/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag
```

Both runs used fresh ROS masters, `/use_sim_time=true`,
`rosbag play --clock -r 1.0 --wait-for-subscribers --duration=1090`, the
canonical split launch, and the same recorded topics:

```text
/dog_livo/ndt_odom
/dog_livo/odom_high_rate
/dog_livo/odom_corrected
/dog_livo/diagnostics
/tf
/tf_static
```

Output directories:

```text
/home/jian/rosbag/loop2/final_delivery_run_A_20260922/
/home/jian/rosbag/loop2/final_delivery_run_B_20260922/
```

Each run produced 10900 NDT and OOSM rows, approximately 10 Hz NDT, and
10889 `APPLIED` plus 11 `NO_HISTORY` OOSM results.  `rosbag play` exited with
code 0 for both runs.  No FATAL, exception, process-death, or
`TF_REPEATED_DATA` message occurred.

## Reference deviation metrics

| Output | Mean (m) | RMSE (m) | Median (m) | P90 (m) | P95 (m) | Max (m) | Endpoint (m) | Hz |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| NDT A | 0.070739 | 0.090203 | 0.058038 | 0.118833 | 0.163199 | 0.804716 | 0.013138 | 10.0000 |
| Corrected A | 0.069395 | 0.089386 | 0.056474 | 0.117934 | 0.163415 | 0.806876 | 0.014155 | 9.9999 |
| NDT B | 0.070739 | 0.090203 | 0.058038 | 0.118833 | 0.163199 | 0.804716 | 0.013138 | 10.0000 |
| Corrected B | 0.069359 | 0.089363 | 0.056421 | 0.117982 | 0.163238 | 0.805711 | 0.014255 | 9.9999 |

The largest corrected deviation was at approximately 1032.38 s, with no
wrong-branch or long-corridor drift.  Corrected deviations at representative
windows were 0.1519/0.1495 m at 618 s, 0.0337/0.0332 m at 705 s, and
0.8069/0.8057 m at 1032.4 s for A/B respectively.

## Determinism and resources

Across A/B, the 10900 common NDT timestamps had zero mismatch for cloud hash,
cloud sizes, initial guess, raw NDT pose, final used pose, fitness,
iterations, convergence, and translation/rotation step-limit flags.  The
largest sign-invariant quaternion difference was about `3.42e-6` degrees and
the largest translation difference was zero.  Corrected odometry had zero
translation difference on 5246 common timestamps and a maximum rotation
difference of about `2.96e-6` degrees.

| Run | NDT RSS mean/peak (KiB) | EKF RSS mean/peak (KiB) | NDT CPU mean/peak | EKF CPU mean/peak |
| --- | ---: | ---: | ---: | ---: |
| A | 91870 / 93432 | 13471 / 14448 | 16.53% / 17.0% | 5.96% / 6.1% |
| B | 86573 / 93760 | 14194 / 14824 | 15.81% / 16.3% | 5.90% / 6.1% |

The Stage C source-level cleanup reduced the NDT translation unit from 2202
to 1502 lines and removed the inactive KD-tree/Hessian/Schur/degeneracy
resource path.  The canonical configuration no longer exposes those dead
parameters or topics.

## Final conclusion

1. The independent NDT + IMU + EKF/OOSM delivery path is reproducible over two
   complete loop2 replays.
2. The active NDT geometry and OOSM result are unchanged by the cleanup, while
   inactive diagnostic resource ownership is removed.
3. Accuracy thresholds for this release are satisfied on both runs: corrected
   mean `<0.10 m`, RMSE `<0.13 m`, P95 `<0.25 m`, max `<1.20 m`, and endpoint
   `<0.05 m` relative to the FAST-LIVO2 comparison trajectory.
4. The only unresolved delivery action is pushing the local commits/tag after
   HTTPS GitHub credentials are configured.  No Stage3B or visual fusion work
   was entered.
