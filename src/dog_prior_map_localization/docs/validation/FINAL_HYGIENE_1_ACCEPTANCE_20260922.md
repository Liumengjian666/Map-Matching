# FINAL-HYGIENE-1 Acceptance

Date: 2026-09-22  
Repository: `fuxianFASTLIVO2`  
Branch: `feature/visual-factor-window`  
Code commit: `89d19bbd86feaf70850b8f6ad37a1ac3e6dae52a`  
Starting SHA: `b0b14755bda27473519ea538ad97c0c6361ddeb4`

## Scope

This stage removes the inactive NDT prediction-feedback path and its dead
legacy delivery surface. It does not change active NDT matching, the local IMU
rotation prior, EKF/OOSM replay, deskew, pending LiDAR handling, or output
topics. The user's staged files and dirty launch change were left untouched.

## Build and contract

Both builds passed:

```text
catkin_make -DCMAKE_BUILD_TYPE=Release -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=OFF --pkg dog_prior_map_localization
catkin_make -DCMAKE_BUILD_TYPE=Release -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=ON --pkg dog_prior_map_localization
```

`oosm_replay_planner_contract_test` returned `OOSM_REPLAY_PLANNER_CONTRACT_PASS`.

## Replay protocol

The valid runs used the same raw loop2 bag, fresh ROS masters, `--clock`,
`--wait-for-subscribers`, `--duration=1090`, RViz disabled, path publishing
disabled, and playback restricted to `/livox/imu /livox/lidar` so the launch
could start without unrelated camera topics. A first unrestricted short
attempt was excluded because it correctly waited for missing camera topics.

| run | directory | NDT rows | OOSM rows | runtime rows | play |
|---|---|---:|---:|---:|---|
| 120 s A | `final_hygiene1_short_A_resources_20260922` | 1200 | 1200 | 59 | PASS |
| 120 s B | `final_hygiene1_short_B_resources_20260922` | 1200 | 1200 | 59 | PASS |
| full A | `final_hygiene1_full_A_20260922` | 10900 | 10900 | 543 | `RC=0` |
| full B | `final_hygiene1_full_B_20260922` | 10900 | 10900 | 543 | `RC=0` |

## Resource results

Values are from the PID-bound sampler (`KiB`, `%`).

| run | NDT RSS mean/peak | NDT CPU mean/peak | EKF RSS mean/peak | EKF CPU mean/peak |
|---|---:|---:|---:|---:|
| short A | 93653 / 93892 | 14.66 / 16.2 | 13922 / 13980 | 5.1717 / 5.9 |
| short B | 93716 / 93932 | 14.83 / 16.1 | 13853 / 14424 | 5.0617 / 5.8 |
| full A | 93545 / 93616 | 16.63 / 17.1 | 13938 / 14488 | 5.8811 / 6.1 |
| full B | 89472 / 93592 | 16.08 / 16.5 | 13497 / 14160 | 5.8843 / 6.0 |

## A/B determinism

Across all 10,900 common NDT timestamps, cloud hash/size, initial guess,
raw NDT pose, final used pose, fitness, convergence, iteration count, and
translation/rotation limiter flags had zero mismatches. OOSM `oosm_result` and
`rollback_stamp` had zero mismatches (`10,889 APPLIED`, `11 NO_HISTORY` in each
run). `imu_history_count` and replay timing fields are scheduler-dependent and
were not treated as geometry mismatches. The two result bags shared 5,287 exact
corrected-odometry timestamps; their maximum translation difference was `0 m`
and maximum rotation difference was `2.96e-6 deg`.

## Reference deviations (not ground truth)

After first-common-sample alignment, full-run NDT versus FAST-LIVO2 reference:

```text
count 10895, mean 0.070739 m, RMSE 0.090203 m, median 0.058038 m,
P90 0.118833 m, P95 0.163199 m, max 0.804716 m, endpoint 0.013138 m
```

Full-run corrected odometry versus the same FAST-LIVO2 reference:

```text
A: mean 0.069414 m, RMSE 0.089393 m, median 0.056493 m,
   P90 0.118058 m, P95 0.163337 m, max 0.806897 m, endpoint 0.014145 m
B: mean 0.069392 m, RMSE 0.089377 m, median 0.056460 m,
   P90 0.117785 m, P95 0.163349 m, max 0.806897 m, endpoint 0.014145 m
```

The FAST-LIO-Localization reference comparison is also a deviation only:
NDT mean/RMSE `0.345367/0.398882 m`; corrected A/B mean/RMSE
`0.345587/0.399150 m` and `0.345607/0.399175 m`.

For the previously sensitive corridor interval (relative time `618--710 s`),
NDT A/B were identical: mean/RMSE/median/P95/max
`0.053304/0.066138/0.043353/0.094021/0.463719 m`. Corrected odometry was
also stable: A `0.050177/0.058392/0.043464/0.088959/0.316203 m`; B
`0.050180/0.058406/0.043551/0.089042/0.316168 m`.

## Delivery boundary

The inactive external prediction subscriber/history and prediction-only CSV
surface are gone. Integrated EKF LiDAR/map ownership remains removed. The
canonical runtime still consists of the independent NDT node, EKF/OOSM/IMU
path, deskew, and the existing diagnostic tools. No Stage3B or visual fusion
work was entered.

## Code size

The NDT translation unit is `1502 -> 1215` lines relative to the starting
commit. The tracked EKF/core runtime source set is `1189 -> 1152` lines and
the EKF header is `241 -> 224` lines. The legacy files removed in the code
commit account for `979` lines; no user-staged file is included in these
counts.

## Classification

`FINAL-HYGIENE-1-PASS` locally. The only deferred item is the user's existing
dirty split launch, which was deliberately not modified; GitHub push remains a
credential-dependent operation.
