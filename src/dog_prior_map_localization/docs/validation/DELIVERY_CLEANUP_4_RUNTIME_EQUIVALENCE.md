# DELIVERY-CLEANUP-4.1 Runtime Equivalence

Date: 2026-09-22
Repository: `https://github.com/Liumengjian666/fuxianFASTLIVO2`
Branch: `feature/visual-factor-window`
HEAD under test: `1c038b5349b0698bf7cfffe4f91df18969304e41`

This stage is validation-only. No localization source, configuration, launch
file, or algorithm parameter was changed.

## Source invariant

The NDT implementation was compared between CLEANUP-3 (`d2e9f265...`) and
CLEANUP-4 (`1c038b534...`):

```text
git diff --quiet d2e9f26508d41e1b13be7020531e3684baba8ee2 \
  1c038b5349b0698bf7cfffe4f91df18969304e41 -- \
  src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp
return code: 0 (empty diff)

BEFORE blob: a176d2054de57adfc66426da547abf44b081646f
AFTER  blob: a176d2054de57adfc66426da547abf44b081646f
```

## Active NDT parameters

The runtime parameter dumps from the canonical CLEANUP-3 and CLEANUP-4 runs
were joined by parameter key. All 96 active keys under `/lidar_update/`,
`/ndt_observation/`, `/map/`, `/imu/`, `/frames`, and `/output/` were equal
except for `/output/ndt_diagnostics_csv_path`; that difference only selects the
diagnostic output file and does not affect NDT execution.

The NDT startup line was identical in both runs:

```text
map=/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd
target=459154  lidar=/livox/lidar  output=/dog_livo/ndt_odom
```

The controlled reruns used the same YAML hash:

```text
4e64239753515fa88c9c5304f6461d63f723e101e2bd5fc23e20ac00fc14316c
```

## Controlled replay protocol

Raw bag:

```text
/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag
```

For all three controlled runs (CLEANUP-3 BEFORE, CLEANUP-4 RUN-A, and
CLEANUP-4 RUN-B):

```text
rosbag play --clock --wait-for-subscribers -r 1.0 --duration=120 BAG \
  --topics /livox/imu /livox/lidar
```

Each run produced 1200 NDT frames over 119.900423 s (`9.9999647 Hz`) and
completed with `PLAY_RC=0`. The controlled output directories are:

```text
/home/jian/rosbag/loop2/delivery_cleanup4_before_valid2_20260922/
/home/jian/rosbag/loop2/delivery_cleanup4_repeat_a_valid_20260922/
/home/jian/rosbag/loop2/delivery_cleanup4_repeat_b_valid_20260922/
```

## NDT geometry equivalence

All comparisons below join by exact `lidar_header_stamp`.

| comparison | common | cloud hash mismatch | initial translation max/P95 | initial rotation max/P95 | raw translation max/P95 | raw rotation max/P95 | final translation max/P95 | final rotation max/P95 | fitness max/P95 | iteration mismatch | convergence mismatch |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| CLEANUP-4 RUN-A vs RUN-B | 1200 | 0 | 0 / 0 m | 0 / 0 deg | 0 / 0 m | 0 / 0 deg | 0 / 0 m | 0 / 0 deg | 0 / 0 | 0 | 0 |
| CLEANUP-3 BEFORE vs CLEANUP-4 RUN-A (same controlled protocol) | 1200 | 0 | 0 / 0 m | 0 / 0 deg | 0 / 0 m | 0 / 0 deg | 0 / 0 m | 0 / 0 deg | 0 / 0 | 0 | 0 |

Thus the cross-version delta is exactly within (and in fact equal to zero
against) the same-version repeatability envelope.

## Historical first divergence and OOSM boundary evidence

The earlier canonical comparison that motivated this stage used a different
startup/playback protocol (full-bag playback without the controlled
`--wait-for-subscribers` topic selection). It showed one early NDT initial-guess
divergence and one OOSM result divergence; those numbers are retained here as
diagnostic evidence, not as the controlled acceptance comparison.

First historical NDT initial-guess divergence:

```text
lidar stamp: 1786179479.6107185
relative time: 0.1010108 s
cloud hash: equal
translation difference: 0 m
rotation difference: 0.088731745 deg
```

At this exact frame, both runs had the same local-IMU interval
`[1786179479.5599465, 1786179479.6604776]`, but their history boundaries
differed:

```text
BEFORE: local_imu_prior_used=0, reason=imu_history_coverage,
        history_first=1786179479.5777223,
        history_last=1786179479.708226,
        delta_rotation=nan
AFTER:  local_imu_prior_used=1, reason=applied,
        history_first=1786179479.5134101,
        history_last=1786179479.708226,
        delta_rotation=0.0887317447 deg
```

This is direct startup/history-boundary evidence: the cloud and NDT source are
the same, while the local IMU prior availability differs.

The unique historical OOSM result mismatch was:

```text
ndt_stamp: 1786179480.559778
relative time: 0.9998314 s
BEFORE: NO_HISTORY, state_now=1786179480.6234019,
        rollback=nan, alignment=nan, replay=0
AFTER:  APPLIED, state_now=1786179480.6177673,
        rollback=1786179480.5577846, alignment=1.9934177 ms, replay=12
```

It occurs at the approximately one-second IMU initialization/history warm-up
boundary (about 200 samples at 200 Hz), not in the steady-state portion.

Under the controlled protocol, no such mismatch remains: BEFORE, RUN-A, and
RUN-B all have `APPLIED=1189`, `NO_HISTORY=11`, and zero `FUTURE`,
`ALIGNMENT`, or `REPLAY_INCOMPLETE` results.

## OOSM repeatability

Joining the controlled OOSM CSV files by exact `ndt_stamp` gives:

```text
RUN-A vs RUN-B:
  result mismatch count: 0
  rollback presence mismatch: 0
  rollback numeric max/P95/mean difference: 0 / 0 / 0 ms

BEFORE vs RUN-A:
  result mismatch count: 0
  rollback presence mismatch: 0
  rollback numeric max/P95/mean difference: 0 / 0 / 0 ms
```

The OOSM comparison is independent of the NDT cloud/geometry comparison.

## Corrected trajectory

Each controlled result bag contains 1189 `/dog_livo/odom_corrected` samples.
Joining exact `header.stamp` values gives:

```text
CLEANUP-3 BEFORE vs CLEANUP-4 RUN-A: 494 common samples
position mean/P95/max:       0 / 0 / 0 m
orientation mean/P95/max:    2.2601e-7 / 1.7075e-6 / 2.4148e-6 deg
velocity mean/P95/max:       0 / 0 / 0 m/s
last common-sample difference: 0 m, 0 deg, 0 m/s

CLEANUP-4 RUN-A vs RUN-B: 522 common samples
position mean/P95/max:       0 / 0 / 0 m
orientation mean/P95/max:    2.3870e-7 / 1.7075e-6 / 2.4148e-6 deg
velocity mean/P95/max:       0 / 0 / 0 m/s
last common-sample difference: 0 m, 0 deg, 0 m/s
```

The lower exact-timestamp intersection is due to callback-time header stamps;
the message counts are equal and the common samples remain identical. The
per-window maximum differences for BEFORE vs RUN-A were:

```text
0-30 s:    position 0 m, rotation 1.7075e-6 deg, velocity 0 m/s
30-60 s:   position 0 m, rotation 1.7075e-6 deg, velocity 0 m/s
60-90 s:   position 0 m, rotation 2.4148e-6 deg, velocity 0 m/s
90-120 s:  position 0 m, rotation 2.4148e-6 deg, velocity 0 m/s
```

There is no growing position or velocity difference and no accumulating
orientation drift. The tiny bounded orientation values are quaternion floating
point representation differences.

## Runtime sample (current HEAD A/B)

The current HEAD resource samples (from `resources_raw.csv`, restricted to the
two PIDs of each run) were:

```text
RUN-A NDT: RSS mean/peak 100391/100604 KiB, CPU mean/peak 13.01/14.8 %
RUN-A EKF: RSS mean/peak 14004/14632 KiB,  CPU mean/peak 4.98/6.0 %
RUN-B NDT: RSS mean/peak 100442/100580 KiB, CPU mean/peak 13.11/15.0 %
RUN-B EKF: RSS mean/peak 14075/14476 KiB,  CPU mean/peak 5.01/6.0 %
```

## Classification

```text
DELIVERY-CLEANUP-4-PASS
```

The NDT source blob is invariant, active NDT parameters are invariant, the
controlled cross-version delta is no larger than the same-version envelope,
the historical OOSM mismatch is localized to startup/history warm-up, the
controlled OOSM results are identical, and corrected trajectories show no
accumulating difference.

This stage does not enter CLEANUP-5.
