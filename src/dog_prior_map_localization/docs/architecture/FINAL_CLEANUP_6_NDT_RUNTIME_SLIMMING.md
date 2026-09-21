# FINAL-CLEANUP-6: NDT runtime slimming

## Scope

This stage removes the inactive NDT information-matrix, Schur-observability,
and local-degeneracy branches from the delivery executable.  The active NDT
path remains unchanged: Livox preprocessing, IMU/prediction timestamp
selection, NDT alignment, step limiting, pose/path/TF publication, and the
determinism fields needed to audit those operations.

The removed diagnostic branches no longer allocate their KD-tree or publish
`/dog_livo/lidar_degeneracy` and `/dog_livo/lidar_information`.  The canonical
YAML no longer carries parameters that the delivery executable does not read.
The intermediate voxelized map is now a local load-time object; only the NDT
target cloud is retained after initialization.

## Verification

Builds from the Stage B commit passed in Release mode with both research-tools
options:

```text
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=OFF --pkg dog_prior_map_localization
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=ON --pkg dog_prior_map_localization
OOSM_REPLAY_PLANNER_CONTRACT_PASS
```

The NDT source decreased from 2202 lines to 1502 lines.  No inactive
information/Schur/degeneracy symbols remain in the NDT source or canonical
configuration.

Two controlled 120 s replays used the same loop2 raw bag and the canonical
split launch:

```text
/home/jian/rosbag/loop2/final_cleanup6_repeat_a_20260922/
/home/jian/rosbag/loop2/final_cleanup6_repeat_b_20260922/
```

Each run produced 1200 NDT frames at approximately 10 Hz, with 1189 OOSM
`APPLIED` events and 11 `NO_HISTORY` events.  The two runs had zero mismatch
for cloud hash, filtered/source sizes, initial guess, raw NDT pose, final used
pose, fitness, iteration count, convergence, and step-limit flags.  Pose
quaternions were compared with sign-invariant absolute dot products; the
largest observed rotation difference was about `3.42e-6` degrees and the
largest translation difference was zero.

Compared with the pre-slimming Stage-A geometry audit, the first 1200 common
NDT timestamps also had zero mismatch for all active geometry fields and zero
fitness difference.  Corrected odometry had zero translation difference on
the common recorded samples; the largest sign-invariant rotation difference
was about `2.42e-6` degrees.  OOSM result and rollback stamps were identical.

The corrected resource sample is from:

```text
/home/jian/rosbag/loop2/final_cleanup6_resource2_20260922/resources.csv
```

| Process | RSS mean | RSS peak | CPU mean | CPU peak |
| --- | ---: | ---: | ---: | ---: |
| NDT | 93289 KiB | 93520 KiB | 15.30% | 16.5% |
| EKF | 14008 KiB | 14696 KiB | 5.20% | 5.9% |

Both replays exited with `rosbag play` return code 0, and no fatal/error,
exception, or `TF_REPEATED_DATA` messages were observed in the controlled
logs.

## Boundary

This is a delivery/runtime cleanup only.  It does not alter NDT mathematics,
EKF/OOSM behavior, IMU propagation, visual code, measurement weighting, or
directional fusion.  The user-owned dirty launch file remains untouched.
