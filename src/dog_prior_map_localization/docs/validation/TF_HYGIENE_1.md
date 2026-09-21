# TF-HYGIENE-1 validation

Date: 2026-09-21  
Branch: `feature/visual-factor-window`  
Baseline before this stage: `5b50a651b818aee90ab6c2d7abe5ee5f5ac8063b`

## Scope

This stage addresses duplicate TF publication only. No NDT, EKF, OOSM, IMU,
vision, parameter, or measurement-output logic was changed.

## Diagnosis

The 60 s RViz run before the fix was recorded in:

`/home/jian/rosbag/loop2/tf_hygiene1_diagnostic_20260921/run3_rviz/`

The launch log contained 589 `TF_REPEATED_DATA` warnings. The recorded topic
counts were:

| Topic | Messages |
|---|---:|
| `/dog_livo/odom_high_rate` | 11800 |
| `/dog_livo/odom_corrected` | 589 |
| `/dog_livo/ndt_odom` | 600 |
| `/tf` | 12389 |

The identity `12389 = 11800 + 589` shows that the duplicate TF messages came
from the high-rate state stream plus the corrected/OOSM stream. The NDT node's
TF broadcaster is registered with ROS, but `output/ndt_publish_tf` is unset and
defaults to false, so it did not contribute TF messages. The input bag also
contains neither `/tf` nor `/tf_static`.

Classification: **TF-ROOT-A** (EKF corrected and high-rate streams are the
effective duplicate publishers).

## Minimal fix

`src/dog_prior_map_localization/src/ros_output.cpp` now sends TF only when
`publishState(..., corrected)` is false:

```cpp
if (publish_tf_ && !corrected)
```

The high-rate current-state stream remains the single TF authority. Corrected
odometry is still published on `/dog_livo/odom_corrected`; only its TF side
effect is suppressed. No timestamp de-duplication cache was added.

## Short before/after validation

The fixed run is:

`/home/jian/rosbag/loop2/tf_hygiene1_diagnostic_20260921/run4_fixed_rviz/`

| Check | Before | After |
|---|---:|---:|
| `TF_REPEATED_DATA` warnings | 589 | 0 |
| `/tf` messages | 12389 | 11800 |
| `/dog_livo/odom_high_rate` | 11800 | 11800 |
| `/dog_livo/odom_corrected` | 589 | 589 |
| `/dog_livo/ndt_odom` | 600 | 600 |

The after-run OOSM results are 589 `APPLIED` and 11 `NO_HISTORY`. The corrected
to next high-rate publication delay, measured by sensor timestamps, is:

- samples: 589
- mean: 4.895749 ms
- P95: 5.897570 ms
- maximum: 6.257534 ms
- minimum: 3.853559 ms

NDT determinism comparison between the independent before/after short runs
found zero mismatches for the 600 NDT rows and zero mismatches in the recorded
NDT odometry fields. This stage did not complete a full canonical 1090 s
regression after the fix; the user requested a focused warning fix rather than
another long replay. The source change is limited to TF publication gating.

## Parameters and static TF

- `output/publish_tf`: `true`
- `output/ndt_publish_tf`: unset; NDT default is `false`
- `/tf_static`: no publisher observed
- Input bag: no `/tf` or `/tf_static` topic

## Conclusion

The duplicate TF warning is removed in the focused RViz replay without changing
the NDT measurement or EKF/OOSM state path. This is accepted as the minimal
TF-hygiene fix for this stage. No Stage3B work was started.
