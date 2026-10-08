# Prospective input is not an initialized trajectory

This is input engineering, not an ambiguity-evidence experiment. Historical
R4/R5/R6/R7 results are unchanged. No historical v1 equivalence is asserted.

## Point time and reference frame

For raw point i in packet p, use
`t_i = packet_sensor_stamp_p + firing_offset_i`, not bag record time. The pinned
VLP16 single-return implementation uses 55.296 μs firing cycles and 2.304 μs
laser offsets. Its ROS XYZ output is still an uncompensated instantaneous sample.
Putting this time-indexed sample in the P7 input format does NOT deskew it.

With `T_IL = T_imu_lidar`, correct later deskew requires the causal trajectory
`T_MI(t)` and transforms samples into the final LiDAR frame:

`p_Lend = inverse(T_MI(t_end) T_IL) T_MI(t_i) T_IL p_Li`.

The existing `ScanEndProcessor` implements this through P7's pose sequence and
deskew geometry. It has not been run on real Corridor01 scans in this task,
because the initial state is not accepted. Synthetic geometry tests passing
does not establish real-data propagation or SAME_OBJECTIVE readiness.

## Why the initialization gate blocks

The current P7 `initializeStatic` estimates gyro bias as the initial sample mean,
sets velocity to zero and uses mean specific force to align map gravity. These
operations need a valid static assumption. On the first 200 raw Epson samples,
both existing variance guards fail; no new threshold was introduced.

The sensor-only historical initialization estimates the first scan's map pose
using 50 scans over five seconds. It does not specify the current-time velocity,
IMU bias/gravity state or their causal availability. The normalized map inherits
this first-scan coordinate reference; coordinate identity is not evidence that
the IMU was at rest. The first scan also precedes the first available IMU sample.

This does not prove that causal moving initialization is mathematically
impossible. It proves the existing static path cannot be admitted here, and the
available pose-only artifact is insufficient to supply the missing state. A
new, validated causal moving initialization contract is the required next step.
No alternative static window was selected after seeing these failures, no
velocity/bias was fabricated, and no historical v2 odometry was substituted.

## Future SAME_OBJECTIVE boundary

Only after initialization and scan-end propagation are accepted may source,
nominal LiDAR terminal, raw score, and frozen P9 curvature share an objective.
Actual code read: P7 reader/runner, IKFoM initialization/propagation, scan-window
and deskew implementations, current-frame NDT, R4 engine, P9 energy chart and
archived U_obs interface. R4's Floor01-specific target count (549606) must not
be reused for Corridor01. No change to those frozen implementations was made.

Current scope: raw timestamp/format/provenance preparation succeeded; baseline
state and source/T0/U_obs/W2 parity remain NOT_RUN. This is not a DUAL-U failure.
