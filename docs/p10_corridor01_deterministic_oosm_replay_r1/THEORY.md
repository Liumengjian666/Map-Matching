# Fixed sensor-time OOSM release

This change controls when an already-computed NDT observation is admitted to
the existing EKF OOSM correction path. It does not change NDT, the EKF update,
state propagation, or the OOSM replay equations.

For an observation with sensor timestamp `t_ndt`, the frozen V2 deadline is

```text
t_deadline = t_ndt + 0.150 s
```

The callback inserts the observation into a bounded queue ordered by sensor
timestamp. It does not correct the filter on callback arrival. After each
causal IMU propagation and state-history snapshot, the scheduler releases an
observation only at the first IMU state timestamp `t_state >= t_deadline`.
That exact release timestamp is passed to the existing correction routine and
checked there; a mismatch is recorded as an error rather than silently using a
different replay endpoint.

If the NDT callback arrives at or after the deadline, it is marked
`LATE_NDT_FOR_FIXED_LAG` and is not applied. There is no fallback to the
asynchronous legacy path. The experiment mode is opt-in and disabled by
default, so the existing production behavior is unchanged when the parameter
is false.

## Why this addresses the prior discrepancy

The previous Control/Shadow comparison first diverged at an OOSM update where
the filter endpoints differed by 4.992 ms and replayed 22 versus 23 IMU
samples. V2 instead fixes the replay endpoint from sensor time. For the
historical observation at `1517157219.693280 s`, both runs release at
`1517157219.844208 s`, replay 31 IMU samples, and report `APPLIED`.

The Control and Shadow runs have the same 396 observation timestamps, release
nodes, OOSM outcomes, and published trajectory values. The existing OOSM
planner remains responsible for selecting/interpolating its rollback state
and replay sequence.
