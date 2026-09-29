# Event timestamp contract

## Causal IMU

`appendImu()` accepts finite samples with strictly increasing sensor stamps.
For a new window node at `tj`, the adapter requires the latest appended sample
to satisfy `stamp >= tj`.  `preintegrateImu()` performs only interpolation
between samples already received by the adapter.  If the right boundary is not
available, the event is rejected with `imu_history_not_causal_for_event`.

The predicted state uses the existing preintegrator and the 15D equations:

```text
Rj = Ri DeltaR
vj = vi + g dt + Ri DeltaV
pj = pi + vi dt + 1/2 g dt^2 + Ri DeltaP
bgj = bgi, baj = bai
```

## LiDAR

The LiDAR factor stamp is the frozen NDT transaction sensor stamp.  NDT
terminal status and map-support validity are checked before factor creation;
non-converged or unsupported events are recorded as skip reasons and do not
enter the objective.

## Visual

The event must satisfy `ref_ns < cur_ns` and `depth_ns <= ref_ns`.  The quality
gate receives these exact sensor stamps and rejects stale pairs.  Both endpoint
states must be present in the same window.  The visual residual is a factor
between the reference and current states; it is never rewritten onto a nearby
scan timestamp.

## Equal timestamps and replay

Events are processed in caller order when timestamps are equal.  A source key
is `(kind, raw transaction id)` for LiDAR, `(kind, ref_ns, cur_ns)` for visual,
and `(kind, start_ns, end_ns)` for IMU factors.  Replaying a source key cannot
allocate another window ID, including after marginalization.
