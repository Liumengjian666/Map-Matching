# P3-R9B Propagation Consistency Audit

## Scope

This is a diagnostic/engineering audit for the experimental Floor01 R9B profile. It does not claim improved localization accuracy, does not use GT, and does not change canonical configuration. `P4_ALLOWED = NO`.

## Measurement and state boundary

`ImuSample` and each `FilterStateSnapshot` store the raw `sensor_msgs/Imu` acceleration and gyro measurement at the snapshot's own timestamp `t_k`. They do not silently label a raw measurement as the interval `[t_{k-1}, t_k]`. The interval input is stored separately as `interval_acc_input`, `interval_gyro_input`, and `has_interval_input`.

For an accepted pair of samples at `t_k` and `t_{k+1}`, the experimental profile forms:

```text
a_interval = 0.5 * (a_k + a_{k+1})
w_interval = 0.5 * (w_k + w_{k+1})
```

The same `propagateImuKinematics` primitive advances the nominal `[p, v, R, ba, bg]` state for the full IMU interval, OOSM replay interval, and deskew's partial interval. OOSM restores the snapshot timestamp as the replay cursor and restores raw/interval measurement fields before replaying the original timestamped IMU samples.

## Scan-boundary rule

Deskew uses head/tail midpoint input for a point-time partial interval only if the tail IMU sample is no later than the admitted scan-end horizon. For the final fragment where the next sample would be after `scan_end`, it uses the head sample. No post-scan-end IMU, later scan, NDT result, or GT is consumed to deskew the current scan. This is scan-complete interval integration, not use of data beyond the scan's temporal boundary.

## Invalid-interval recovery

In the experimental midpoint mode, a non-positive IMU `dt` is ignored without moving the accepted timestamp/sample cursor backward. If `dt > imu/max_dt` (default `0.05 s`), that interval is not propagated: the timestamp and raw measurement cursor are rebased to the current sample, and any stored midpoint input is cleared so the next valid interval cannot average across the dropped gap. State propagation therefore has a temporal hole; scan coverage validation sees the enlarged gap between saved propagated states and rejects a scan that spans it when it exceeds `deskew/max_imu_gap_sec`. Subsequent valid intervals can then resume from the rebased raw sample. This recovery branch was added after the two full replays; it is not exercised by their input: raw `/input/imu` has minimum adjacent `dt = 0.004911899567 s`, maximum `dt = 0.039999961853 s`, zero non-positive intervals, and zero intervals above the configured `0.05 s` propagation limit. The one interval above the 20 ms deskew coverage limit is explicitly rejected in both runs.

## Covariance prediction

The EKF nominal input and the acceleration used in the transition matrix `F` are both derived from the same interval acceleration average; `F` is formed using the post-rotation `R_` produced by that interval's gyro average. The existing diagonal process-noise model `Q` and its `dt` scaling are unchanged. This stage checks semantic consistency of the nominal input and first-order `F` linearization; it does **not** re-derive or claim a newly calibrated continuous-time covariance model. That remains an explicit limitation.

## Compatibility boundary

The profile flag `imu/midpoint_interval_input_enable` defaults false in existing configs. Only `config/superloc_floor01_imu_deskew_r9b.yaml` enables it. Default/legacy propagation continues selecting the tail sample. R9B changes are isolated to the experimental launch/profile; no canonical YAML is changed.

## Evidence status

- Release build: PASS (see R9B run protocol and final run artifacts).
- OOSM replay contract: PASS.
- Deskew interval/no-post-scan-end contract: PASS.
- P/F nominal-input alignment: PASS at first-order implementation level, with unchanged approximate `Q` caveat above.
- Full Floor01 Run A/B: PASS for the specified no-GT scan/NDT determinism gate: 4,126 common frames; zero timestamp, cloud-hash, disposition, point-count, raw/final pose, fitness, iteration, convergence, or limiter mismatch. OOSM results and rollback stamps also agree. Replay sample counts vary with ROS callback scheduling (maximum absolute count delta: four IMU samples); this is recorded separately and is not an NDT branch divergence.
- Each full run rejected the same single in-sequence scan because the source IMU interval is 40 ms while deskew allows at most a 20 ms coverage gap. This is an explicit coverage rejection, not silently extrapolated data.
