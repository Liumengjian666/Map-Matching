# PAPER-P6-ALG-INTEGRATION-A2A

## Scope

This release adds a causal real-measurement adapter around the existing
`FixedLagExperimentalController`.  It is an experimental 15D window boundary
only.  The formal FULL path, NDT implementation, IMU preintegrator, visual
frontend and frozen A1 Schur implementation were not rewritten.

## Implemented

- `FixedLagEventAdapter` owns one experimental fixed-lag controller, a causal
  IMU sample buffer, a monotonic internal observation-ID allocator and source
  deduplication records.
- Incoming IMU samples are strictly timestamp checked.  A new window state is
  predicted only with `preintegrateImu()` over samples already appended to the
  adapter.  A missing right boundary sample rejects the event rather than
  consuming future IMU data.
- Frozen LiDAR transactions are converted from `map_T_lidar` to
  `map_T_imu` with the fixed `T_imu_lidar` extrinsic.  NDT is not called.  A
  non-full-rank `LocalRisk` installs the existing residual-consistent basis
  callback; the callback recomputes the physical weak-direction mapping at the
  current window linearization state.
- Frozen visual events use the real reference/current timestamps and the
  existing sensor-quality gate.  They become a two-state
  `VisualRelativeMeasurement` factor; no position pseudo-measurement and no
  legacy EKF innovation noise are used.
- Source duplicates are rejected before a new internal observation ID is
  allocated.  Window IDs remain separate from raw transaction IDs and visual
  timestamp pairs.
- Initial prior input is explicitly 15D in the window order
  `[right rotation, position, velocity, gyro bias, accelerometer bias]`.
  There is no fabricated 23x23 IKFoM covariance mapping.

## Tests

`p6_alg_integration_a2a_adapter_test` drives the real adapter interface with a
deterministic event sequence.  It checks causal IMU boundaries, equal-time
LiDAR/visual ordering, visual two-endpoint connection, LiDAR basis callback,
factor counts, unified IDs, duplicate rejection, optimization feedback,
duration/node marginalization, and explicit NDT/visual skip reasons.

Release standalone build and all 7 CTest targets pass.  No Floor01,
Corridor01, rosbag, GT tuning, or formal FULL replay was run.

## Boundaries and limitations

- The adapter is synthetic-fixture validated in this release; no real bag was
  consumed.
- The 15D window remains an experimental state owner.  `setWindowPredictionSeed`
  and the 23x23 IKFoM posterior are intentionally not called.
- A 15D preintegrator covariance is physically PSD for a single interval.  The
  adapter records a `1e-9` solve-only numerical floor when the existing window
  factor requires SPD; this is not represented as a 23D covariance or a new
  noise model.
- Expired source keys are retained for replay protection; this source ledger is
  deliberately separate from the window's bounded active-ID set.

`READY_FOR_FORMAL_EXPERIMENT = NO`
