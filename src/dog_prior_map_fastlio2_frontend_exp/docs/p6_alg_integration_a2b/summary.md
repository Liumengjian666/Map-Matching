# PAPER-P6-ALG-INTEGRATION-A2B

## Outcome

All requested adapter-correctness P0 items are closed in the synthetic fixed-lag
test boundary. Runtime finite differences were removed from LiDAR-basis
construction, visual endpoints are created at their sensor timestamps, state plus
IMU-factor insertion is atomic, and visual translation is admitted only through
the causal LiDAR weak-direction policy.

The formal FULL path remains unchanged and isolated. No Floor01, Corridor01,
rosbag, GT tuning, or public-data integration was run. These modules therefore
remain synthetic-only and are not ready for a formal experiment.

## Key results

- Release standalone build: PASS.
- Release CTest: 8/8 PASS (the original seven targets plus A2B).
- Debug Eigen A2B target: PASS.
- Six-axis analytic-vs-central-FD Jacobian: PASS, including nonzero pose and
  extrinsic, multiple weak directions, near-zero rotation, and explicit near-pi
  rejection.
- Async visual reference/current insertion and same-timestamp reuse: PASS.
- Atomic initialization retry and state/IMU rollback: PASS.
- Directional visual projection and pure-rotation no-complement behavior: PASS.
- 10,000-event lifecycle: PASS; peak IMU buffer 301, peak active source records
  8, expired records removed 192, final IMU buffer 300, final active source
  records 6.
- Existing Schur information-conservation regression: PASS.

## Safety boundary

The experimental window does not feed the formal EKF, so it adds no duplicate
information to the frozen formal estimator. A visual factor is added only after
sensor-quality acceptance and a causal, non-stale LiDAR-risk decision. Normal
LiDAR and pure-rotation weakness do not trigger translation fusion. No-map-support
use is explicitly labelled `RELATIVE_ONLY_NO_GLOBAL_RECOVERY`.

`READY_FOR_FORMAL_EXPERIMENT = NO`
