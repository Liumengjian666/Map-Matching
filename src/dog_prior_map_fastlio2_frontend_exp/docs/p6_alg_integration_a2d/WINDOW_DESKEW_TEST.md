# Deskew regression

`window_deskew_math_test`: PASS in Release and Debug.

Six deterministic cases: stationary, constant velocity, constant angular
velocity, combined translation/rotation, nonzero IMU–LiDAR lever arm and
nonzero gyro/accelerometer biases. World points are observed at exact IMU knots
and half-knots (genuine timestamps of the synthetic observations).

The synthetic IMU inputs are constructed for the frozen midpoint-input /
left-orientation discrete model. They are not asserted to be an independent
high-order continuous-time integration benchmark.

Maximum scan-end coordinate error: **3.30093e-15 m**. Endpoint propagation and
standalone preintegration are identical in the tested semantic fields.
An independent reconstruction of the old processor's point transform, supplied
the same trajectory, agrees within 1e-12 m. The old end-to-end processor test is
also built and run; no old processor is called by V3.

`measurement_provenance_test` jointly optimizes an in-scan visual factor, then
re-queries scan start. The 15D start-state change is 2.04767 (mixed-coordinate
norm, **not a metric pose error**). Current-anchor and stale-anchor deskew differ;
the test fails if the cached start state is reused. A small-window lifecycle
also proves a marginalized start cannot be resurrected.

Tests use synthetic truth for construction only. No GT, map asset, bag, KLT or
PnP from a real dataset was consumed or regenerated.
