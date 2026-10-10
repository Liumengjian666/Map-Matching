# P10 Corridor01 Coupled Causal Comparison R1

## Frozen algorithm

The experiment reuses the R6 local regularized refinement without changing its
math, bounds, trigger, or acceptance rules:

```text
eta = W*u + S*v
H = J^T * H_native * J + sum_a(g_native[a] * K_a)
(Hvv + lambda*I) * delta_v = -(g_v + Hvu * delta_u)
```

The ROS node supplies the current nominal NDT result, the causal EKF
`map_T_lidar` prediction, the current source cloud, and an `ObservablePclNdt`
backend. The backend's `nativeJet()` and `dynamicScore()` use the same inherited
PCL NDT target grid and objective instance that just performed nominal
`align()`; the refinement does not create a second NDT object, map, source, or
full alignment.

The nominal path remains:

```text
PCL raw NDT pose -> existing limitNdtStep -> used LiDAR pose -> EKF OOSM
```

For feedback modes, only a recommended R6 candidate replaces the raw pose
before the same `limitNdtStep()` call. The resulting LiDAR measurement is
published through the existing ROS topic and passed to the unmodified EKF
OOSM update. The next frame's nominal prediction therefore follows the state
actually updated by that run. `COUPLED_SHADOW` never substitutes the candidate.

## Causal ROS anchor

The EKF high-rate publisher converts its internal IMU state through the frozen
`T_imu_lidar` extrinsic and publishes a `map_T_lidar` pose with the dataset's
configured map and LiDAR frame IDs. Its `nav_msgs/Odometry` twist is expressed
at the LiDAR child frame: `ros_output.cpp` transforms the IMU-origin velocity
with the extrinsic lever-arm term and rotates both linear and angular velocity
into LiDAR axes. The NDT node accepts only matching frame IDs and finite rigid
poses/twists. It chooses the latest high-rate pose at or before the scan
reference, then advances that sample to the exact scan-reference stamp with
the sample's body twist using a right-multiplied constant-twist SE(3)
exponential. This causal extrapolation is bounded by the existing 20 ms OOSM
alignment allowance; the node waits for the high-rate topic watermark to reach
the scan reference before processing. The actual sample stamp, target stamp,
age, and prediction status are logged separately. No future-stamped pose or GT
is used. If the sample is stale, malformed, or cannot be advanced, refinement
is skipped and the nominal measurement remains in force.

The existing R5/R6 anchor helpers propagate and expire the anchor. Ordinary
untriggered frames may establish it from the current causal EKF prediction;
recommended feedback consumes it. Missing/stale predictions retain nominal.
This is explicitly an ROS EKF-derived causal anchor, not a claim that R6's
IKFoM shadow input can be reused unchanged.

## Modes

- `CONTROL`: default; no EKF-pose subscription and no coupled refinement.
- `COUPLED_SHADOW`: runs refinement and logs it, but publishes the unchanged
  nominal NDT pose.
- `WEAK_ONLY_FEEDBACK`: feeds back only the accepted R6 weak correction.
- `COUPLED_FEEDBACK`: feeds back the R6 weak correction and any accepted
  displaced-point strong conditional correction.

The launch default is `CONTROL`. The mode is an experiment-only parameter; no
production IKFoM/EKF update implementation or noise setting is changed.
