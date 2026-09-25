# PAPER-P3-R10A Runtime-1B: wired FAST-LIO2 frontend

This opt-in executable wires the pinned IKFoM frontend to the existing prior-
map NDT transaction topics. It does not replace the legacy localization nodes.

## ROS path

- Inputs: `sensor_msgs/Imu` and a timed `sensor_msgs/PointCloud2` with `x`, `y`,
  `z`, and configured point-time fields (default `time`, seconds from scan
  start).
- Callbacks enqueue only. One worker owns the filter, scan transaction and
  committed sensor-time watermark.
- The worker waits for an IMU watermark at/after scan end, then gives the scan
  processor only samples at or before scan end. When the last legal IMU sample
  precedes scan end, the existing held-input tail path reaches the exact end.
- The atomic NDT request carries the end-frame cloud and predicted
  `map_T_lidar` with matching end stamps.
- NDT success updates a shadow IKFoM candidate from `used_map_T_lidar`; the two
  ordinary reject dispositions commit prediction only. Other terminal errors
  fail-stop without committing the candidate.
- Corrected scan-end odometry is published on the configured odometry topic.

## Start-up gate

Load `launch/fastlio2_frontend_runtime.launch` only after the external NDT
server is configured for the same frame names, map, and NDT parameters. Before
running a dataset, fill the template's map/config SHA-256 values from the
server status and add a provenance-approved explicit `initial_map_T_lidar`.
The template intentionally leaves the initial pose absent and hashes empty;
the node stays blocked (`WAIT_SERVER`/`WAIT_INIT`) until those inputs are
provided. It never substitutes identity or a GT pose.

The point-time field must be a numeric `PointCloud2` field, little-endian, and
relative to the message header's scan-start time. For a nanosecond-valued field
use `point_time_scale_to_seconds: 1.0e-9`. Raw Livox `CustomMsg` is not accepted
directly by this frontend; convert it losslessly to the timed PointCloud2
contract first.

## Contract tests

`frontend_runtime_end_to_end_test` drives one actual runtime coordinator through
SUCCESS, insufficient-points reject, and not-converged reject, then verifies
scan 3 starts from scan 2's committed prediction watermark. The separate
`external_ndt_server_integration_test` starts the real NDT transaction server
against a synthetic PCD and exercises BEGIN_SESSION, request/result, the
predicted initial guess, SUCCESS and insufficient-points reject. PCL's
`hasConverged()` outcome for a third synthetic non-overlap cloud is not forced
by the test; the frontend's not-converged terminal path is covered by the
end-to-end coordinator test.
