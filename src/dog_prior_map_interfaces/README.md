# dog_prior_map_interfaces

Neutral MIT ROS message package for atomic prior-map NDT scan transactions.
It contains message definitions and catkin message-generation metadata only;
it contains no NDT, FAST-LIO, IKFoM, or localization implementation.

When `CATKIN_ENABLE_TESTING` is enabled, it builds an adversarial protocol
contract test against the generated ROS message classes. The no-ROS-master
test harness models policy, cache, session control, and a synthetic single FIFO
worker; it is not a production server, does not instantiate NDT, and does not
exercise a ROS runtime frontend or filter.

The protocol semantics are specified in
`dog_prior_map_localization/docs/P3_R10A_INTERFACE_CONTRACT.md` in the paper
workspace. Request/result timestamps carry both ROS `time` and exact integer
nanoseconds; consumers must verify they agree exactly. `scan_end_ns` is the
canonical transaction timestamp. Request/result `protocol_version` is a real
message field and must match the negotiated `NdtServerStatus` field.

`NdtScanResult` records translation and rotation limiter outcomes separately;
the retained `step_limited` field is derived as their logical OR. Successful
results require `pose_valid` plus finite translations and finite unit
quaternions; non-success results must have `pose_valid=false`.
`NdtServerStatus` reports a sticky fatal latch and reason. Queue or cache
exhaustion clears `server_ready`; BEGIN_SESSION/BEGIN_EPOCH cannot recover it.
Only a process restart creates a fresh server instance.
