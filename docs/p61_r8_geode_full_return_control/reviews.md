# Bounded pre-run review

Local fresh-context read-only review (two bounded passes) found and closed:

- Recorder prediction TCPROS callback may arrive later than the NDT diagnostics.
  Missing recorder delivery now waits to the existing deadline, not an immediate
  false production failure. No production prediction changes.
- An exception before cloud publication/terminal handling now retains an explicit
  current-frame STOP row. This is not a successful ACK or fabricated pose.
- Pin algorithm YAML and input manifest SHA literals, revalidate original assets,
  and check actual ROS process `/proc/PID/exe` against the frozen R7 binaries.

The reviewer confirmed production NDT odometry publishes only on `ok`; failure
does not publish an old pose. A finite recorded prediction can therefore permit
the original IMU-only continuation without waiting for nonexistent odometry.
High-rate watermark uses the original IMU ROS stamp, not a double reconstruction.

Three input/terminal test methods passed, including delayed prediction, rejected
fusion, fatal time/state, exact frozen boundary identities and actual input hash.
No ROS input or NDT align was used in these tests. No C++ source was modified.
Optional external second opinion offered; not run without a new explicit approval.
