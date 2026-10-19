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

Post-run analysis review (one bounded pass) found three required issues, fixed
before the archived metrics were generated: ordinal NDT/deskew joins now use
measurement time with unique same-frame evidence; missing fusion no longer shifts
timing rows via zip; missing STOP-frame correction values render as gaps rather
than crashing the plotter. No experiment rerun resulted from these changes.
The actual2441 APPLIED rows pass the independent raw-archive event-order/hash
verifier. This verifier additionally checks per-frame state revisions, prior
correction counts and real corrected replay times, not just aggregate counts.

The final post-run CLI review was separately selected and explicitly confirmed.
It uses one immutable bounded archive and an immutable prompt, ephemeral/read-only
mode and existing authentication. It is a post-run limited code/result consistency
review, not an independent certification of all original raw measurements.
No external review was a pre-run gate for this run. See CLI_REVIEW.md for receipt.
