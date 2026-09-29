# Asynchronous visual event test

`processVisualReferenceStamp(ref_ns)` creates an exact sensor-time node only
after a causal right IMU boundary exists. A missing boundary does not consume an
ID, so retry succeeds after the sample arrives. An existing node at the same
timestamp is reused. A reference older than the retained window is rejected as
marginalized; an absent reference earlier than the latest state is rejected
rather than inserted out of order.

The current image endpoint follows the same `ensureStateAt(cur_ns)` path. Only
after both endpoints exist is the cross-state visual factor added. Tests cover a
reference timestamp distinct from every LiDAR timestamp, separate ref/current
arrival, retry, and same-timestamp reuse.
