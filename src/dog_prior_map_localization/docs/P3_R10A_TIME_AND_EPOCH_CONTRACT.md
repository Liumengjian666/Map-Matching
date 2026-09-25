# PAPER-P3-R10A Time and Epoch Contract

Status: protocol specification; no runtime time handling has been implemented.

## Identifier ownership

The frontend creates one canonical nonempty UUIDv4 `frontend_session_id` at
process start. It remains fixed for that process lifetime; a restarted frontend
creates a new one. The NDT server independently creates a canonical nonempty
UUIDv4 `server_instance_id` at server-process start and reports it in its
latched status. A server restart therefore has a new instance ID and cannot
silently continue an in-flight transaction.

The frontend owns `epoch`, initially zero. It increments it exactly once for an
explicit filter reset or detected LiDAR/IMU sensor-time rewind. A new dataset
run must use either a new frontend process/session ID or an explicit epoch
increment. The server echoes/validates the frontend's epoch; it never creates,
increments, or substitutes one.

The frontend owns `transaction_id`, starting at one and strictly increasing
within `(frontend_session_id, epoch)`. It must not wrap. The full transaction
key is `(frontend_session_id, epoch, transaction_id)`. Sensor stamps are
validated payload fields, not identity keys.

## Integer sensor time and duplicate scan policy

All ordering, identity and equality checks use exact integer nanoseconds from
ROS `time` (`sec * 1,000,000,000 + nsec`). `double` seconds may be used for
numerical integration after an interval has been selected, but never as a key
or equality comparison. `scan_end_ns` is the canonical transaction timestamp.
A request/result repeats `scan_start` and `scan_end`; `header.stamp` is exactly
`scan_end`. For both message types, header stamp, ROS `scan_end`, and
`scan_end_ns` must match exactly; start-time mirrors must match too. The server
rejects an inconsistent request before cache lookup/NDT, and the frontend
fail-stops on an inconsistent current-key result.

Within one `(frontend_session_id, epoch)`, accepted scan ends must strictly
increase. For a repeated `scan_end`:

- If scan start, frame IDs, canonical cloud payload/hash, and all consumed
  metadata match, classify it as `DROP_DUPLICATE`; do not allocate a new
  transaction or run NDT.
- If the stamp matches but any consumed input differs, classify it as
  `FATAL_PROTOCOL_ERROR`.
- A scan end earlier than the accepted watermark is not sorted/replayed
  silently. A detected rewind begins a new frontend-owned epoch; otherwise it
  is `FATAL_OUT_OF_ORDER`.

## IMU ordering and equality

The common time axis is:

```text
t_imu_in_lidar_clock_ns = t_imu_raw_ns + imu_to_lidar_offset_ns
```

Positive offset means add to the raw IMU timestamp. The offset is applied once
at ingestion and thereafter all scan coverage/interpolation is performed on
the common integer-nanosecond axis. Wall time must not participate in sensor
synchronization.

Within an epoch, IMU stamps must increase. For equal stamps, identical consumed
fields (frame ID, angular velocity, and linear acceleration) are deterministically
collapsed with a diagnostic. Conflicting consumed values at one stamp are a
fatal protocol error. A backward IMU stamp invokes the fixed rewind policy
below; it is never sorted into the current epoch.

Current Floor01 audits establish monotonic IMU and LiDAR streams and overlap,
but the reviewed records do not establish a physical IMU-to-LiDAR clock offset
of exactly zero. Therefore `imu_to_lidar_offset_ns=0` is **not yet approved**;
the runtime gate stays blocked until an existing authoritative sync record or
a separately authorized timestamp-offset audit closes this value. No fitted
offset and no GT-derived offset may be used.

## Session bind, reset, and server restart

At startup, the frontend remains in `WAIT_SERVER` until the latched status is
ready and matches expected protocol version, external mode, map/config hashes,
server instance ID, and cache capacity. It then binds that server instance.
After provenance-approved initialization, it sends
`NdtSessionControl(BEGIN_SESSION)` with its own session ID and epoch zero,
expected map/config hashes and bound server ID. It accepts no scans until the
control ACK is accepted and ready status echoes the same server instance,
frontend session and epoch.

For an explicit reset or sensor-time rewind, the serialized worker handles the
control event before newly queued sensor data: it discards the active
candidate, increments the frontend-owned epoch, flushes queues, resets filter
initialization and transaction-local watermarks, and sends
`NdtSessionControl(BEGIN_EPOCH)` for exactly the next epoch. The NDT server
clears the ending epoch's terminal cache, external previous/delta pose,
limiter history and pending queue, then ACKs and republishes matching ready
status. No scan in the new epoch is accepted before that handshake completes.
`END_SESSION` explicitly releases the bound session. Ordinary requests never
bind a session or advance an epoch.

Before any transaction has started and while still in `WAIT_SERVER`, the
frontend may bind a later valid server instance. After it leaves `WAIT_SERVER`,
any instance-ID change is `SERVER_RESTART_DETECTED` and fatal, including while
waiting for session binding or initialization. A current transaction may never
complete across server instances. A result from an old session/epoch is
diagnostic-only and dropped; a result for the current key with another server
instance is fatal.

A late result from a prior epoch is dropped. A clock rewind is not a valid
propagation interval. A timeout or other fatal error remains fail-stop and is
not converted into an automatic epoch reset. Bag replay/loop must use a fresh
frontend session or explicit reset event; implicit loop continuation is
forbidden.

## Wall-time deadline and queue overflow

The fixed result watchdog is 5.0 seconds measured with `ros::WallTime`. It is
only a fault detector. On expiry, the scan candidate is discarded, the
previous committed state is retained, and the frontend enters `FATAL`; it does
not process the next scan. `/clock` pause or rewind cannot extend or complete
this deadline.

All scan, IMU, result and control queues have explicit bounded capacities
recorded in the run configuration. Capacity exhaustion is fatal; callbacks do
not silently drop oldest data or reorder. On overflow, the affected frontend
or server enters sticky `FATAL`, stops accepting scan transactions, and no
longer advertises transaction readiness (`server_ready=false` for the server).
BEGIN_SESSION, BEGIN_EPOCH, ACK, and ordinary requests cannot clear or replace
the first fatal reason. Only process restart creates a fresh state and, for a
server, a new `server_instance_id`. The limits must be fixed before a run and
included in the effective-configuration hash. The NDT terminal cache
is separately bounded by `terminal_cache_max_entries`, configured before the
run to exceed the formal input's expected maximum scan count, and retains
results until the active epoch/session ends or the server restarts. ACK does
not free cache entries. Exhaustion fail-stops the active session; there is no
LRU eviction.
