# PAPER-P3-R10A ROS Interface Contract

Status: interface specification and protocol-policy tests only. Full runtime
integration has not started.

## Package and dependency direction

`dog_prior_map_interfaces` is a minimal MIT package containing ROS message
definitions and message-generation metadata. It may depend on standard message
packages. The MIT `dog_prior_map_localization` NDT package and GPL-2.0-only
`dog_prior_map_fastlio2_frontend_exp` package may both depend on it. The MIT
package must never depend on the GPL experimental package.

## Identifier ownership and representation

The protocol supports exactly one active experimental frontend per external
NDT server instance.

- `frontend_session_id`: canonical nonzero UUIDv4 string created by the
  frontend once per process start; immutable for that process lifetime. The
  server only echoes/validates it and uses it as part of cache keys.
- `server_instance_id`: canonical nonzero UUIDv4 string created by the NDT
  server once per process start. A new server process gets a new value. The
  frontend binds the value from the initial valid status and includes it in
  every request/control message; the server echoes it in results.
- `epoch`: owned by the frontend, starts at zero for a new session and
  increases monotonically for explicit filter reset or sensor-time rewind. The
  server echoes it and never increments it.
- `transaction_id`: owned by the frontend, starts at one and increases
  strictly within `(frontend_session_id, epoch)` without wrapping.

The immutable result/request key is
`(frontend_session_id, epoch, transaction_id)`. Timestamps are checked payload
fields, not transaction identity. IDs are distinct protocol entities; there is
no generic `session_id` field.

## Message: `NdtScanRequest`

The request is one atomic ROS message; cloud and predicted pose are not sent on
separate topics.

```text
std_msgs/Header header                 # stamp = scan_end; frame_id = map_frame
uint32 protocol_version                # exact negotiated version
string frontend_session_id
uint32 epoch
uint64 transaction_id
string server_instance_id              # exact bound server process
time scan_start
time scan_end
uint64 scan_start_ns
uint64 scan_end_ns
string map_frame
string lidar_frame
sensor_msgs/PointCloud2 cloud_end_frame
geometry_msgs/PoseStamped predicted_map_T_lidar
uint64 request_cloud_hash
```

Required invariants:

- `scan_start_ns < scan_end_ns`, using integer nanoseconds internally.
- `scan_end_ns` is the canonical transaction timestamp representation.
  `header.stamp`, `scan_end`, and `scan_end_ns` must represent exactly the
  same integer nanoseconds. Start mirrors must also agree.
- Integer nanoseconds exactly equal lossless conversion of the ROS `time`
  fields; they are canonical internally and are never reconstructed through
  `double`.
- `header.stamp`, `cloud_end_frame.header.stamp`, and
  `predicted_map_T_lidar.header.stamp` equal `scan_end`.
- `cloud_end_frame.header.frame_id == lidar_frame`; points are expressed in
  the LiDAR frame at scan end.
- `predicted_map_T_lidar.header.frame_id == map_frame`; child-frame semantics
  are exactly `lidar_frame`.
- The prediction is `map_T_lidar`, converted from the filter's
  `map_T_imu` using fixed calibrated `T_imu_lidar`.
- `request_cloud_hash` is a stable 64-bit FNV-1a hash over the canonical
  serialized cloud contract (header frame/stamp, dimensions, fields, point
  step, endianness, and data bytes). It is an identity/checking aid, not a
  cryptographic integrity claim.
- Hash equality alone is not payload identity. For a repeated key, compare all
  consumed request fields, including exact cloud bytes and predicted pose.
- Requests are not latched. No scan request is allowed before the ready and
  session-binding handshakes complete.

## Message: `NdtScanResult`

```text
std_msgs/Header header                 # stamp = scan_end; frame_id = map_frame
uint32 protocol_version                # exact negotiated version
string frontend_session_id
uint32 epoch
uint64 transaction_id
string server_instance_id
time scan_start
time scan_end
uint64 scan_start_ns
uint64 scan_end_ns
uint8 disposition
string reason
bool pose_valid
string map_frame
string lidar_frame
geometry_msgs/PoseStamped raw_map_T_lidar
geometry_msgs/PoseStamped used_map_T_lidar
float64 fitness
uint32 iterations
bool converged
uint64 request_cloud_hash
uint64 ndt_source_cloud_hash
bool translation_limited
bool rotation_limited
bool step_limited
```

`step_limited` is a derived compatibility/diagnostic field and must equal
`translation_limited || rotation_limited`; producers may not assign it an
independent meaning. All three fields participate in terminal identity and
must be checked by the frontend. The unit-test validator uses a unit-quaternion
norm tolerance of `1e-6`; success poses require finite position/quaternion
components, finite norm greater than `1e-12`, and `abs(norm - 1) <= 1e-6`.

Disposition constants are `SUCCESS=0`, `REJECT_INSUFFICIENT_POINTS=1`,
`REJECT_NOT_CONVERGED=2`, `REJECT_INVALID_SOURCE=3`,
`ERROR_NDT_INTERNAL=4`, `ERROR_PROTOCOL=5`, and
`ERROR_TERMINAL_CACHE_EXHAUSTED=6`, `ERROR_REQUEST_TIMESTAMP_INCONSISTENT=7`.

The timestamp error rejects a request before cache-miss processing or NDT.
The result's `header.stamp`, `scan_end`, and `scan_end_ns` must agree exactly;
the start mirrors must also agree.

Exactly one terminal result is emitted for every well-formed request accepted
for NDT processing. Success carries finite raw/used poses and `pose_valid=true`;
all other dispositions have `pose_valid=false`. The frontend measurement is
`used_map_T_lidar` after the existing NDT step limiter; raw pose is
diagnostic-only. The NDT optimizer and limiter calculations are not changed.

`REJECT_INSUFFICIENT_POINTS` and `REJECT_NOT_CONVERGED` mean prediction-only
commit. `REJECT_INVALID_SOURCE` is terminal and fatal: it covers structurally
invalid source data, non-finite point coordinates, or invalid preprocessing
output. `ERROR_NDT_INTERNAL`, `ERROR_PROTOCOL`, and
`ERROR_TERMINAL_CACHE_EXHAUSTED` also fail-stop. Only the first two reject
dispositions advance the predicted state. A cache-exhaustion response means
the request was not passed to NDT; the server fail-stops that session and does
not evict prior entries.

For a current-key `SUCCESS`, the frontend requires `pose_valid=true`, finite
raw and used translations, finite unit quaternions under the tolerance above,
and finite fitness. A zero, non-finite, or non-unit quaternion is
`FATAL_CURRENT_TRANSACTION_CORRUPTION`. Every non-success disposition
requires `pose_valid=false`; a true value is also
`FATAL_CURRENT_TRANSACTION_CORRUPTION`. In either case, the frontend discards
the candidate, performs neither an IKFoM update nor a prediction-only commit,
retains the last committed state, and stops advancing scans. The fatal latch is
sticky; only process restart can recover it. Only `REJECT_INSUFFICIENT_POINTS` and
`REJECT_NOT_CONVERGED` are normal prediction-only decisions.
`REJECT_INVALID_SOURCE`, all `ERROR_*`, and unknown dispositions are fatal.

## Message: `NdtServerStatus` and startup handshake

```text
std_msgs/Header header
uint32 protocol_version
bool server_ready
bool fatal_latched
uint8 fatal_reason
bool external_mode_enabled
uint8[32] map_sha256
uint8[32] canonical_ndt_config_sha256
string map_frame
string lidar_frame
string server_instance_id
string bound_frontend_session_id
uint32 epoch
uint64 terminal_cache_max_entries
```

`fatal_reason` is `FATAL_NONE`, `FATAL_QUEUE_OVERFLOW`, or
`FATAL_CACHE_EXHAUSTED`. A latched fatal status must set `server_ready=false`.
The first fatal reason is retained; later errors cannot overwrite it.

Status is latched. A frontend may leave `WAIT_SERVER` only when one status
simultaneously satisfies all of the following:

1. `server_ready == true`.
2. `external_mode_enabled == true`.
3. `protocol_version` equals the expected protocol version.
4. Both map and canonical NDT-config SHA-256 values equal the run manifest.
5. `server_instance_id` is a valid nonempty UUIDv4.
6. `terminal_cache_max_entries` equals the recorded run configuration and is
   greater than the formal input's maximum expected scan count.

Missing, not-ready, malformed, or mismatching status keeps the frontend in
`WAIT_SERVER` and logs the exact reason; it never authorizes a scan. At initial
startup, `bound_frontend_session_id` may be empty. After the server receives a
frontend session-start/reset control request, status must echo that exact
frontend ID and epoch before the frontend enters `READY`.

The frontend binds the server instance at the initial successful handshake.
While still in `WAIT_SERVER` and before any transaction, it may bind a later
valid server instance. After it leaves `WAIT_SERVER`, any server instance ID
change is `SERVER_RESTART_DETECTED` and fatal; an in-flight transaction may
never complete across server instances.

## Messages: `NdtSessionControl` and `NdtSessionControlAck`

```text
uint8 command                    # BEGIN_SESSION / BEGIN_EPOCH / END_SESSION
uint32 protocol_version
string server_instance_id
string frontend_session_id
uint32 epoch
uint8 reason
uint8[32] expected_map_sha256
uint8[32] expected_ndt_config_sha256
```

`BEGIN_SESSION` establishes a new frontend-owned UUID at epoch zero.
`BEGIN_EPOCH` requires the currently bound frontend ID and exactly
`active_epoch + 1`; a larger request epoch cannot trigger a transition.
`END_SESSION` requires the exact active session and epoch. Each control is
scoped to `server_instance_id` and carries expected map/config hashes. The
server validates readiness, protocol, server instance, map/config identity and
transition legality before changing binding. A successful control is echoed
in `NdtSessionControlAck` and latched status; the frontend sends no scan until
both match. A failed control changes no binding.

On `BEGIN_EPOCH`, the server clears the prior epoch's terminal cache, external
previous NDT pose, delta pose, limiter history and pending/in-flight queue
before binding the new epoch. `BEGIN_SESSION` replaces previous client state;
`END_SESSION` releases it. Ordinary requests never mutate active session or
epoch. Requests for another session or a non-active epoch are rejected before
cache lookup/NDT.

`NdtSessionControlAck` echoes command, protocol version, server instance,
frontend session and epoch; it carries an accepted/rejection code and reason.

## Message: `NdtScanAck`

```text
string frontend_session_id
uint32 epoch
uint64 transaction_id
string server_instance_id
uint64 scan_start_ns
uint64 scan_end_ns
uint64 request_cloud_hash
```

The frontend publishes ACK only after atomic state/time commit. The server
validates the full key, instance, exact stamps and cloud hash. ACK means the
frontend consumed/committed the terminal event. ACK is diagnostic and may
advance a `highest_committed_transaction_id` watermark; it is not a memory
reclamation signal and never evicts a cached result. A duplicate ACK is
idempotent; a conflicting ACK is a protocol error.

## Terminal-result cache and duplicate requests

The cache key is `(frontend_session_id, epoch, transaction_id)`. For a first
unseen key, the server checks capacity before invoking NDT, processes it once,
and stores the complete terminal result plus consumed request identity. An
identical repeat returns the same cached result without rerunning NDT, including
when ACK for that key has already arrived. A same-key request whose stamps,
cloud payload/hash, predicted pose, frames, server binding, or protocol fields
differ is `ERROR_PROTOCOL`; NDT is not rerun.

The server retains all terminal results for an active frontend session/epoch
until explicit epoch reset, explicit frontend-session replacement/end, or
server process restart. There is no LRU/early eviction. A run must configure
and record `terminal_cache_max_entries` before startup; it must be strictly
greater than the expected maximum number of processed scans in the formal
Floor01 input. The configured value is reported in latched status. If an
unseen key arrives when the cache is full, the server does not invoke NDT,
publishes `ERROR_TERMINAL_CACHE_EXHAUSTED`, and fail-stops that active session.
It latches `FATAL_CACHE_EXHAUSTED`, clears `server_ready`, and rejects further
scan requests and state-changing controls. The frontend treats this disposition
as fatal and discards the candidate. BEGIN_SESSION or BEGIN_EPOCH cannot clear
either fatal latch; only process restart creates a fresh state and a new server
instance ID. After a fatal latch, only status/diagnostic queries are allowed.

The request callback only performs basic envelope checks, appends to a bounded
FIFO, and signals one dedicated external-NDT worker. Only that worker may
touch the external NDT instance, previous/delta pose, limiter history, or
terminal cache. It consumes requests serially. If two identical requests are
queued before the first is processed, the first executes once and caches its
terminal result; the second finds the cache and republishes that exact result.
A same-key queued request with a different payload becomes a protocol error
after the first completes and never causes a second NDT invocation.

Before cache lookup or NDT, the worker applies this order:

1. Validate message structure, frames and canonical timestamp mirrors.
2. Validate `request.protocol_version` against the bound version.
3. Validate server instance and active frontend-session/epoch binding.
4. Construct the transaction key.
5. Check terminal cache and compare full request identity on a hit.
6. On a miss, check capacity and execute NDT exactly once.

Old-epoch requests after reset are rejected stale; future unbound epochs are
rejected. Neither path changes binding, cache or NDT process count. Server map
and NDT-config hashes, protocol version and frame contract are immutable for
one `server_instance_id`; a change forces `server_ready=false` or a new
instance ID and requires a fresh handshake.

## Canonical terminal-result identity and completed ledger

`TerminalResultIdentity` contains every result field that can alter protocol
or algorithm meaning: protocol version; server/frontend IDs; epoch and
transaction ID; ROS and integer start/end stamps; request and NDT-source cloud
hashes; disposition/reason; pose validity; map/LiDAR frames; raw and used
poses; convergence, iteration, fitness, translation/rotation limiter outcomes
and their derived `step_limited` field; plus result
header stamp/frame. Pose and fitness values use exact stored-value equality,
not a numeric tolerance. This is determinism identity, not a geometric
accuracy comparison.

The frontend retains each complete identity in a bounded
`completed_terminal_ledger` for the active `(frontend_session_id, epoch)`.
Capacity is fixed in the run manifest and must cover the formal Floor01 scan
upper bound. Exhaustion is `FATAL_LEDGER_EXHAUSTED`; the ledger never silently
evicts entries.

## Frontend result classification

Given current pending key `Kcur=(Scur,Ecur,Tcur)` and result key
`Kr=(Sr,Er,Tr)`, classify in this order:

1. Validate structural identity (valid UUIDs and nonzero transaction). A
   malformed identity is fatal protocol corruption.
2. Look up `Kr` in `completed_terminal_ledger` before stale ordering. An exact
   terminal-identity match is `DROP_DUPLICATE_RESULT`; a changed identity is
   `FATAL_NONDETERMINISTIC_SERVER_RESULT`.
3. `Sr != Scur`: `DROP_FOREIGN_OR_STALE_SESSION`, diagnostic only.
4. Same session and `Er < Ecur`: `DROP_STALE_EPOCH`, no state change.
5. Same session and `Er > Ecur`: `FATAL_FUTURE_EPOCH_RESULT`.
6. Same session/epoch and `Tr < Tcur`: `DROP_LATE_OR_DUPLICATE_RESULT`, no
   state change.
7. Same session/epoch and `Tr > Tcur`: `FATAL_UNSOLICITED_FUTURE_RESULT`.
8. Only when `Kr == Kcur`, validate protocol version, exact ROS/integer stamp
   consistency, request-cloud hash echo, frames, bound server instance and
   terminal payload. A mismatch is `FATAL_CURRENT_TRANSACTION_CORRUPTION`; the
   candidate is discarded while the last committed state is retained.

Thus a replay of completed transaction T while waiting for T+1 is compared to
T's saved identity before it can be classified as an old transaction.
Old-session/old-epoch/old-transaction results absent from the ledger remain
drops even if their payload belongs to another server instance. For the exact
current key, a server-instance mismatch is fatal.

The frontend consumes at most one exact-key terminal result while in
`WAIT_NDT`. If no terminal result arrives within 5.0 seconds measured by
`ros::WallTime`, it discards the candidate and enters `FATAL`; it does not
advance the next scan. A later result is classified by the same key rules and
cannot mutate the filter.

## Verification boundary

`dog_prior_map_interfaces` has a no-ROS-master protocol-policy test compiled
against actual generated ROS message classes. Its minimal FIFO/single-worker
server harness is only a test oracle: it does not instantiate the production
NDT object, ROS callbacks, frontend, or IKFoM filter. The separate IKFoM pose
API spike remains a generic compile/API check. Neither test implies complete
runtime integration or permits Floor01 rosbag execution.
