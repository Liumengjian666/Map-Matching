# PAPER-P3-R10A Transaction State Machine

Status: protocol specification only; runtime integration has not started.

## Identity ownership

- The experimental frontend creates a canonical, nonempty UUIDv4
  `frontend_session_id` once at process startup. It remains immutable for that
  process lifetime; a process restart creates a new ID.
- The frontend owns `epoch`, incrementing it for an explicit filter reset or
  sensor-time rewind. A new dataset run uses either a new frontend session or
  an explicitly incremented epoch.
- The frontend owns `transaction_id`, starting at 1 and increasing strictly
  within `(frontend_session_id, epoch)`; it must never wrap.
- The NDT server creates a canonical, nonempty UUIDv4 `server_instance_id`
  once per server process start. It echoes and validates the frontend ID; it
  never creates or replaces that ID or advances the epoch.
- Exactly one active frontend is supported per external NDT server instance.

The transaction key is
`K = (frontend_session_id, epoch, transaction_id)`. `scan_end_ns` is the
canonical timestamp representation; ROS time/header stamps are exact mirrors
validated before cache lookup or transaction consumption.

## State ownership and candidate isolation

Exactly one frontend scan worker owns and mutates the IKFoM filter and
committed sensor time. ROS callbacks only validate basic metadata, enqueue
messages, and signal the worker. The worker never holds the queue or
publication mutex while waiting for NDT. Separately, exactly one external-NDT
worker serially consumes its FIFO request queue. Its ROS request callback only
validates/enqueues; it never calls `ndt.align()` or mutates external previous
pose, limiter history, or terminal cache. The test harness models this
single-worker/cache ordering but is not runtime integration.

The frontend keeps two filter values during a transaction:

- `committed`: accepted state/covariance at the last committed scan end.
- `candidate`: a copy of `committed`, propagated through the current scan end.

The candidate is not visible as committed output until the transaction
terminates. On NDT success, generic IKFoM update runs on a shadow copy of the
candidate. A successful postcondition check atomically replaces `committed`.
On an ordinary NDT reject, the predicted candidate is committed without a
measurement update. On a fatal event, the candidate is discarded and the
worker stops; no later scan is processed.

## States and transitions

| Current state | Event / guard | Action | Next state |
|---|---|---|---|
| `WAIT_SERVER` | Latched status has `server_ready=true`, external mode enabled, expected protocol/map/config hashes, valid `server_instance_id`, and expected cache capacity | Bind that server instance; send no scan request yet | `WAIT_INIT` |
| `WAIT_SERVER` | Status absent, not ready, malformed, or any handshake field mismatches | Keep scans disabled and log the exact failed field; a later valid status may be evaluated | `WAIT_SERVER` |
| `WAIT_INIT` | Static IMU initialization passes and provenance-approved `initial_map_T_lidar` is available | Initialize official IKFoM state, fixed extrinsic, gravity, biases, covariance and time watermark; send `NdtSessionControl(BEGIN_SESSION)` with frontend-owned session ID and epoch 0; wait for accepted ACK and matching status echo | `WAIT_SESSION_BIND` |
| `WAIT_INIT` | IMU/static-init invalid or configured pose invalid | Do not publish a scan request or claim a valid map pose | `FATAL` |
| `WAIT_INIT` | Initial-pose provenance remains blocked | Do not publish a scan request; retain the explicit blocked gate | `WAIT_INIT` |
| `WAIT_SESSION_BIND` | Accepted control ACK and ready status echo the bound server instance, exact `frontend_session_id`, epoch and protocol version | Record the completed binding/reset handshake | `READY` |
| `WAIT_SESSION_BIND` | Status absent/not-ready or does not yet echo the requested binding | Keep scans disabled and wait; log mismatched field | `WAIT_SESSION_BIND` |
| `WAIT_SESSION_BIND` | Control ACK rejected or its server/session/epoch/version/hash identity differs | Do not send scans; preserve last binding and report the exact rejection | `FATAL` |
| Any state after leaving `WAIT_SERVER` | A different `server_instance_id` is observed | Discard any candidate; report `SERVER_RESTART_DETECTED` | `FATAL` |
| `READY` | Next scan passes identity, frame, timing, and strict-order checks; required IMU interval is available | Copy committed state/covariance, propagate candidate to exact scan end, construct scan-local poses and end-frame deskew cloud | `BUILD_CANDIDATE` |
| `BUILD_CANDIDATE` | Deskew and predicted pose pass finite/frame/stamp checks | Publish one atomic request carrying the bound `server_instance_id`; start fixed 5.0 s wall-time deadline | `WAIT_NDT` |
| `BUILD_CANDIDATE` | Invalid cloud, missing IMU coverage, or internal prediction failure | Discard candidate; stop processing | `FATAL` |
| `WAIT_NDT` | Current-key `SUCCESS`, bound server instance, finite pose and exact payload/frame/stamp/hash checks | Copy candidate to shadow filter; apply generic pose update and validate all postconditions | `COMMIT_CORRECTED` if valid; else `FATAL` |
| `WAIT_NDT` | Current-key `REJECT_INSUFFICIENT_POINTS` or `REJECT_NOT_CONVERGED`, with exact payload checks | Do not apply a measurement update; retain the fully predicted candidate through scan end | `COMMIT_PREDICTION_ONLY` |
| `WAIT_NDT` | 5.0 s wall-time deadline expires | Discard candidate; retain prior committed state; fail-stop. Any later result is classified by key and cannot mutate state | `FATAL` |
| `WAIT_NDT` | Current-key invalid source, NDT internal error, cache exhausted, current-result payload corruption (`FATAL_CURRENT_TRANSACTION_CORRUPTION`, including any non-`SUCCESS` result with `pose_valid=true`), or protocol error | Discard candidate; perform neither an IKFoM update nor prediction-only commit; retain last committed state and stop advancing scans | `FATAL` |
| `WAIT_NDT` | Result belongs to an old session, old epoch, or lower transaction ID | Diagnostic-only drop; do not mutate filter or transaction state | `WAIT_NDT` |
| `WAIT_NDT` | Result claims a future epoch or future transaction ID in the current session/epoch | Discard candidate; fail-stop as unsolicited future work | `FATAL` |
| `WAIT_NDT` | Result key equals current key but server instance differs from the bound instance | Discard candidate; report `FATAL_SERVER_INSTANCE_CHANGED` | `FATAL` |
| `COMMIT_CORRECTED` | Atomic commit completes | Publish corrected scan-end state; set committed time to this scan end; send matching `NdtScanAck` | `READY` |
| `COMMIT_PREDICTION_ONLY` | Atomic commit completes | Publish predicted-only scan-end state with explicit disposition; set committed time to this scan end; send matching `NdtScanAck` | `READY` |
| Any nonfatal state | Explicit reset or sensor-time rewind | Discard candidate; frontend increments epoch exactly once; send `BEGIN_EPOCH`; server clears old terminal cache, external previous/delta pose, limiter history and queued work; wait for matching ACK/status before accepting scans | `WAIT_SESSION_BIND` |
| Any state | Scan/result/IMU/control queue capacity exceeded, conflicting duplicate request, out-of-order scan, transaction counter exhaustion, or filter invariant failure | Latch the first fatal reason, clear transaction readiness, and do not silently drop/reorder or recover automatically | `FATAL` |
| `FATAL` | BEGIN_SESSION, BEGIN_EPOCH, request, result, or other state-changing event | Reject/log; do not mutate filter, binding, queues, cache, or fatal reason | `FATAL` |
| `FATAL` | Status/diagnostic query | Report `fatal_latched` and the retained first reason; no transaction state changes | `FATAL` |

Handshake mismatch is not permission to proceed and is not an automatic
algorithm failure: the frontend stays in `WAIT_SERVER` with a specific reason.
Once the frontend has advanced beyond that state, an instance-ID change is a
server restart and is fatal. Rebinding is only allowed while no transaction has
ever started in the current frontend session.

## Result classification order

For a current pending key `Kcur=(Scur,Ecur,Tcur)` and received result key
`Kr=(Sr,Er,Tr)`, apply exactly this order:

1. Validate structural identity (UUIDs and nonzero transaction).
2. Check the completed-terminal ledger. Exact terminal identity is a
   diagnostic-only duplicate drop; a different identity for the completed key
   is `FATAL_NONDETERMINISTIC_SERVER_RESULT`.
3. `Sr != Scur`: drop as foreign/stale session, diagnostic only.
4. `Sr == Scur && Er < Ecur`: drop as stale epoch.
5. `Sr == Scur && Er > Ecur`: fatal future-epoch result.
6. Same session/epoch and `Tr < Tcur`: drop as late transaction.
7. Same session/epoch and `Tr > Tcur`: fatal unsolicited future transaction.
8. Only when `Kr == Kcur`, validate protocol version, ROS/integer timestamp
   agreement, request-cloud hash echo, frames, terminal payload and bound
   server-instance ID. Any mismatch is fatal current-transaction corruption.

After a transaction commits, an identical repeated terminal result is dropped
without a second update or commit. The frontend retains the first terminal
identity for every committed key in the active epoch, before classifying lower
transaction IDs. A second payload for the same key that differs is
`FATAL_NONDETERMINISTIC_SERVER_RESULT`. The bounded ledger is sized from the
formal Floor01 run manifest; exhaustion is fatal, with no silent eviction.

## Transaction semantics

For scan `k`, `candidate_k^-` is predicted from `committed_(k-1)^+` using only
the sensor-time IMU interval through `scan_end_k`. NDT receives the deskewed
cloud and predicted `map_T_lidar` in one request. The frontend accepts at most
one pending transaction and does not propagate a later scan while in
`WAIT_NDT`.

An ordinary NDT reject is a completed observation decision, not a timeout: it
commits `candidate_k^-` and advances the committed watermark to `scan_end_k`.
This prevents replaying the same IMU interval for scan `k+1`.

A timeout is different. The old committed state is retained, the candidate is
discarded, and the pipeline fail-stops. It never advances to `k+1`. The timeout
uses `ros::WallTime` only; sensor timestamps and `/clock` do not affect it.

An IKFoM generic update is performed on a shadow filter constructed from the
candidate state/covariance. Its API returns `void`, so acceptance is determined
only by finite-state/covariance, symmetry, nonnegative diagonal tolerance,
fixed gravity norm, fixed extrinsic, and unchanged scan timestamp checks. If
any postcondition fails, the shadow is discarded and the entire pipeline
enters `FATAL`; partial state or covariance is never committed.

## NDT cache and ACK lifecycle

The NDT server retains each full terminal result for every processed key in the
active `(frontend_session_id, epoch)` until that epoch is explicitly reset, the
frontend session is explicitly ended/replaced, or the server process restarts.
ACK records commit diagnostics/watermark only. ACK never evicts a terminal
result. An identical duplicate request, including one arriving after ACK,
returns the original cached result without rerunning NDT. A conflicting
same-key payload is a protocol error.

`terminal_cache_max_entries` is configured and recorded before a run and must
exceed the maximum scan count expected for the formal Floor01 input. There is
no LRU eviction. If a new key arrives while the cache is full, the server does
not run NDT, emits `ERROR_TERMINAL_CACHE_EXHAUSTED`, and fail-stops that active
session. It reports `fatal_latched=true`,
`fatal_reason=FATAL_CACHE_EXHAUSTED`, and `server_ready=false`. The frontend
discards the candidate and enters sticky `FATAL_LEDGER_EXHAUSTED` if its
completed-result ledger cannot store a newly completed identity. Queue and
ledger/cache exhaustion cannot be cleared by BEGIN_SESSION or BEGIN_EPOCH;
only process restart can recover. The capacity value must
be exposed in server status and match the run manifest before initialization.

A successful result advances the external `previous_used_pose`; an ordinary
reject does not. Session/epoch reset clears external limiter state and the
ending epoch's cache and queued requests without modifying legacy mode state.
Server-side request processing checks structure/timestamps, negotiated
protocol version, server instance, then active session/epoch before constructing
the key and checking cache. Only a cache miss after these checks may invoke
NDT. Future epochs never auto-bind. Map/config/version/frame identity is
immutable for the lifetime of one server instance; an identity change makes
the server not ready or requires a new `server_instance_id`.

`FATAL` has no automatic recovery. Restarting the frontend creates a new
`frontend_session_id`; starting a new dataset in the same process uses a new
epoch only while not in `FATAL`. Once fatal is latched, BEGIN_SESSION,
BEGIN_EPOCH, requests, results, and other state-changing controls cannot
restore `READY`; only process restart can. A timestamp rewind during a nonfatal
run requires an explicit epoch-reset handshake before returning to `READY`.
