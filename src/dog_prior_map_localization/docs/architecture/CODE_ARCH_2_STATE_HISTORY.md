# CODE-ARCH-2: ROS-free EKF state history

## Scope

This stage extracts the storage and lookup policy for `FilterStateSnapshot`
from `DogPriorMapEkfNode` into the small ROS-free `StateHistory` class.  It
does not extract, or change, OOSM orchestration.

## Ownership before and after

Before this stage, `DogPriorMapEkfNode` owned a
`std::deque<FilterStateSnapshot>` and also implemented insertion, pruning,
lookup, and truncation directly in `imu_processor.cpp`.  The NDT callback in
`vision_observation.cpp` additionally indexed and copied that deque.

After this stage, the node still owns the history object, while
`StateHistory` owns only snapshot copies:

```text
DogPriorMapEkfNode
  nominal state: p, v, R, ba, bg, P
  StateHistory state_history_
    FilterStateSnapshot copies
```

`restoreStateSnapshot()` remains on the node because it mutates the nominal
EKF state.

## StateHistory responsibilities

`StateHistory` provides only:

- monotonic insertion with duplicate-timestamp replacement;
- time-based pruning;
- reverse linear lookup at-or-before a target timestamp;
- removal of snapshots after a timestamp;
- indexed const access, size, empty, and value-copy semantics.

It has no ROS, message, PCL, OpenCV, NodeHandle, publisher, subscriber, NDT,
visual, EKF-update, or OOSM-result knowledge.

## Preserved boundary semantics

The extraction preserves the original numerical boundaries exactly:

- non-finite snapshot timestamps are ignored;
- an insertion older than `back().stamp - 1e-9` is rejected;
- timestamps within `1e-9` replace the existing tail snapshot;
- pruning uses strict `current_stamp - front.stamp > keep_sec`;
- lookup scans backwards and accepts `snapshot.stamp <= target + 1e-9`;
- alignment is `max(0.0, target - snapshot.stamp)`;
- truncation removes while `back.stamp > stamp + 1e-9`.

The old node methods remain as thin guards/delegation wrappers so the OOSM
callback and its execution order are unchanged.

## Why the full OOSM manager is not extracted here

Rollback, NDT correction, velocity feedback, replay sample selection, IMU
replay, covariance handling, and recovery are estimator orchestration.  They
remain in `DogPriorMapEkfNode` for this stage.  A later OOSM-manager stage can
use this boundary only after the current behavior is revalidated.

## Validation

The stage requires a deterministic standalone container contract test covering
monotonic insertion, duplicate replacement, out-of-order rejection,
at-or-before lookup, truncation, and copy/restore.  It also requires Release
build and the existing short deterministic replay to show no NDT, corrected,
OOSM, or rollback/replay-lineage differences.
