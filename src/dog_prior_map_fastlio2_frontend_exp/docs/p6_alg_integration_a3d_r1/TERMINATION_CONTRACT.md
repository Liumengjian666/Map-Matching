# A3D-R1 optimizer termination contract

This is a control-flow/status repair, not an estimator or acceptance redesign.
The production solver, frozen LiDAR surrogate, strict finite
`candidate_cost < current_cost`, damping updates, eight-iteration budget,
maximum step, 1e-9 gradient tolerance and existing 1e-8 step termination
threshold remain unchanged.

After strict candidate rejection, restore the entire iteration state backup.
Only a finite candidate cost, a positive raw solver norm below 1e-8, and no
policy clipping establish natural small-step convergence of this restored
state. No rejected candidate is committed. Successful revision/feedback
certification follows the existing convergence path. If earlier iterations
accepted a step, the existing overall ACCEPTED_UPDATE precedence is retained.

Clipping a nonzero raw step to zero or a tiny applied norm does not certify
convergence. NaN/+Inf/-Inf objective values do not certify convergence. A
zero raw solver step also does not use this new branch. Strict equality at
the existing 1e-8 threshold is not below it.

The old applied-step break condition is retained. The raw norm/clipping
semantics are now independent of whether trace capture is enabled.
Trace adds termination reason, small-step flag and rollback-state difference.
Summary adds matching termination observability; no new measurement is added.

## Regression evidence

Before changing production, the deterministic conflicting-observation
fixture had current/candidate cost 200, raw/applied step
3.3333302758042944e-9, clipping false, acceptance false. The new test failed
on old production with FAILED_ALL_CANDIDATES, isolating the status gap.

With the repair it returns CONVERGED_WITHOUT_STEP, feedback ready, and exact
zero state difference. Trace OFF/ON matches. Clipping to 0 and 1e-10 still
returns FAILED_ALL_CANDIDATES with no feedback or state mutation, including
OFF/ON parity. The existing A3B-R1 forced-zero optimizer regression is
retained and passes unchanged.

The NaN/+Inf/-Inf negative controls call the exact production classification
predicate, not an artificial optimizer objective-injection hook. This is
predicate-level coverage; it is not represented as a full nonfinite-candidate
optimizer transaction. Production calls that guard only after restoring state.

Independent read-only review identified stale diagnostic reason and live
replay guard fail-open/cleanup/identity-check issues. These were corrected
before the only real launch. The guard rejects partial/missing identity,
uses relative-only identity tolerance (not optimizer acceptance tolerance),
checks candidate/applied/raw facts, and has process-group cancellation with
bounded cleanup. Its self-test passes. External cross-model review remains
manual after the final report, per the user's instruction.

READY_FOR_FORMAL_EXPERIMENT = NO
