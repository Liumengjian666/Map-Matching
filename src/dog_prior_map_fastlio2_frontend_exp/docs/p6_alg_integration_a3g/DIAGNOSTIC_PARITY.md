# Diagnostic-only changes and parity

No estimator model/threshold/weight/solver/backend-selection/acceptance/deskew/
initialization change. The new capture_marginalization_health flag enables only
attempt bookkeeping, existing QR trace storage and a failure-only capsule; the
legacy shadow block remains guarded by the original full diagnostics flag.

Light-mode failure handling serializes the last optimizer trace but does not
run directional FD or damping sweep. Covariance records serialize the existing
request result with no extra query or callback. Health inspects states and prior
only. No state mutation or retry was introduced.

Synthetic OFF/ON comparison: exact states/stamps, A/b, H/g caches, factor/ID
counts, revision, optimizer and marginalization outcome, and callback counts.
Both successful removal and synthetic later-attempt failure lifecycle are tested.

Real prefix parity: 298 A3F rows, source CSV SHA256
`3c5dfc8715c54164c51c595ef1925bfa333a352cc64c513284c0129c16e63a33`;
zero differences in every events field except excluded qr_marginalization_ms.
See `RUN_FULL_FIRST_FAILURE/FROZEN_PREFIX_PARITY.json`.

A fresh-context read-only review identified six plumbing/guard issues, all
addressed before real launch. It did not re-audit frozen mathematical core.
External cross-model review remains manual by the user after final handoff.
