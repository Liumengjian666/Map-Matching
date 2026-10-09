# Bounded implementation review

Two fresh-context, read-only reviewers examined the startup protocol and its
new executable orchestration. No external model CLI was used; the user chose
the current independent review only. No reviewer ran dataset registration.

Actionable startup findings:

1. Missing complete moving IKFoM state does not block the explicitly allowed
   standalone scan-to-map interface. Reconciled against the real `align()`
   interface and old sensor-only map-normalizer source. Replaced the proposed
   all-NOT_RUN stop with the bounded diagnostic protocol, without inventing an
   official pose/time interpretation.
2. TX51 straddles the prior's five-second availability boundary. Fixed the
   **pre-run** protocol at TX52, not selected by GT or NDT outcome.

Actionable code findings, fixed before the first real NDT call:

- C++14 evaluation order could invalidate `reason.c_str()` while a registration
  call updates `reason`. Split call failure and owned exception construction.
- Preserve active transaction/arm/stage, completed nominal pose and source
  identity on exception; never call an exception a successful executed state.
- Move completed-frame count after checked output flush.
- Separate registration API attempts from actual full-align calls; canonical
  insufficient-source returns do not call PCL align.
- Verify the historical proper-extrinsic receipt SHA rather than merely read
  the matrix from a historical JSON.
- Check per-frame output writes and final receipt write. The Python launcher
  also preserves a process-exit receipt and never silently reruns.
- Final bounded recheck distinguished an already committed in-memory arm
  state from incomplete scientific archival acceptance. Failure receipts now
  track `current_state_committed` truthfully and mark `archival_acceptance=false`;
  receipt-write failures have a separate active stage.

Tests include causal future-IMU tampering, missing leading coverage, nonidentity
extrinsic lever arm, SO(3)/SE(3) round trip, streamed SHA256, and a synthetic
input-error run that makes **zero** NDT calls and preserves a failure receipt.
Existing NDT, source hash, Anchor and refinement tests remain unchanged.

Build environment issue: initial link failed at `libusb_set_option` because
the inherited MVS library path selected an incompatible library. Rebuilding
with `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu` passed without scientific source or
parameter changes. This occurred before dataset NDT execution.

Accepted scope limitation: source rotation compensation retains the same
small V2 midpoint integration contract in an independent input helper because
the old helper lives inside its standalone GICP executable. No V2 registration
or optimizer is copied into this benchmark; old source/archive remain intact.
Translation deskew and full IKFoM remain unimplemented here and explicitly
excluded from accuracy/online claims. This limitation is not hidden by the
diagnostic name or by NDT convergence.

One pre-launch interface repair: invoking the Python launcher via a relative
script path produced `Path.relative_to()` failure while constructing source
hash receipts. This happened before `RUN_STARTED`, any binary invocation, or
any dataset NDT call. Canonicalize file paths before deriving repository-relative
keys; added a relative/absolute invocation regression test. No scientific
parameters or inputs changed. The first preflight failure is preserved below.
