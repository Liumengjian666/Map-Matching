# Contract and review checks

- Real internal bag SHA matches the task's frozen identity.
- Time-uniform100 indices frozen before registration;200 exported PCD SHA checked.
- Target file index exactly source index minus1; no same-scan target contribution.
- Source max1400 unchanged; B/C counts exactly equal in100/100 pairs.
- Common Target hash equal across A/B/C in100/100 pairs.
- Common reference is immutable FULL raw result; each query sets its own Source
  pointer before using the existing PCL derivative backend.
- 300 align rows,600 derivative rows,100 per-arm common/own pose records.
- Gaussian support minimums: FULL74.738%, FoV71.942%, count73.435%; all exceed
  the frozen50% safeguard. No invalid/indefinite row was removed from inference.
- R2 first-pair Source/pose/Fitness/iterations/Hessian parity passed.
- NumPy independently verifies all600 matrix eigenpairs; see independent_math_checks.json.
- Nine Python tests: time sampling, epoch precision, bad times, insufficient sample
  guard, anisotropy-only negative label, native-run criterion, FoV-specific criterion,
  eigenvector sign invariance, resource-log parsing.
- One C++ test covers crop boundaries, deterministic count/order/range/finite guards
  and the exact R2 target-lifecycle grid-size distinction.
- Process resource log, loader failure, manifests and binary identities frozen.
- No production src/CMake/launch changes, no external disk access, no pseudo-GT,
  IMU, Deskew, EKF, Recovery or Coupled candidate calls.

## Five-axis self-review

Correctness: key hazards checked were wrong internal Source in native derivatives,
target-grid lifecycle divergence, comparing different poses, mean-vs-total score
normalization, and calling relative anisotropy physical failure. Actual1.0m grid
and the absence of severe labels were preserved rather than hidden.

Readability/architecture: small offline docs tools; production native objective
and chart reused unchanged. No second optimizer or runtime switch introduced.

Safety: only explicit internal paths, exclusive new input/output directories,
bounded100 pairs, no retries of any completed align. MVS libusb loader conflict
handled using an existing process-local workaround, not system modification.

Performance: one Target per pair, bounded source<=1400,300 total align calls;
diagnostic600 jets recorded separately. Process RSS is not deployment RSS.

Limits/counterexamples: all360 results are positive definite; translation blocks
are not near the severe label. FoV's extra loss is real in this objective but its
range/structure/time-sampling confounds prevent a pure-angle causal claim. There
is no reference motion and no proof of an incorrect local minimum. This review
was local; no independent agent or external CLI review is claimed.
