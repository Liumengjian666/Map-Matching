# Verification and review

## Executed

- Whole internal bag SHA256 recomputed and matched frozen R2 hash.
- Offline fresh traversal: only `/imu/data` and `/velodyne_points`, 39,315 and 974 messages.
- Five unit tests: cross-product sign; yaw-only rotation/lever nullspace; zero
  motion (no NaN energy-ratio claim); exact preservation of negative time offsets;
  invalid timestamps and NaN offsets rejected. All passed with `-W error`.
- `py_compile`: analyze_motion.py, excitation.py, test_excitation.py passed.
- Cross-artifact checks: JSON parses, 974 CSV rows, 973 bracketed intervals,
  reported energy fraction matches raw metrics, exactly 16 source records,
  no candidate matrix and fusion readiness false.
- `git diff --check` passed; no production `src/` differences from R2.

## Five-axis self-review

- Correctness: the Gram matrices are explicitly *diagnostics*, not full joint
  Fisher matrices. Numerical full rank is retained as counter-evidence to a
  strict UNOBSERVABLE claim. No accepted matrix means SO(3)/inverse validation is
  N/A, not a pass. Header-relative timing is conditional on the producer contract.
- Readability: separate small excitation math, bag traversal and reports; no
  framework or production integration introduced.
- Architecture: offline-only docs tools; no launch/CMake/EKF/Deskew/NDT changes.
- Safety: bag opened read-only; output directory must not exist (exclusive file
  creation too). No GT-topic selection, external storage, sudo, dependency install
  or source mutation. Public downloaded metadata are outside stable workspaces.
- Cost: one bounded fresh sensor traversal; no matching/optimization; no whole
  point cloud trajectory output or raw measurements added to Git.

No independent agent/CLI review was run or claimed. Source issue API HTTP403 is
an acknowledged investigation limit, not interpreted as an empty author answer.
The incremental/code-review skill checklists guided the offline-only split and
the explicit retention of counter-evidence; no production behavior was changed.
