# Pre-GT independent audit correction (not an algorithm improvement)

The first Python audit stopped before any GT load. Engineering budgets and
8254 nominal/source/state comparisons passed. Eight individual W/S-fraction
recomputations exceeded the predeclared 1e-8 audit tolerance, maximum
1.1159356988477143e-7; individual anchor cost errors were <=2.17e-9.

Cause: Python scipy Rotation.from_matrix orthogonalizes float NDT matrices
differently from the pinned Eigen normalized-quaternion carrier used by the
runtime. W/S fraction divides by a small displacement squared norm, amplifying
that tiny rotation difference. The runtime uses Eigen::Quaterniond(matrix),
normalized, then the existing quaternion SO3 logarithm. The independent audit
now reproduces the matrix-to-quaternion Shoemake branches from the actual
/usr/include/eigen3/Eigen/src/Geometry/Quaternion.h (Eigen 3.3.7), without
changing the runtime, poses, scientific rules or 1e-8 tolerance.

Original failed CSVs are preserved at attempt_0 root. Corrected audits go to
attempt_0/audit_revision_1, with evaluator hash/code receipt before GT. No real
replay is repeated for this postprocessing error. It does not consume the one
permitted targeted algorithm improvement. debug_anchor_carrier.py preserves
the minimal non-GT reproduction. The CTest invocation initially used an
unsupported --test-dir flag in this CTest version; tests were actually run in
the build directory, 6/6 PASS. A synthetic equality test also needed its seed
to account for the first inertial increment; corrected before real replay.
