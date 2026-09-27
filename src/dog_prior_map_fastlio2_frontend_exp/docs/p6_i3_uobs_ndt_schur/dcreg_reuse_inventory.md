# DCReg Reuse Inventory (read-only reference)

Reference repository: `/home/jian/livox_ws/DCReg`
Pinned commit: `ce7db8220f549a4a4391729e3bf4de4d4ab74635`
Working tree at audit: clean. No files in that repository are modified by P6-I3.

## Reusable analysis ideas

- **Partition and Schur formulas:** `DCReg/include/dcreg.hpp`, `DetectDegeneracy`, partitions a rotation/translation normal matrix and forms both directional Schur complements. The separation concept transfers to a symmetric PCL NDT score-information matrix after explicit canonicalization and fixed unit scaling.
- **Small symmetric eigensystems:** DCReg uses `Eigen::SelfAdjointEigenSolver<Matrix3d>` for the 3x3 rotational and translational Schur blocks. This solver class is suitable for the symmetric PCL-derived blocks; eigenvalues are ascending and eigenvectors are columns.
- **Physical-axis basis matching:** `AlignEigenBasisToAxes` greedily assigns each basis vector to a unique canonical x/y/z reference by maximum absolute dot product and fixes the sign. Squared basis coefficients are used as contribution ratios. These descriptive operations can be reused in an adapter without reusing any degeneracy threshold.
- **Numerical failure as data:** DCReg exposes factorization success and reports spectra/condition values. P6-I3 similarly reports solve method, rank/conditioning, and failures rather than forcing a binary degeneracy label.

## Not directly portable

- DCReg's Hessian is an ICP point-to-plane weighted normal matrix `JᵀJ`, not PCL NDT's Hessian of its scalar probability score. Its sign, units, parameterization, and scale are not interchangeable.
- The NDT analytic parameter vector is translation-first and Euler-angle based; P6-I3 must permute it to canonical `[rx, ry, rz, tx, ty, tz]` and negate score curvature at a local score maximum before treating it as local information.
- DCReg's `degeneracy_condition_threshold`, `kappa_target`, weak-axis mask, and eigenvalue clamping are ICP-specific control settings. They are intentionally not imported or used to produce a P6-I3 binary label or runtime trigger.
- DCReg's ICP registration, normal-equation update, preconditioner, PCG, QR fallback, and pose output are out of scope. P6-I3 is diagnosis only; it does not modify the optimizer or pose.
- DCReg currently forms inverses after `FullPivLU::isInvertible()`. P6-I3 uses a no-explicit-inverse solve adapter and records factorization/rank status; no DCReg source change is needed or allowed.

## Relevant implementation locations

- `DCReg/include/dcreg.hpp`: `AlignEigenBasisToAxes`, `DetectDegeneracy`, `CharacterizeDegeneracy`, `SolveRawNormalEquation`, `SolvePreconditionedUpdate`, and `BuildSe3LinearSystem`.
- `DCReg/include/utils.hpp`: `[roll,pitch,yaw,x,y,z]` pose fields and `SolverParameters` including ICP-specific threshold and preconditioner parameters.
