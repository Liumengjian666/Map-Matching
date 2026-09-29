# Analytic LiDAR Jacobian test

`p6_alg_integration_a2b_test` perturbs all six physical NDT axes explicitly. It
does not call `Vector3d::Unit(axis)` for indices 3--5. The fixture uses nonzero
state rotation/translation, measurement rotation/translation, and LiDAR lever
arm.

Checks:

- analytic `A_exact` versus central FD for rotation X/Y/Z and translation X/Y/Z;
- two simultaneous weak axes;
- `B^T A_exact U_w` leakage below `1e-9`;
- zero-angle series branch;
- near-pi explicit `ROTATION_RESIDUAL_NEAR_PI` rejection.

Release and Debug results: PASS.
