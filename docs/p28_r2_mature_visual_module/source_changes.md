# Actual source changes

- `scripts/p28_visual/prepare_tracks.py`: raw camera/MEI input to bounded genuine
  FAST/KLT pixel tracks, no PnP/depth/GT. `run_replay.py`: isolated inherited ROS
  launch/resource runner, new visual switch, unchanged map/bag contracts.
  `freeze.py`: input/output hashes before any GT access.
- `core/msckf_visual_measurement.hpp/.cpp`: independent camera projection and
  right-pose Jacobians, bounded landmark triangulation, nuisance-feature QR
  nullspace, dynamic clone factors, correlated-noise Joseph root primitive,
  actual causal IMU F/G linearization used at runtime.
- `fusion/msckf_visual.cpp`: standalone P28 loader, calibrated real pixel
  measurements, exact image-time clones, held interval-noise driver, FEJ
  registry not overwritten by replay, track-level NIS, compression, DCReg
  overlap/constrained gain, right injection/reset, fixed-lag replay and logs.
- `estimator_types.hpp`, `dog_prior_map_ekf_node.hpp`: joint clone/root/driver
  storage alongside the unchanged 15D core. `imu_processor.cpp`: default-off
  dispatch, joint history save/restore and visual event scheduler.
- `ndt_observation.cpp`: default-off joint snapshot backup and same NDT DPD
  core-row scaling, no changes to nominal corrections or blend 0.6.
- `dog_prior_map_ekf_node_core.cpp`: isolated module initialization only.
- `VisualDegeneracy.msg` and NDT node: timestamped independent DCReg report
  with FULL scan support end. Existing detector and Multi-NDT are unchanged.
  Initialization gate recognizes the new P28 mode when old vision is OFF;
  other modes retain the original gate. No extra NDT align is added here.
- Split launch/config: new visual diagnostics and enable override. Original
  configurations keep both new adapter and new visual update disabled.
- CMake and `msckf_visual_measurement_test.cpp`: directed numerical tests.

## Explicit shared A/B flow differences from P19/P24/P27

Whole-interval midpoint becomes causal head/ZOH with camera event subdivision.
IMU propagation Jacobian uses right attitude error (including Exp and Jr),
position/bias couplings and the same physical noise parameters. The stochastic
IMU measurement driver is held across camera subdivisions; partial/full noise
cross-covariance is explicit. Extra joint factor/short clones are stored even in
A. B alone consumes pixel residuals. Thus B−A tests this new visual update,
not the combined difference from historical P24/P27.

Legacy continuous-gravity heuristic is not part of the new causal model;
frozen Corridor01 acceleration is in m/s², outside its historical norm≈1 gate.
No initialization/gravity vector or noise numbers are tuned. NDT pose fusion
and its DPD remain heuristic; exact Bayesian/statistical consistency of the
entire hybrid estimator is NOT established by a clone or PSD test.

## Concentrated build/test and first startup fault

Implementation was compiled incrementally in the existing isolated catkin
tree, not once per ROS experiment. Focused tests verify actual projection/F,
clone correlation, shared-noise positive Joseph root and legacy root/OOSM
interfaces. The first launch attempt exited before any bag playback because
the historical P24 gate disallowed old-vision-OFF/new-vision mode. This explicit
interface omission was repaired; the attempt and its original receipt remain
untouched. It is not a completed sensor prefix or an accuracy experiment.
