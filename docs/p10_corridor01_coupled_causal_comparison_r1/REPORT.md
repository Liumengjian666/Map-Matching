# P10 Corridor01 Coupled Causal Comparison R1 — implementation stage

## Status

`SOURCE_INTEGRATION_AND_BUILD_READY_SHADOW_REPLAY_REQUIRED`

R6 refinement has been integrated into the existing ROS split NDT node and
compiled. The control/shadow parity test and frozen output checks must still be
run in a regular Ubuntu ROS terminal; the sandbox cannot create the ROS local
TCP/XML-RPC sockets. No Shadow, Weak-only, or Coupled ROS replay has yet been
run by Codex, and no new GT evaluation has been performed. Therefore this is
not an accuracy result and does not establish a Coupled advantage.

## Implemented

- `ObservablePclNdt` factors the derivative/value objective path already used
  by R6 into a shared PCL subclass. The ROS NDT instance remains the one that
  performs nominal `align()`, owns the preprocessed target grid, and evaluates
  refinement jets/scores against the same current source cloud.
- The launch exposes `coupled_mode`, default `CONTROL`.
- The NDT node can consume causal `/dog_livo/odom_high_rate` poses, validates
  the configured `map_T_lidar` frames, chooses only a pose at-or-before scan
  reference time, and advances it to the scan timestamp with the published
  LiDAR-frame body twist. The twist is already transformed from the EKF IMU
  state with the frozen extrinsic and lever arm. Extrapolation is bounded to
  20 ms; sample/target stamps and age are logged separately. It also logs the
  prediction, anchor, R6 candidate, scores, correction coordinates, step
  limiting, and actual observation substitution.
- `COUPLED_SHADOW` leaves the used NDT measurement nominal. Weak-only and
  Coupled feedback substitute only a recommended candidate and still pass
  through the original `limitNdtStep()` and EKF OOSM path.
- Two post-run tools are added: strict frozen-Control Shadow parity and GT
  evaluation with the already frozen Control transform. Neither tool selects
  candidates or modifies runtime outputs.

## Verification performed here

- Catkin build completed with the existing optimized `-O2` compilation
  configuration (the temporary CMake Release build was discarded for the run
  binaries so the EKF SHA remains identical to Control).
- New PCL native jet/value score parity test passed:
  score `24084.3`, relative tolerance `1e-12`.
- Existing OOSM replay planner, IMU interval, frame conversion, and IMU deskew
  contract executables passed.
- New causal body-twist SE(3) prediction contract passed exact-time,
  body-frame translation, constant-twist arc, and stale-age rejection cases.
- New Python tool `--help` startup checks, launch XML parse, and `git diff
  --check` passed.
- New NDT binary SHA256:
  `9da66b47b89053e523f3ce2d18086395e565ef6d9353ebd34c4f4f315afd3db4`.
- EKF binary SHA256 exactly matches frozen Control:
  `4d1655cc3f7d3fdc6cac2372f380ca8fe7b0b250227235767947da3d4030f402`.
- Input slice, normalized map, extrinsic, and frozen Control topics bag hashes
  match their archived values. The base and dataset configuration hashes also
  match the frozen manifest; the launch has only the opt-in `coupled_mode`
  argument/parameter addition and keeps `CONTROL` as its default.

## Not run

```text
COUPLED_SHADOW ROS replay = NOT_RUN
WEAK_ONLY_FEEDBACK ROS replay = NOT_RUN
COUPLED_FEEDBACK ROS replay = NOT_RUN
new NDT alignment calls = 0
GT loaded for this task = NO
trajectory metrics / runtime / RSS = NOT_RUN
```

The exact Shadow terminal procedure is in `RUN_SHADOW.md`. Its parity result is
the hard gate for the two feedback replays.
