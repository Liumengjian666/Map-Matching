# Source and verification scope

- `core/imu_deskew.hpp/.cpp`: immutable scan-local trajectory, causal anchor,
  raw timestamp/coverage gates and read-only point queries; old helpers retained.
- `imu_deskew_runtime.cpp`: default-off branch chooses the new trajectory;
  original SE(3), point order/layout, extrinsic and safety bounds unchanged.
  Optional diagnostics on the eight already-prescribed scans only.
- `dog_prior_map_ekf_node.hpp/core.cpp`: experimental flag, optional stream and
  selected diagnostic identities; incompatible non-mature mode refuses startup.
- `CMakeLists.txt`, `tests/scan_local_imu_test.cpp`: directed ROS-free tests.
- `config/p30_scan_local_deskew.yaml`: one physical key; other overlays retain
  frozen P28 physical configuration and only add output/capture paths.
- `p28_visual/run_replay.py`: explicit P30 identity, fixed21s prefix allowance
  and runtime flag check; old invocation defaults remain unchanged.
- `p30_deskew/`: preparation, actual runtime/source gates, fixed offline GT
  evaluation, bounded arithmetic regression tests and light archival.

NDT executable unchanged. Only EKF rebuilt in `/tmp/p30-build.Mk4FPk/devel`;
final CXX_FLAGS match P29 `-O2 -g -Wall`, same GNU9.4/Eigen3.3.7/PCL1.10.
Initial configure used Release; before any ROS run it was changed to P29's
actual empty build-type flags and uniformly rebuilt. No replay used the earlier
Release binary. Unchanged IMU/filter/MSCKF/OOSM core files were compared with
the actual P29 isolated build source and match byte-for-byte.

One21s short and one40s full C replay only. No A/B rerun, parameter search,
candidate strategy change, new optimizer, map copy or subsequent experiment.
Raw source/pose binaries, deskew bags and outputs bags remain local in the
independent `.p30_r1_scan_local_deskew` cache; none are submitted to Git.

Two posthoc-only statistics errors were fixed without altering frozen outputs:
timings belong to DiagnosticArray in outputs.bag, not determinism CSV; visual
OFF has no applied-update timing samples and is recorded as count0/missing,
not zero-cost vision. Exact count396/finite guards and artifact tests cover both.
The first-divergence comparator uses normalized rotations to avoid an acos
rounding residue falsely labeling identical frame1 initial rotations as changed.
GT errors continue using the unchanged original evaluator protocol.
