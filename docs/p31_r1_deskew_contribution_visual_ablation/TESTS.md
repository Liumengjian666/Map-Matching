# Actual targeted verification

New isolated build: `/tmp/p31-build.k3TwQd/{build,devel}`; GNU9.4,
Eigen3.3.7/PCL1.10, unchanged final `-O2 -g -Wall`, empty CMAKE_BUILD_TYPE.
`deskew_trajectory_mode_test` and `dog_prior_map_ekf_node_cpp`: build PASS.
Historical NDT executable is copied without replacement and hash-verified;
no NDT rebuild and no protected workspace build.

Executed `deskew_trajectory_mode_test`: PASS for defaults/mutual exclusion,
Deskew-only flag copy, global physical config immutability, D exact historical
correction retention, causal head-only partial queries both before/within tail
horizon, unchanged C node results, immutable stored history.

Executed `scripts/p31_ablation/test_gate.py`: 3 tests PASS. Coverage includes
finite/on-time visual terminal release, queued release NaN legal, future/NaN
Deskew source time rejected. These were added following one bounded independent
read-only review. Review findings were gate-only; runtime binaries did not
change afterward. Original P30 mathematical suite was not repeated.

Python compilation of changed/new scripts PASS. No full default-off trajectory
replay or binary-level default-off parity is claimed. E/common-state isolation
is additionally verified against actual ROS parameter dumps and DISABLED
measurement diagnostics, not merely a config filename.

Artifact tests after output freeze recompute reported D/E translation/rotation
statistics, comparison deltas, retain historical A/B/C values, and verify OFF
timing is unmeasured rather than zero milliseconds. See final REPORT/results
for the actual run and artifact-test outcome.
