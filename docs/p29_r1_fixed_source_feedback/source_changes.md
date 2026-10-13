# P29 source and execution scope

Unchanged runtime sources: NDT optimizer, MSCKF update, EKF/IMU propagation,
Deskew, OOSM, DCReg, Multi-Hypothesis NDT, visual input and Step Limit.
No runtime binary was rebuilt. Frozen NDT SHA256:
`a33586a425dac0b35f366ef4e3d7da0cfdbc9d76c00132eb59d843bc09883c1d`;
EKF: `cbc2c1b39f3c9fa9b5fa7ae0307e252dc1f0e110f9ef91906ac48f7847349876`.

Runner change: optional default-false bounded passive Deskew recorder and P29
receipt identity. Real NDT Source still uses existing ExperimentalSourceCapture
without any new point transformation, ordering or reconstruction logic.

New offline harness: prepare.py freezes the prescribed exact IDs and inputs;
audit.py validates SHA/ordered point hashes and whole P28 output parity;
offline_pair.cpp calls unchanged ObservablePclNdt and P23 frozen production
map/hash/Step Limit helpers; run_offline.py enforces one run,32 align budget,
and output freeze. analyze.py performs finite same-cell effects and frozen GT
evaluation. test_contract.py tests IDs, real source/cell parity, shared-transform
isometry and nonzero-lever-arm origin conversion. archive.py copies only light
evidence, with CSV newline normalization recorded separately.

The standalone C++14/Release CMake build uses existing system PCL1.10/Eigen,
not a different optimizer. The initial pkg-config attempt failed because PCL
metadata was absent from its search path; CMake using the already-configured PCL
installation succeeded. Sixteen self-cells match captured native results exactly.
No dependency, sensor model, parameter or solver fix was required.

Preparation/native-harness commit:
`f509e5cd5e5c038c7224f722b00a8e8d5b9ff9f3`.
Analysis/archive delivery SHA is supplied by final Git/chat receipt.

Analysis source commit:
`5eb68f204f34c122ead5a5093dd892ee0a0e803f`.
