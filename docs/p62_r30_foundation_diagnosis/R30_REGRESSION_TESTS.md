# Necessary tests and actual run accounting

1. The supplied `test_r30_trace_preflight.py`:2/2 Python tests pass. The supplied read-only preflight was then run once on all four committed R29 trajectories, each2023 rows; B/C equality and failure retention confirmed.
2. Old global-PCD/PCL construction reproducer: expected exit1, configured.8m versus actual1m cells; zero native align calls. Initial diagnostic compile errors (include path/pkg-config suffix/Eigen include) remain in logs and are not benchmark failures or passes.
3. Independent ROS-free Release C++ tests:6/6 pass: target resolution, frame/lever-arm/twist conversion, causal pose prediction, IMU interval numerics, IMU Deskew, OOSM replay planner. Actual commands, exits and outputs are in CORE_REGRESSION_RESULTS.json.
4. Actual repaired node Release build passes with original ROS Noetic/Livox underlay, no environment upgrade/install. Command:

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="dog_prior_map_interfaces;dog_prior_map_localization" --make-args -j1 dog_prior_map_ndt_node_cpp ndt_target_resolution_contract_test
```

5. CTest from the real `build` working directory runs exactly1 target regression and passes. The first invocation used an unsupported `--test-dir` form and discovered no tests; its output is preserved and is not counted as passing. CMake-built regression also ran directly and passed.
6. Eight predeclared reference-seeded native NDT probes completed once:8 finite/converged,0 fused observations,0 ROS/EKF runs,0 complete replay trajectories. Same Source FNV and target FNV/93146points checked before each align. The reference is a map-anchored batch estimate, not independent truth.
7. Reused Sources for an8-frame Mask count check, with zero align. Exact raw62 translation-limiter reproduction is zero-error; cross-library rotation normalization differs2.85e-8 and is explicitly outside that exact-reproduction claim. The initially overbroad whole-matrix check was narrowed to the actual translation causal claim, without changing an algorithm or numerical threshold.
8. Existing logs explain all185 stale-query latest sample identities/method. A/B/C/D REPLAY_INCOMPLETE5/2/2/11 show no per-observation correction/replay-complete event. This is evidence about the rejected transaction, not a new physical correctness certificate.

Actual probe executable/run command:

```bash
env LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0 .r30_diagnostic_build/reference_seeded_probe docs/p62_r30_foundation_diagnosis/evidence/probe_inputs/targets.txt docs/p62_r30_foundation_diagnosis/evidence/probe_inputs/requests.tsv docs/p62_r30_foundation_diagnosis/evidence/REFERENCE_SEEDED_NDT.csv
```

This command records the completed invocation, not authorization to rerun/overwrite it. Raw timing/RSS, compile command, actual binary and input hashes are retained. All probes together stayed within8 native align calls. No extra call is hidden as a regression or confirmation.
