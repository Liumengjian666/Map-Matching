# Tests, fresh-context review and retained failures

- test_prepare.py:5 synthetic schema/zero-padding/SE3 tests PASS (test_prepare.log).
- test_pilot.cpp: actual C++ hold/body-CV/causality/rejection/native nonfinite/frozen45 tests PASS (test_pilot.log).
- frozen R13 coarse tests:64 checks PASS; frozen R12 pool tests:27 cases PASS. No real NDT align in these tests.
- test_evaluation.py:16 tests PASS, including seal/registration rejection before reference admission, missing/duplicate calls, accepted/native matrix consistency, NA exclusion, noncommuting frame chain, zero/pi rotation and declared linear P95.
- Native preflight21 originals: actual PCL declared-byte and XYZ-bit equivalence PASS. Map built once; formal Source FNV/block counts match preflight all20.
- Fresh-context local input/runtime/evaluation reviews found the target-filesystem/preregistration binding gaps, native-score finite-stop gap, and incomplete evaluation admission guard. They were patched and tested BEFORE formal NDT. Final/native matrix consistency was also added before the source freeze. No external CLI was run.

Retained failures: initial preflight executable loader exited127 before main, due to MVS libusb missing libusb_set_option; no PCD/map run happened. Original receipts/log/binary retained. A separately named repair restored ONLY the existing R13 child LD_PRELOAD system-libusb and LD_BIND_NOW settings, same binary, then the native input/map test passed. No system configuration changed. An initial build request before CMake regeneration had no pilot target; build_pilot.log retained, reconfiguration then successful build. These are not successful first attempts and not extra localization experiments.

Formal run is one-shot guarded, input/source/binary/SHA sealed,40 actual native calls, no rerun. Visualization smoke is ROS publishing with rviz=false, not actual GUI rendering. Existing ROS log directory>1GB warning retained; no cleanup performed.
