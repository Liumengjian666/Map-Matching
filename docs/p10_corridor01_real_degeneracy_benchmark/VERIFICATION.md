# Verification receipt

No verification command reruns dataset NDT. The single scientific execution
is identified by `experiment_freeze.json`, `RUN_STARTED` and `execution.json`.

Release build directory: `/tmp/p10_corridor_benchmark_build`.

Configure from the new `scripts/p10/corridor_benchmark/CMakeLists.txt` using
`-DCMAKE_BUILD_TYPE=Release` and the existing FAST-LIO2 checkout at commit
`7cc4175de6f8ba2edf34bab02a42195b141027e9`.
Build target: `p10_corridor_benchmark`, `-j2`.
Runtime/build environment uses `LD_LIBRARY_PATH=/lib/x86_64-linux-gnu` to avoid
the inherited incompatible MVS/libusb resolution. Compiler GNU 9.4.0, CMake
3.16.3, PCL 1.10.0, C++14, `-O3 -DNDEBUG`. NumPy 1.24.4 is used only for
postprocessing. No dependency installed or upgraded.

Checks actually executed:

- New Release CTest: 9/9 PASS; `release_ctest.log` preserves results. Includes
  time/frame/future-IMU/lever-arm self-test and original P7/NDT/P10 R2–R7 tests.
- Existing P9 CTest suite: 41/41 PASS, `p9_ctest.log`. This is compatibility
  verification, not a new Corridor01 oracle or dataset registration run.
- Python execution-contract tests: 4/4 PASS. Streamed hashes, relative/absolute
  script-path regression, synthetic missing-input error with zero NDT calls,
  and use of canonical modules rather than IKFoM/covariance reimplementation.
- Python reporting tests: 5/5 PASS. LF output format, separate convergence recovery from position
  truth, reject false full-sequence receipts, distinguish intermediate strong
  candidate choice from accepted feedback, and reject source/budget/future-IMU/
  nonfinite pose corruption.
- Reused input hash gate: 15/15 PASS before real registration.
- Execution CSV audit: checked against receipts, common source/hash/map,
  finite rigid poses, final execution path, strict finite NDT terminals,
  causal IMU timing and budgets. See `execution_audit.json`.
- Canonical kernel parity: committed code versus current files, without
  modifications to R6, Anchor, NDT or replay reader. See `source_provenance.json`.
- No GT reads, no oracle/B12, no full IKFoM, no map reload/second target object.

Final artifact hash and JSON/CSV audits plus `git diff --check` are recorded
with the result commit. Old experimental archives and stable workspace are
not modified. `NaN` cells in unevaluated candidate-score fields mean NOT_RUN;
they are not accepted poses, zero evidence, or invented accuracy values.

Remote read-only query failed with `Couldn't connect to server`.
The user-reported remote baseline is `ea12a9edd757bc08bebd83f75405c28982704b13`;
this session does not claim independently verified current remote HEAD.
No push is executed.

During final reporting, adding the unsegmented processing-time statistic exposed
a missing `common_source_ms` column in two synthetic test fixtures. The real CSV
already contains this column. Updated the fixtures, reran all reporting tests;
no executor, input or scientific threshold change and no NDT rerun.

Staged whitespace audit exposed the standard CSV writer's CRLF and trailing
whitespace in a copied generated CMake flags record. Derived archive text now
uses LF and whitespace-clean flags; original execution cache/build record and
all scientific numeric values are unchanged. Added LF regression test and
regenerated artifact hashes. No scientific execution repeated.
