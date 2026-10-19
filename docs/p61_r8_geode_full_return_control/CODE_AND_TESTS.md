# Actual implementation and verification

All following production files are inherited unchanged from accepted R7:

| Stage | Actual source location |
|---|---|
| Fullprecision state and fixed15D restart prior | `src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp:390` |
| Real IMU propagation and original-header high-rate publication | `src/dog_prior_map_localization/src/imu_processor.cpp:153` |
| Scan-end deskew and same-end prediction | `src/dog_prior_map_localization/src/imu_deskew_runtime.cpp` |
| Actual Gaussian resolution before setInputTarget | `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp:1643` |
| Source preprocessing | `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp:1820` |
| Same-scan-end prediction as initial guess | `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp:2244` |
| One normal align per input | `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp:2345` |
| Native limiter and actual publication | `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp:2562` |
| Geometry CSV's literal PASS (not validated coverage) | `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp:2635` |
| Limiter against previous used NDT pose | `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp:2694` |
| LiDAR→IMU measurement transform | `src/dog_prior_map_localization/src/fusion/ndt_observation.cpp:217` |
| Held-head restore to exact native measurement time | `src/dog_prior_map_localization/src/fusion/ndt_observation.cpp:273` |
| Fixed-ratio correction | `src/dog_prior_map_localization/src/fusion/ndt_observation.cpp:319` |
| Existing velocity_blend0.6 | `src/dog_prior_map_localization/src/fusion/ndt_observation.cpp:334` |
| Snapshot, real-tail replay and corrected publish | `src/dog_prior_map_localization/src/fusion/ndt_observation.cpp:372` |
| Actual pose publication | `src/dog_prior_map_localization/src/ros_output.cpp:8` |

The extra candidate `ndt_.align` call in the inherited dormant branch is not
invoked: CONTROL, recovery and feedback are off; actual per-frame diagnostics and
2441 publications contain no candidate/extra align, jet or value calls.

R8 additions are isolated in this documentation directory:
`prepare_inputs.py`, `replay_full.py`, `terminal_contract.py`, launch/RViz,
post-run `analyze_run.py`, `archive_run.py`, and `verify_archive.py`.
The R8 code commit contains only the pre-run preparation/recorder/launch/tests;
the subsequent archive commit contains analysis and data, not a new runtime.

Read-only tests after the formal run:

- R8 pre-run unittest:3 test methods PASS (boundary/source manifest, terminal outcomes).
- R8 post-CLI unittest:5 methods PASS, adding missing/stale/correct/early lineage
  evidence cases. Archive-only recorder change did not participate in this run.
- The fifth case guards exact integer-ns→native-double conversion. Premature
  conversion of epoch ns before division caused652 false mismatches in a new
  offline semantic gate; exact rational conversion yields zero. All other native
  replay, receive-order and event-order semantic checks also yield zero violations.
- Existing R7 input/ACK/TCPROS tests:4 methods PASS, no estimator run.
- `verify_archive.py`: all raw file sizes/SHA in `output_freeze.json` unchanged;
  all2441 original scan indices in order; all2441 distinct callback→rollback→
  correction→replay→publication event chains; state revisions and NDT-update
  counts link each frame; scan-end correction/native-time equality; no post-end
  IMU in measurement prediction; same current time restored after replay.
- First20 R7 source, optimizer input/output/used, fitness and iterations equal.
- Six normalized archived CSVs have exactly identical cell strings to raw data.
- `py_compile` and `git diff --check`; no estimator rebuild or extra align.

The failure-terminal paths are policy/unit-tested, but not exercised by actual
NDT nonconvergence in this run: all2441 native statuses report convergence.
Matching-quality anomalies remain preserved separately, without false zero-error
interpretations of those native statuses.

Local adversarial review closed recorder timing and evidence association issues.
The user selected optional CLI review after the formal run had already finished;
the exact bounded package/command was separately presented for confirmation.
Do not retrospectively label that review as a pre-run gate.
