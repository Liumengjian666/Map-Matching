# PAPER-P6-ALG-INTEGRATION-A2C

RESULT: PASS (engineering bridge and lightweight regression only)

READY_FOR_FORMAL_EXPERIMENT = NO

START_SHA: `7329f74e5691c08379251046cce268477677f4f3`

CODE_SHA: `c78e01fc38d583d8839257c172c41eef8fc34e0e`

Branch: `research/p6-i6d-full-algorithm`

## Delivered boundary

The opt-in `FULL_FIXED_LAG_V2_EXPERIMENTAL` producer now connects causal IMU
preintegration, window-owned NDT prediction, geometric U_obs, window-covariance
U_nonlocal probes, selected LiDAR factors, prepared cross-state visual factors,
optimization and existing Schur marginalization. It does not modify the old
`FULL_ALGORITHM_V1` execution body or its default behavior.

IKFoM performs static initialization and an optional bounded initialization-epoch
prediction only. The initializer is destroyed before the event loop. The window
is the sole post-handoff state owner; there is no window-to-EKF-to-window cycle.

## Eight required answers

| Question | Answer / evidence |
|---|---|
| A2B stale A_exact closed? | YES. Risk stores frozen measurement/risk/commit metadata, not A. Admission reads the current active risk state, recomputes A, constructs Qw and freezes it. Optimized-state test passes. |
| Window pre-measurement covariance? | YES. Undamped joint H solve returns the latest 15D marginal before the new LiDAR factor. Independent full-inverse reference and singular rejection pass. |
| U_nonlocal independent of old IKFoM P? | YES. Trigger, principal probe direction and amplitude use window map-left P6. Unavailable covariance disables probes explicitly, without EKF fallback. |
| P0/P1/P2/P3 meanings retained? | YES. BASE/no NIS, ADAPTIVE/no NIS, BASE/selected NIS, ADAPTIVE/selected NIS. All four use the same producer. |
| NDT initial guess entirely window-owned? | YES. `prepareStateAt` precedes NDT; every nominal/probe seed is based on predicted WindowState and the real T_imu_lidar. |
| Visual still cross-state? | YES. The existing relative translation factor joins actual reference/current window states. No old projected EKF position update is called. |
| IKFoM stops after handoff? | YES. No IKFoM object exists in the event loop. Source boundary audit and fixture diagnostics report zero post-handoff prediction/measurement calls. |
| What is not in this producer? | Live ROS acquisition, raw image/depth feature production, state-dependent regeneration of saved deskewed clouds, formal long-sequence validation, and global relocalization. Prepared visual/cloud inputs are consumed rather than regenerated. |

## Verification

Release build: PASS. Release CTest: 14/14 PASS, including all 10 previously
registered regressions and 4 new A2C tests/audits.

Debug targeted build: PASS. Debug targeted CTest: 3/3 PASS, with Eigen assertions
enabled. No failed test was deleted or weakened.

Real PCL memory-cloud fixture: 3,000 target points before source preprocessing;
three scan events and two asynchronous visual pairs yield seven optimization
events per policy. Each policy commits three LiDAR factors, runs one positive /
negative probe pair, and makes five actual NDT align calls.

| Policy | Events | LiDAR factors | Visual factors | NDT calls | NDT time ms | Total ms |
|---|---:|---:|---:|---:|---:|---:|
| P0 / LEGACY_BASE_NO_GATE | 7 | 3 | 0 | 5 | 52.1791 | 81.2164 |
| P1 / ADAPTIVE_NO_GATE | 7 | 3 | 0 | 5 | 52.9817 | 81.7748 |
| P2 / BASE_SELECTED_NIS | 7 | 3 | 0 | 5 | 52.9942 | 82.1768 |
| P3 / ADAPTIVE_SELECTED_NIS | 7 | 3 | 0 | 5 | 54.6594 | 84.2122 |

The full-support fixture correctly rejects unneeded visual updates as normal
LiDAR. Directional visual admission and rejected-LiDAR relative-only fallback
are exercised by the adapter test with actual window factors. The fixture
contains synthetic visual input; it is not evidence of real-camera accuracy.

## Mathematical scope and limits

The 15D initialization prior conditions the 23D IKFoM posterior on fixed gravity;
it is not an equivalent 23D posterior. Covariance is a local linearized / Gaussian
approximation of the current joint objective, with existing fixed-lag prior
approximations. It is not a calibrated global uncertainty guarantee.

For fixed measurement, extrinsic and Uw, A's translation rows are state
independent. Optimizing the risk state changes A's rotation rows, not necessarily
Qw. The test verifies a measurable full-A change (>0.1 Frobenius norm), equality
to the recomputed admission A, and equality of Qw's projector to the recomputed
translation-subspace reference. No artificial Qw change was introduced.

During fixture wiring, a no-probe diagnostic reason was initially passed as a
terminal status. This falsely caused mature adaptive-noise code to reject later
LiDAR. It was fixed: algorithm status remains `NOT_PROBED`; trigger reasons stay
in the separate diagnostic field. Final regressions include all four policies.

## Preservation and handoff

Read-only source audit reconstructs the entire old runner by removing only the
new includes, mode dispatch and usage line, then compares exact source bytes to
START_SHA. It passes. No frozen result, baseline configuration, bag, PCD, image,
formal FULL path, or other workspace was modified.

No Floor01/Corridor01 replay, complete rosbag, GT tuning, accuracy evaluation,
formal FULL integration or next-stage work was performed. The incremental
implementation / Git workflow skills separated interface and producer commits;
the code-review skill supplied test-first boundary, covariance and causality
checks. Missing optional local skill-reference files did not alter the protocol.

See `DECISION_AI_HANDOFF.md` for the detailed scientific-controller report and
`BUILD_AND_CTEST_RESULTS.txt` for reproducible commands/results. STOP.
