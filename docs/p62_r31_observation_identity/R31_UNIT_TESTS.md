# R31 tests completed before formal input

Eight independent ROS-free cases pass:
1. admitted unchanged terminal is published unchanged;
2. translation-only limitation is refused;
3. rotation-only limitation is refused;
4. both limitations are refused;
5. original quality refusal remains refused;
6. unconverged NDT is refused;
7. no previous map observation imposes no fictitious limiter;
8. refusal does not mutate history supplied to the pure decision.

The unchanged limiter is shared with the actual node; tests do not implement a different approximate selector. For every permitted STRICT result the decision pose is the identical original matrix. Runtime state-before/after and output counters additionally test actual node history, avoiding reliance on a standalone fixture alone.

Raw62 uses the exact frozen raw/previous matrices: LEGACY full matrix reproduces the historical published matrix with difference0, STRICT refuses. No native align or EKF operation is performed by these tests. These are semantic checks, not accuracy experiments.

Actual native node, real IMU/EKF node, real input publisher and unit executable built in Release using the existing ROS Noetic/Livox underlay. CTest from the actual build directory runs1 test executable successfully; the executable reports8 cases. Compile logs and unit outputs are retained. No ROS environment upgrade or dependency install occurred.

The subsequent C startup abort was a scheduler error, not a failed identity test: NDT appeared before EKF in OS polling. It received0 scans, performed0 native align/IMU propagation, and is preserved separately. The monitoring-only fix enables the remaining predeclared input without rerunning B or adding a trajectory with processed input. No reference error was read to make that correction.

## Complete real-node checks after all output was sealed

-4106/4106 per-scan publication decisions agree with independent captured ROS NDT messages;65 published messages match canonical certified pose within8.882e-16, including normalized quaternion serialization. Strict admission itself uses exact unchanged matrix plus limiter flag, not this logging tolerance.
-4041 rejected frames keep previous_pose/delta/current NDT pose/reference timestamp and publish count unchanged. Published Path/dynamic TF are inside the guarded publishPose call; rejected frames cannot enter that call or the aligned-observation block. Raw sensor, diagnostics and static coordinate TF remain normal.
-Actual EKF evidence:65 NDT_CALLBACK_ENTER,65 corrections and65 completed OOSM replays, matching65 publisher ACKs; no raw62/raw96 callback/update for either strict Indoor run.
-4106 queried state timestamps are not later than reference time; receipt wall time precedes prediction query and actual NDT start.65 applied OOSM records have measurement≤state-now, rollback≤measurement and original alignment tolerance. The inherited IMU max_dt gaps/stale-query rejection are not repaired or certified away by these tests.
-384+480+68 actual native calls=932;104 extra calls, per-event≤2 and every disjoint60-input-frame block≤12. No extra LEGACY trajectory or unreported align.
-Stable workspace BEFORE/AFTER snapshot identical; frozen NDT/EKF/publisher/test binary SHA all unchanged after execution.

Focused self-review: default LEGACY still selects the original legacy publication path; unknown policy fails startup; STRICT is restricted to approved C/G configuration; quality and convergence captured before limiter; rejection occurs before map-history/pose/Path/TF mutation; EKF/IMU/OOSM and thresholds unchanged. Complete performance failure is retained instead of adding a second fix. Review scope is this semantic boundary, not a new all-source audit.

Orchestration limitation: initial C launch had zero input and was stopped by a too-early component-exit guard. Only the supervisor was repaired; original receipt retained, no NDT binary change, no completed run repeated.
