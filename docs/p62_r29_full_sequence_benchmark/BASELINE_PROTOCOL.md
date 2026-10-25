# Frozen R29 complete-sequence protocol (before formal accuracy)

START fa08c12ac965206e576643dae578b63d49719d6f. Algorithms inherited from frozen R24/R25. Native PCL NDT .8m resolution, .08 step, .001 epsilon,40iterations,.55outlier. Source .25m voxels, .5–80m, cap1400. Map P45 two .15m XYZ voxel passes /20m XY tiles,10m source safety and30m selection hysteresis. Same sensor stream, map and anchor state for every matched policy. No threshold tuning or future reference in online state.

A ORIGINAL: inherited pre-geometry veto median<=1m,p90<=3m,overlap@.5>=.50; nominal native finite rigid/converged output; inherited observation step limits and EKF/OOSM safety. No retries.

B G_ONLY: original pre-geometry quantities recorded diagnostically, no optimization veto; frozen basic candidate quality and Coverage Consistency Mean<=.30; no retries.

C C_PLUS_G: same as B, retries only after nominal quality failure, fixed map-yaw +5/-5 attitude seeds preserving TP position, max native Score among valid candidates with existing distance/ID ties. Exactly the inherited rules, no relative D/E/F. Each disjoint block of60 **input scans** has max12 extra Full calls; event max2; unused budget does not accumulate. Only budget resets, never predictor/filter/state. A final short block has the same12-call ceiling.

D NATIVE_NO_PRE_GATE: same diagnostic pre-gate behavior as G, but no Coverage/Overlap/.30 admission veto. Require native finite score/fitness, valid SE3 and native convergence; retain inherited .5m/5degree adjacent published-observation step limits and existing EKF2m/20degree/OOSM safeguards. No retries. Quality statistics are still logged. This is a controlled native registration+our EKF baseline, **not** official HDL Localization. Unlike B, it can admit a numerically valid match that fails the heuristic consistency gate. A vs D isolates pre-gate; B vs D isolates post-quality veto. No duplicate independent algorithm claim.

Run order: admitted Koide Easy first, all A/B/C/D C0; then fixed+8degree map-Z initialization, preserving LiDAR position and recomputing IMU lever arm; then Hard with the same order. Anchor is first point stamp with legal reference/IMU support; one initial reference only, zero velocity/bias priors, no later reset. Include complete continuous input after anchor to the last IMU-supported scan and disclose any unbracketed prefix/suffix. No manual exclusion of poor output.

Stop expansion if completed C0 C+G has a clear safety side effect relative to A/D (translation RMSE worsening>.03m or rotation>.3degree, with high-error episodes disclosed); preserve all completed policy results, do not tune or move to new sensors/windows. If legal Koide execution is blocked, continue the other specified sequence, not new datasets. Conditional Newer College full H_B/H_C runs are secondary and cannot conceal a Koide failure or bypass this safety stop.

Reference evaluation only after all compared online outputs are sealed. Report direct map-frame translation/rotation RMSE,P95,Max with **no post-hoc alignment**, labelled map-anchored/batch benchmark reference. ATE is direct map-frame position error, not aligned odometry ATE. RPE uses relative SE3 over1sensor-second, nearest endpoint<=50ms, reference gaps<=150ms. Reference interpolation endpoint allowance50ms. Full metrics require complete finite estimates; partial-reference/partial-execution coverage must be explicit, never called complete success.

A reference-high-error frame exceeds.5m or5degrees. A sustained episode/recovery needs3 consecutive high/low frames; report startup separately from tracking loss. Recovery latency uses sensor time, not process wall time. Rejections are not zero-error observations. Full EKF output at each scan's actual consumed IMU watermark is scored, including propagation-only output. Secondary raw NDT errors have their own coverage.

Error-admission diagnostics are reference-relative proxies: accepted candidate high-error and worsens corresponding TP error by>.03m or>.3degree; beneficial rejected candidate improves one by those margins without worsening the other. These are offline descriptors, not online rules or proof from independent physical truth. Report detection, selection, measurement publication, actual EKF update, and reference improvement separately.

Resource scope: sampled per-node and simultaneous NDT+EKF RSS at50ms; callback mean/P95/max, event max, wall/inputspan rate, native nominal/extra lists, target hashes/counts and tile switches. Initialization costs separate. No 10Hz certification from averages.
