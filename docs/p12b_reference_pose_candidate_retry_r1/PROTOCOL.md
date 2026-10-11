# P12B reference-pose candidate retry — frozen before registrations

Task: P12B-REFERENCE-POSE-CANDIDATE-RETRY-R1. Start/verified remote SHA:
`7a3afb0209f55b7c5246dd1a377b10607685577c`. Existing research branch and
safe workspace `/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`; dirty main and
stable robot workspaces are untouched. No new algorithm or localization replay.

## Coordinate decision and evidence

FAST-LIVO2 `LIVMapper::pointBodyToWorld` implements
`p_map = R_state * (extR * p_lidar + extT) + p_state`. Its pose logger writes
`lio_time`, `_state.pos_end` and `_state.rot_end`. Thus reference entries are
IMU/body-origin mapping states, nominally `map_T_imu`, not exact LiDAR poses.
At the same state time the correct composition would be
`map_T_lidar = map_T_imu * imu_T_lidar`, including rotated lever arm.

Current inspected mapping configuration:
`/home/jian/FAST-LIVO2_project/FAST-LIVO2/config/ours_config.yaml`,
`extrin_calib/extrinsic_R=I`, `extrinsic_T=(-.011,-.02329,.04412)m`.
This proves the current numerical configuration, NOT that these values were
used for the active Loop2 map. No frozen generating-run configuration was
established by the bounded inspection. These numbers are NOT applied or guessed.

`build_livox_keyframe_scan_db.py` selects the closest raw scan, copies mapping
pose, then overwrites the saved stamp with raw scan header. The 389-row P11
pairing ledger retains both clocks; max association offset is about .0502s.
It does not transport orientation/position to the header, midpoint or all
individual points. The builder stores raw, not deskewed scans. Database entry
stamp therefore is raw header, while pose belongs to a distinct mapping state.
No per-reference angular/linear motion transport is established here.

**Decision:** exact reference-to-LiDAR/time conversion is NOT CLOSED. Perform
only the user-authorized approximate-reference-pose diagnostic, then stop.
No GT, calibrated extrinsic invention, future data, or historical archaeology.

Claim under bounded independent review: the B interface changes only candidate
orientation from query prior to stored reference-state orientation, keeps
translation and IDs identical, and makes no exact-LiDAR-pose claim. Why it
matters: unresolved pose/time approximations must not become false evidence
of precise/global relocalization.

## A/B definition and immutable samples

A: historical P12 `coarsePosition(reference,query_ndt_prior)`:
reference IMU-origin translation + query NDT INITIAL rotation.

B: `APPROXIMATE_REFERENCE_STATE_POSE`:
copy reference IMU-origin translation AND reference-state rotation. No origin
or timestamp transport is claimed. This is NOT legal calibrated exact
map_T_lidar; it is an explicitly approximate initialization hypothesis.

Reference full quaternions are used, not just yaw. Point coordinates remain
in `livox_frame`; target remains the active `camera_init` map. Historical P12
deskew=false and header/start versus midpoint association remain unchanged.
The target/source direction is source -> map, not its inverse; DCReg terminal
already is a complete transform and receives no postmultiplication by initial.

Use exactly six original P12 frames: **16,45,124,708,1285,1891**. Selection is
fixed before B by first two chronological frames per sequential category:
1. any previously converged candidate;
2. not already selected, no converged candidate, at least one failed reference
   <=5m from query NDT initial position;
3. not already selected, no converged candidate, at least one failed reference
   >5m from query NDT initial position.
These use permitted previous P12 outcomes, not B results or GT. All three IDs
per chosen frame are frozen. No sample or ID replacement, threshold adjustment
or selective rerun. A outcomes/cost are reused, not newly measured paired timing.

External immutable selection lives at
`/home/jian/rosbag/p12b_reference_pose_retry_r1_20261011/experiment/`.
`freeze.json` timestamps/hashes its inputs before any B call. Live SOLiD query
must reproduce each P12 ID, similarity, reference pose and coarse A initial.
Six existing source PCDs are reused without extraction. Prepared source counts
and FNV hashes must match P12; target must have459154 points/FNV
`ea1e4e69cf9992c6`. Map/DB SHA are checked by freeze script.

## Unchanged solver, cost and stopping

Only **18 new calls**: same original external DCReg pin
`8ce8451b15491a4bbe17cf85ab02a8bed6696861`, SO3/Serial, DCReg, verbose=false,
has_ground_truth_pose=false, unmodified author SolverParameters defaults.
Radius1m, max30, convergence translation1e-3m/rotation1e-5rad stay fixed.
No local-prior rerun, no NDT/SOLiD mathematics or EKF/OOSM changes, no feedback.
Author license unconfirmed: original headers/binaries stay external, not vendored.

Log rotation/translation change, result SE3, iterations, convergence, failure,
pre-last-update point-plane residual, correspondence fraction, author degeneracy
mask and QR use. Author convergence is not location correctness. NDT is not GT.
Per-call wall includes target copy and KDtree construction; author timer starts
after tree setup. `api_setup_proxy_ms=call_wall_ms-author_iteration_ms` is a
setup proxy (copy/tree/wrapper), NOT directly isolated KDtree instrumentation.
Record every call separately, plus actual query->Top3 wall and process RSS.

Stop after this fixed retry regardless of convergence. No DCReg parameter
trials, performance optimization, new descriptors, Dual-U or online fusion.
Accuracy is not evaluated, and an approximate diagnostic cannot certify an
exact candidate interface. Recommend continuation/pause based on actual
convergence and cost, with limits stated.
