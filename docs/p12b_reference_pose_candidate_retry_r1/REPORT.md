# P12B — reference orientation retry

FINAL_RESULT: **REFERENCE_POSE_RETRY_PARTIAL**.
NEXT (one only): **PAUSE_DCREG_MULTICANDIDATE_ONLINE_VALIDATION**.

The bounded retry executes successfully, but is expressly an approximate
reference-state-pose diagnostic, not an exact LiDAR-pose conversion. Author
convergence rises from3/18 to4/18; all8 far hypotheses still fail. This is not
a broad recovery or a localization-accuracy improvement. Stop here; no solver
tuning, new samples, additional stages or performance optimization.

## Git and scope

START_SHA / verified remote before work:
`7a3afb0209f55b7c5246dd1a377b10607685577c`.
CODE_SHA: `0e2ab13e8eccd7d1f72797e93fd2cf7a81dbfd2c`.
Branch: `research/p9-r4-heldout-visual-evidence`.
Safe workspace: `/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`.
PUSH_EXECUTED=NO. END_SHA is the subsequent archive commit at handoff; a local
commit is not remote delivery. User already uploaded historical P12 and its
remote SHA was directly verified. No main dirty/stable robot workspace changes.

P12 coarsePosition and archived results remain unchanged. The added
`approximateReferenceStatePose()` and standalone `reference_pose_retry` use
the existing P12 helpers and unmodified external author solver. CMake adds two
targets; two own Python utilities freeze/evaluate the retry. The evaluator's
later archive-copy addition does not alter the compiled experimental binary.
No DCReg/SOLiD/NDT/EKF/OOSM mathematics, thresholds or data are changed.

## Coordinate finding

`map_T_reference` stores IMU/body-origin FAST-LIVO2 mapping state at lio_time.
The logger and pointBodyToWorld prove the direction:

```text
p_map = R_map_imu * (R_imu_lidar * p_lidar + t_imu_lidar) + p_map_imu
T_map_lidar = T_map_imu * T_imu_lidar
```

Current `ours_config.yaml` has R_imu_lidar=I and
t_imu_lidar=(-.011,-.02329,.04412)m. Its hash is recorded, but its use for the
active Loop2 mapping run is NOT established. This current configuration is
not a certified historical physical calibration and is NOT applied here.

The reference builder copies the mapping state and overwrites timestamp with
closest raw-scan header, without pose transport. The14 distinct references in
this subset exactly copy the active mapping log's XYZ AND quaternion. The
pose's state clock and scan header differ: mean absolute24.027ms,
max48.164ms (subset, not all389 references). No scan deskew or exact pose-time
transport is invented. Historical deskew=false and midpoint query association
are preserved. There is no proof of an inverse-direction bug; the proven
interface limitation is using a query-dependent orientation at a different
reference place, with origin/time approximations remaining unresolved.

Exact conversion: **NOT_CLOSED**, exact conversions=0. B is
**APPROXIMATE_REFERENCE_STATE_POSE**, not precise map_T_lidar. This fulfills
only the user-permitted approximate diagnostic, not the exact-interface target.

```cpp
// A: original historical path, unchanged
A = p12::coarsePosition(reference.map_T_reference, query_ndt_initial);
// B: keep reference state position AND orientation; approximate, no extrinsic guess
B = p12::approximateReferenceStatePose(reference.map_T_reference);
test_case.initial_pose = dcreg::MatrixToPose(B);
test_case.has_ground_truth_pose = false;
result = dcreg::RunRegistration(*source, *target, test_case, options);
```

Source is not pretransformed; terminal output already is source-to-map and is
not composed with the initialization again. Rotation change is SO3 geodesic
angle; full reference quaternion is used, not a guessed yaw.

## Frozen real comparison

First two chronological frames in each allowed historical category, excluding
already selected frames: previously converged; only failed with a <=5m
reference; only failed with a >5m reference. Exact selection/ranks are frozen
before B. All Top3 of each selected frame are retained. No GT/sample replacement.

|Frame|Reference IDs (rank1/2/3)|A converged|B converged|A -> B rotation changes, deg|
|---|---|---|---|---|
|16|0 / 1 / 163|2/3|2/3|7.694 /10.197 /100.510|
|45|0 / 1 / 2|0/3|0/3|7.793 /10.293 /5.271|
|124|0 / 1 / 157|1/3|2/3|5.993 /7.515 /91.438|
|708|172 /242 /319|0/3|0/3|8.391 /174.912 /147.911|
|1285|352 /180 /53|0/3|0/3|176.617 /62.710 /9.285|
|1891|90 /93 /92|0/3|0/3|32.042 /27.530 /31.019|

Six scans,18 new candidate registrations, no local rerun. A=18 immutable prior
P12 calls, not rerun with a new initialization or selectively rescored.
Near<=5m:3/10 ->4/10; far>5m:0/8 ->0/8. Newly converged1, lost convergence0.
Frames with any convergence remain2/6 ->2/6. Convergence fractions are
16.67% ->22.22%, NOT place-recognition or localization correctness rates.

Only newly converged case: TX124 / reference0, initial rotation changed5.993deg,
iterations30 ->28. Native last-pre-update plane residual .0630266 ->.0628718m;
correspondence fraction stays .8456325. Terminal poses differ only
**0.000306738m /0.002783627deg**. Thus this is a stopping-status improvement
near the old terminal, not demonstrated recovery into a new correct position.
No independent labels or GT were used. "新增有效候选" if meaning independently
verified locations remains unverified; only one additional solver-converged
initialization is observed.

B18/18 finite rigid poses,0 nonfinite,14 max30 iteration failures. Maximum
motion from B initialization1.156565m /16.532858deg; no >3m/>45deg diagnostic
excursion. Legal SE3 and no large correction do not exclude a wrong place.
Rotation changes mean50.951deg, range5.271..176.617deg. Translation unchanged
to numerical roundoff. No NDT/feedback calls and GT_LOADED=NO.

## Same inputs and unchanged author settings

DCReg pin `8ce8451b15491a4bbe17cf85ab02a8bed6696861`; clean external source.
DCReg / SO3 / Serial / verbose=false; all SolverParameters defaults unchanged:
radius1m,max30,translation1e-3m,rotation1e-5rad. No parameter changes or retries.
SOLiD pin `9a4f2895e5211f69fbbd6d4732577e1a67e9b882`, unchanged P11 library.
Same389-entry DB and active Loop2 map. Live Top3 IDs/similarities/reference
poses and A coarse initialization all reproduce18/18 frozen records.
Sources6/6 exact count/content-hash; same target459154/FNV ea1e4e69cf9992c6.
No new extraction, no new raw bag or map. Author license remains unconfirmed;
only own wrapper/tests/results are in Git, not original solver or binaries.

## Cost, directly measured this retry

|B stage, ms|mean|P95|max|
|---|---:|---:|---:|
|One complete DCReg API call|151.111|194.916|196.771|
|Author iteration timer|91.386|134.716|138.599|
|API setup proxy (copy/tree/wrapper)|59.725|67.688|73.062|
|SOLiD descriptor|1.809|7.701|10.202|
|SOLiD query/search|.111|.200|.238|
|Actual query -> Top3 calls + logging|455.466|545.603|572.368|
|Complete scan incl. source preparation|456.331|546.497|573.258|

Each of18 calls logs its actual setup proxy. Author internal timer starts
after target.makeShared() + KD-tree setInputCloud. The difference is NOT a
direct isolated KD-tree time and includes API wrapper overhead. No author
instrumentation or solver optimization is introduced. One cached target hash
avoids repeated archive instrumentation in the per-frame timer.

Process wall2.82s including startup, user CPU2.61s, system CPU.19s.
Peak64540KiB =63.027MiB. A historical call mean139.419ms/max157.050ms is
separately retained; no same-run performance advantage claim is made.
No warmup scan or excluded timing outlier was added; first descriptor10.202ms
is included. Offline CPU cost is not complete ROS/EKF online cost.

## Verification and stop

Release build PASS; CTest2/2 PASS (historical synthetic original-author
registration plus new reference-pose/NaN/SE3/direction tests). Freeze script
checks map/DB hashes and immutable expected selection. Real run verifies all
source/target/query-to-initial associations. CSV/JSON/hash audit and
git diff --check PASS. A bounded independent read-only review found the
invalid-result stats gap; finite-only geometry statistics now retain explicit
excluded counts/nulls, with strict timing stats unchanged. Regression tests
for mixed/all-NaN geometry and rejected invalid timing passed. No external
model CLI ran; no extra real registration was consumed for review.

**Recommendation:** pause DCReg multi-candidate ONLINE validation. A few local
registrations run, but changing reference rotation alone does not recover any
far hypothesis; three-candidate verification costs455ms mean. This is not
proof that DCReg generally fails, because true candidate calibration/time and
place correctness remain uncertified. Keep the connected offline probe and
full negative results; do not extend parameters or online feedback this round.

ACCURACY_GAIN_NOT_EVALUATED. This is interface evidence only, not paper
accuracy/ambiguity or global relocalization evidence. Completed fixed experiment;
wait for the decision AI rather than start another stage automatically.
