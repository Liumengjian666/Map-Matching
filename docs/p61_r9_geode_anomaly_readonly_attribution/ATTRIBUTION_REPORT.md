# P61-R9 resumed bounded attribution

## Result and execution identity

**MIXED_OR_INCONCLUSIVE**. Prediction-to-map geometric support collapses, Raw
candidates become motion-inconsistent, and the previous-Used step limiter changes
which geometry reaches fusion. Their absolute correctness and causal ordering
cannot be uniquely identified from this closed-loop, no-GT record.

This continuation preserves the genuine earlier asset-contract stop in REPORT.md
and commit `aea49024c5da5294b716bc17247713c4fca7b833`. The user explicitly corrected
the malformed61-character SHA before any resumed attribution.

- Repository: https://github.com/Liumengjian666/Map-Matching
- Worktree: `/home/jian/livox_ws/p61_r9_geode_anomaly_audit_ws`
- Branch: `research/p61-r9-geode-anomaly-readonly-attribution`
- Original START: `eba814000a06a2b110acf3835247cd6cffe9db6b`; resumed START: `aea49024c5da5294b716bc17247713c4fca7b833`.
- Actual R8 algorithm: `17922f2b63dae1f2af34f353287728d50a312f73`.
- Actual R8 tools: `1367e1d9587d1f90a47bf6f44405239a7b65e4f6`; final R8 archive: `eba814000a06a2b110acf3835247cd6cffe9db6b`.
- Actual numerical R9 analysis source: `f1d4ad321b6c77d7305ed42f015f95fc77c59e1f`.
- No push executed. END is the eventual archival commit, not a self-referential value embedded in this file.

## Asset gate and strict scope

Before analysis125 SHA/size comparisons passed over95 unique absolute paths,
including all31 output_freeze.json raw files,28 archive_hashes files, the original
and prepared bags, map, initialization, covariance, R2 ledger, numerical config,
fullprecision YAML and both actual executables. See ASSET_GATE_RECHECK.md and
the complete before/after snapshots; presence alone was never accepted as identity.

Input semantic checks independently matched all2441 admitted scans2986–5426
to R2 raw headers/offset extrema/serialized SHA. Boundary2985 and5427 remain
excluded. Prepared input has24419 external IMU messages. These are input identity
checks, not a new full-sequence trajectory analysis.

Numerical scope:112 unique scans, union5035–5045 and5140–5240. The six approved
overlapping phases are unchanged. Only5034 and5139 are read as predecessors.
There is no selection by outcomes. Immutable analysis input manifest SHA:
`2bf69f3273559f28b7aca43853fa275b448734ec46b91429129fd3970c8ac71b`.

GT reads0; ROS localization nodes0; rosbag play0; new NDT align0; EKF runs0;
jet/Hessian calls0; Source reconstructions0; new/corrected trajectory estimates0.
Python rosbag was used solely to inspect112 frozen prepared raw messages offline.

## Definitions and evidence provenance

Every original R8 geometric number is copied without numeric alteration from
geometry.csv. Source counts in scope are868–1400, not uniformly1400. NN reference
is the actual twice0.15m-voxelized R8 Target318377points. Overlap@r is the count of
valid Source points with nearest Target distance **<=r**, divided by the valid NN
point count; r=.5,2,8m. The recorded median/P90 uses order statistic
`floor(fraction*(N-1))`, not an interpolated percentile. `gate_status=PASS` is not
treated as map coverage certification. Fitness is the recorded PCL result, not
absolute error or a substitute for native NDT objective value.

No runtime deskew Source clouds were declared frozen. We did not recreate them.
New density statistics use the original frozen321053point map in fixed2m/5m
balls around existing Initial/Raw/Used LiDAR origins, with5m XYZ spans, nearest
origin distance and AABB-distance. These origin statistics are neither full-FOV
coverage nor uniqueness tests; their pointset differs from recorded NN Target.

Transforms use T_A_B:B→A. For existing LiDAR poses,
Delta=T_previous^-1*T_current; E=Delta_prediction^-1*Delta_raw;
Log(E)=[rho metres,phi radians]. Translation residual is rho=J_left(phi)^-1*t,
solved stably; it is NOT the map-frame position difference. Translation and
rotation norms remain separate. Fused increments retain their actual IMU replay
watermark stamps, while Initial/Raw/Used correspond to scan end. Independent
SciPy matrix logarithms verify all112 relative residuals.

## Phase overview

Numbers below are phase medians; phases overlap and must not be summed as independent samples.

| Phase | n | Initial/Raw/Used overlap@.5 | rho m / phi deg | translation/rotation limited |
|---|---:|---|---|---|
| pre5140–5157 |18|.762500/.785000/.773571|.088157/.666997|5/7|
| transition5155–5165 |11|.232143/.539286/.234286|1.251294/15.119301|10/11|
| main5159–5229 |71|.268571/.289286/.268571|.238651/1.552787|35/23|
| severe5169–5175 |7|.044286/.093571/.041429|1.209157/15.864619|7/7|
| recovery5220–5240 |21|.481429/.500714/.500714|.047334/.233606|1/0|
| secondary5035–5045 |11|.505714/.510000/.510000|.084321/.289578|0/0|

The pre interval is **not wholly healthy**: rotation limiting already occurs at
5150/5152, both limits at5153–5157, and pre maximum motion residual is
1.895805m/26.537267deg. This prevents claiming5159 is the first causal fault.

The71-frame main interval really has Used overlap<.5 in all71 records. Its first
header is1706671805199903249ns and final end1706671812299679066ns. Severe
5169–5175 spans header1706671806200306415 to end1706671806899588626ns.
The earlier rounded severe time summary did not describe precisely these same
boundary scans; frame identities here are determined from frozen records.

## Formation and limiter interaction

| scan | Initial / Raw / Used overlap@.5 | Raw correction from Initial m / deg | Raw–prediction motion rho m / phi deg |
|---|---|---|---|
|5158|.5621840243/.8645096057/.5561172902|2.336165464/32.473985421|.500821987/6.791800448|
|5159|.3971176942/.8158526821/.3610888711|2.777343772/37.413231104|.636487/7.392108|
|5160|.2321428571/.5392857143/.2342857143|2.514440063/33.271677931|1.608981/19.815569|
|5161|.1350000000/.1528571429/.1421428571|.807092756/10.186818944|2.956394186/39.821544554|

At5159 Raw median NN=.170803752m, versus Initial=.835797664m and
Used=.893204150m. Raw improves over Used by45.476381105percentage points.
At5158/5160 the improvements are30.839231547/30.500000000points.
These are direct geometric facts, not certification that Raw is correct.

`dog_prior_map_ndt_node.cpp:2692–2725` uses previous_pose_ (previous **Used**), not
current prediction, as the limiter reference. Translation cap=.5m, rotation
cap=5deg; translation is truncated along previous→Raw and rotation is quaternion
slerp. Call at:2562; subsequent previous_pose_=used_result feeds the next step.
No runtime parameters or poses were changed in R9. Recorded step magnitudes
match this algebra to6.11733e-14m and1.52830e-6deg. Axis/collinearity checks are in
anomaly_frame_table.csv; no replacement Used trajectory was constructed.

Raw increment5158 is1.019625m/10.345120deg, versus prediction
.595460m/4.044942deg. At5161 these become2.429552m/35.073139deg versus
.558815m/4.878519deg. Raw candidates therefore also carry significant motion
reliability risks. At5172 relative residual reaches3.984377664m/53.606305723deg.
Prediction itself is closed-loop and influenced by previous Used measurements;
it is not independent motion truth. Across112 scans Raw overlap exceeds Used
on33, is lower on8 and ties on71. Limiting can also protect against poorer Raw
geometry. No counterfactual feedback run exists to settle the tradeoff.

## Severe stage and map evidence

5171 header1706671806399974584, scan end1706671806499800629ns:
fitness393.55775097905757; iterations40; nativeconverged=true. Initial/Raw/Used
NN median=9.121137664/9.263498303/9.156141678m; UsedP90=33.380730869m;
overlap@.5=.0414285714/.0435714286/.0407142857. Reaching40 iterations and
reporting converged is not proof of a good match. Full2m/8m numbers remain
verbatim in the per-frame CSV, not inferred from.5m.

At5171 origin-centred map counts2m/5m are Initial208/6005, Raw369/5653,
Used204/6008. Severe Used5m counts span5185–6029. These positions lie inside the
global map AABB. This argues against a gross absence of nearby map points, but
does not certify that all Source rays/structures are mapped or distinguish a
wrong attitude, displaced position, missing distant structure, or repetition.

Secondary5035–5045 has **lower**5m density: Used2005–2339points, median2171,
but no limiting and maximum motion residual.371671m/1.614780deg. Thus a low
origin density alone is not an explanation of the severe anomaly. In recovery,
Used overlap rises from.486428571 at5229 to.500714286 at5230 and.810000 at5240;
the corresponding5m density is lower than severe. This is recovery of geometric
support, not verified true-position recovery.

## Time and coordinate audit

All112 raw adapted message byte hashes, header/frame and uint32 offsets match
the frozen scan manifest. t_point=header_ns+offset_ns, exact integer scanend
is independently checked. Actual runtime C++ is pinned to17922f2, not the
post-run recorder. `imu_deskew_runtime.cpp:142–163,324–331` uses original offsets;
:289–292 sets output reference header=end; :388 writes output float time relative
to reference. `core/imu_deskew.cpp:351–354` applies reference-pose inverse to
point-time LiDAR pose. These checks found no cross-frame offset/timebase misuse.

**Confirmed diagnostic-only defect:** :503–505 passes original input extrema to
CSV, while :549–550 labels them `seconds_from_output_reference`. All112 rows
are mislabeled; they are seconds from the original header, not from output end.
For5171 input range0..0.099826045s must not be interpreted as post-end points.
Actual output time would be -0.099826045..0s. We did not rewrite frozen CSVs or
production C++. The copied offset_time field also remains original while header
changes; this is a metadata hazard for another consumer, not proof that the
actual NDT path (internaldeskew=false) used it incorrectly.

5171 last input IMU at/before end1706671806492429971ns; actual bridge restores
that head, advances to native scanend1706671806.4998007s using the real held
head input, applies correction there, and replays1 true tail IMU to
1706671806502695083ns (native1706671806.5026951s). Next prediction end is
1706671806599643137ns. `fusion/ndt_observation.cpp:273–281` is the already-run
R7 time fix. It is a held-input propagation approximation, not a new future
sample or interpolation invented in R9. All112 recorded time checks pass;
actual native scanend representation error at5171 is53.067871094ns, not a
physical measurement delay. No illegal future head or incomplete replay was
found in these records. OOSM APPLIED is not treated as a correctness guarantee.

Coordinate chain is T_M_L=T_M_I*T_I_L, T_M_I=T_M_L*inverse(T_I_L), established
by `fusion/ndt_observation.cpp:217`, `imu_deskew_runtime.cpp:490`, inline
`core/frame_conversions.hpp`, and `ros_output.cpp:19`. Source inputframe is
livox_frame, input IMU imu_link, configured map camera_init. Actual published
output frame_id headers were not retained in R8 CSV: source/config agreement
is available, independent output-message frame certification is NOT_AVAILABLE.

Maximum numerical roundtrip errors5.695433295e-14m/5.562697819e-15deg;
full homogeneous Frobenius error5.895443414e-11. T_I_L rotation determinant
1.0000000000416869; orthogonality error5.895427338e-11. Used→IMU measurements
match actual correction records to4.850592728e-13m/2.476088454e-11deg;
Initial matches rollback LiDAR to6.394884622e-14m/2.251244448e-11deg;
fused LiDAR matches replayed IMU*T_I_L to5.684341886e-14m/2.248105202e-11deg.
This tests arithmetic/recorded semantics, not independent physical extrinsic
accuracy. The update is fixed-ratio pose correction and velocity blending,
not a claimed standard Kalman measurement update.

## H1–H5 assessment

| Hypothesis | Status | Evidence strength and limitation |
|---|---|---|
|H1 prediction loses map support|CONFIRMED|Direct112-frame records; no claim of true-position drift|
|H2 Raw candidate reliability risk|SUPPORTED|Large corrections and motion disagreement; no proof of wrong/nonlocal basin|
|H3 limiter interaction|SUPPORTED|Measured clipping and support loss confirmed; benefit/harm to true pose unresolved|
|H4 missing local map coverage|NOT_EVALUABLE|Nearby density is substantial; full-FOV structure/true pose unavailable|
|H5 actual time/coordinate violation|NOT_SUPPORTED|No violation in112 audited records; diagnostic label defect confirmed separately|

The machine-readable cause_assessment.json contains evidence, counterevidence,
scope, code points, timestamps and unanswered questions for each hypothesis.
No confidence percentages are invented. No Hessian or GT inference is added.

## Unique NEXT proposal — not executed

Ask research control to approve **one bounded independent angular-increment
check using frozen external Xsens gyro over5155–5165**, with the existing
timestamp/extrinsic/bias contract, no NDT/EKF/GT and no parameter adjustment.
This resolves whether Raw's large rotations are supported by a motion signal
that is not the already-corrected prediction trajectory, before considering any
limiter change. It cannot validate translation or absolute pose.

Proposed preregistration, for approval only: all11 intervals must have valid IMU
coverage and a traceable causal bias; otherwise NOT_EVALUABLE. Raw-to-integrated
angular discrepancy P95<=2deg and max<=5deg gives limited angular consistency
support; >=2 consecutive intervals>5deg contradicts that consistency; remaining
outcomes are inconclusive. These thresholds were **not applied this round** and
are not new production gates. Do not loosen the limiter based only on overlap.

## Verification and residual limits

Seven unit/integration tests passed, including all112 residuals against independent
matrixlog, verbatim geometry preservation, bounds, timestamps and scope. Fresh
adversarial review identified the fused-watermark label issue before archival;
it was corrected without changing any R8 pose. Final asset snapshots and
analysis_manifest/checksums document before/after identity and script SHA.
No new dependencies or production code changes. Runtime source/Target overlap,
independent output frame headers, physical calibration truth, counterfactual
feedback and absolute localization correctness remain unavailable or untested.
