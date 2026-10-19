# P61-R10 Independent Xsens angular increment check

## Formal result

**B — ANGULAR_CONSISTENCY_CONTRADICTED**, under the fixed R6 bias, frozen
extrinsic and recorded ROS-time contract. All11 intervals are valid. Residual
median14.433498508587023deg, maximum44.83768695312652deg;9/11 exceed5deg, with
consecutive blocks ending5155–5157 and5159–5164. The preregistered B rule is met.
P95=37.46486516242033deg is descriptive linear interpolation only, not a gate.

This is evidence that some Raw NDT adjacent rotations lack independent gyro
increment support in this window. It is **not** proof of a wrong local minimum,
absolute orientation/translation error, or that a limiter should be relaxed.
Physical clock offset, later gyro-bias drift and physical extrinsic accuracy
are not independently certified here. No innovation claim is made.

## Git and actual execution identity

Repository: https://github.com/Liumengjian666/Map-Matching

Worktree: `/home/jian/livox_ws/p61_r10_geode_gyro_increment_audit_ws`

Branch: `research/p61-r10-geode-gyro-increment-audit`.

START_SHA: `d1f9fa9464dac835598e72d52ea2484fd9aa801c`.
R9 local and remote actually both equaled this full SHA at the new task start;
R9 was clean. Earlier R9 receipt remote information was historical, not reused.

Actual numerical CODE_SHA:
`da24a7ef962f02f9478d64cf612f6792dab202b0`.

Actual run_audit.py SHA:
`ab65d0522a58de5ad41f274d305098ac62c4ec97b6630833ccbd28ad1261236d`.

Earlier code checkpoint `dbfa813a552e2a7bc26a1890b323ecceeefc265b` was examined
by the CLI package before final hardening; do not call it the numerical version.
R8 actual algorithm17922f2b63dae1f2af34f353287728d50a312f73 and tools
1367e1d9587d1f90a47bf6f44405239a7b65e4f6 were not executed.
The eventual END commit archives results; it is not self-embedded as this file's
own identity. No push executed; final remote status is in the execution receipt.

## Assets and provenance

Before and after156 identity comparisons passed across126 unique absolute
paths. This includes all inherited R8 declared inputs/config/binaries/output
files and every R9 tracked archival file against its immutable d1f9 Git object.
File sizes/digests/absolute paths are individually recorded in both asset gates;
all original assets and12 Raw pose records remain byte-identical.

| Key asset | Bytes | SHA256 |
|---|---:|---|
|Original bag /home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag|4500527001|162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091|
|R8 /home/jian/livox_ws/p61_r8_geode_runs/input/full_return.bag|1416133574|a138b5ceb0e8ee4a2e5203e70b3943596811ff15d7a45495909934d3a1494331|
|R8 input_manifest.json|3759|4444883c441f0c3c08650bf629c05812d34b1a069c7102d565053a47b976c76d|
|R5 frozen_map.pcd, identity only|10273947|4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4|
|R6 initialization.json|3352|9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d|
|R6 initial_covariance.csv, identity only|7352|55ac110fd02620c494e916fdb575bd514b734245b26d09764fc7de1116670f39|
|R8 fullprecision input/initialization.yaml|1463|084a1f4b04265c82b8fcf9e519bc35708692e5b409eb9c0fcb551955310ac6fa|

Numerical configuration SHA is
9d2e32ebf5bd89230d46b7af7b5d0cf9115cbe72d600634899337eab09eba0d3.
Exact scoped input pin SHA:
a2f3b61260e727addc935fff38cdfee587a88ba806d4a3992e094af9bb8af690.

The task-mentioned time_coordinate_audit.csv is the actual frozen R9 artifact,
not an invented R8 file. R8 lineage/fusion/live trajectories were hashed as
assets, not used as independent gyro truth. The main numerical inputs are
Raw poses, exact scan ends, fixed R6 bias/T_I_L, and original external IMU.

## Sensor, time, bias and coordinate contracts

Actual topic /imu/data; type sensor_msgs/Imu; frame imu_link; angular_velocity
xyz in rad/s per message schema and unchanged runtime usage. Original message
header and bag record use the same ROS epoch ns. No Livox internal IMU, device
timebase, IMU orientation, acceleration, prediction or future bias contributes.

The exact11 intervals span1706671804799723787–1706671805899593546ns, duration
1.099869759s.5154 is solely the first predecessor. Each interval's integer
scanend equals its Raw measurement timestamp, including the separately checked
5154 endpoint. Exact endpoints and sample limits are in the CSVs; rounded double
seconds were never substituted. No temporal window was changed after results.

111 unique external messages were decoded from1706671804791549444 to
1706671805892444849ns. The byte-binding pass re-inspected the SAME111 messages,
not extra samples outside the window. Every serialized SHA matches R8 imu.csv;
original ROS byte header/frame/sequence and exact gyro doubles also match the
decoded JSON before integration. The original decoded JSON was not replaced.

All11 intervals use11 sample records each, counting shared boundary heads again:
these are121 interval assignments, not121 independent sensor messages. Median
positive within-window IMU spacing10.402680ms; maximum allowed31.208040ms.
Observed maximum10.973930ms. All gaps/head/tail ages pass; no after-end sample
is used. Integration_details.json records every positive-duration hold segment.

Fixed R6 gyro_bias_I, exactly matching R8 fullprecision YAML:
[-0.0008725237314739192,0.0023757483405765214,-0.010976615256704694]rad/s.
Rounded9decimals matches the task vector. This bias is216.969883664s old at the
window start; no drift estimate is fit. It predates and is independent of these
Raw candidates, but no claim of constant physical bias for200+s is justified.

T_I_L direction L→I is checked by actual runtime frame_conversions.hpp:9–18 and
fusion/ndt_observation.cpp:217. T_M_I=T_M_L*inverse(T_I_L). The full matrix and
source/units are in coordinate_and_bias_audit.md. Rotation determinant
1.0000000000416869, orthogonality error5.895427338e-11. Frozen coefficients were
not recalibrated, optimized or replaced. No GT transform is fitted.

## Mathematics and independent verification

dR_M_I/dt=R_M_I*[omega_I]x, as in actual core/imu_propagation.cpp:50–55.
For each interval use the nearest true sample<=start, then chronological ZOH:

G=product_j Exp((gyro_j-bias)*delta_t_j), right-multiplied.
D=R_M_I(previous)^T*R_M_I(current).
E=G^T*D; residual=norm(Log_SO3(E))*180/pi.

The last sample is held only to end; no post-end interpolation. Nanosecond
subtraction precedes seconds conversion. SO3 validity/trace bounds are checked;
stable quaternion atan2 log handles nearzero and nearpi. Independent matrixlog
residuals agree within7.439169280587521e-10deg. Independent scipy.linalg.expm
products reproduce every persisted gyro increment to1e-13 matrix tolerance.
No Euler-axis summation or mixed coordinate residual is used.

Raw relative angles, gyro relative angles and their residual are distinct:
the residual may exceed either angle when axes/directions disagree. Raw-to-Initial
correction is a fourth, separately sourced quantity, NOT an adjacent increment.

## All preregistered intervals (degrees)

| interval | Raw adjacent | gyro adjacent | residual | Raw vs Initial correction | maxgap ms |
|---|---:|---:|---:|---:|---:|
|5154→5155|21.151843973|12.781078823|30.092043372|32.046410086|10.601044|
|5155→5156|18.675357357|12.517921027|14.433498509|23.295682159|10.487795|
|5156→5157|14.428318813|10.313856153|10.229935373|26.907413270|10.780334|
|5157→5158|10.345119973|9.089911038|3.802016278|32.473985421|10.575056|
|5158→5159|11.337250611|9.555640206|6.009088724|37.413231104|10.519266|
|5159→5160|18.846094291|10.769363805|20.499559441|33.271677931|10.468483|
|5160→5161|35.073138622|10.716635799|44.837686953|10.186818944|10.973930|
|5161→5162|5.746687934|9.094400180|7.954079759|2.187386919|10.767937|
|5162→5163|13.831385751|8.125133494|16.559455068|15.404660603|10.517597|
|5163→5164|10.554857663|7.830744171|18.345997632|19.090833538|10.515452|
|5164→5165|3.849383201|6.274385406|4.837111771|12.017540619|10.812045|

Each interval has VALID coverage,11samples and no future sample use. Most
important counterexamples:5157→5158 residual3.802016deg and5164→5165
4.837112deg do NOT exceed5deg. Thus the conclusion is not that every Raw
increment is unsupported.5158's32.473985deg correction from Initial must not
be confused with its10.345120deg adjacent Raw rotation or3.802016deg residual.

## Decision and limits

RuleA fails: median>2deg and max>5deg. RuleB passes: all11 valid and consecutive
exceedances, including5158→5159 /5159→5160 and5159→5160 /5160→5161.
Nine exceedances form two runs, not eleven independent scenes. No P95 decision
gate or threshold/bias/interval tuning was introduced. This is a conditional
motion inconsistency result, not a gyro-derived absolute pose or test GT.

No wrong NDT minimum, wrong absolute position, or cause of a final trajectory
error is established. The recorded physical timestamp/extrinsic assumptions
may still have systematic errors not visible to arithmetic tests. Bias drift
and ZOH approximation remain interpretation limits, especially for small errors.
No alternate integration rule/offset/bias was tried to improve agreement.

## Review, tests and resource scope

Two local fresh-context review rounds plus ONE user-confirmed external CLI
review completed. External CLI exit0 found value-binding and5154-time gaps,
repaired and regression-tested before real calculation. See
EXTERNAL_REVIEW_RESULT.md and REVIEW_AND_TESTS.md; this is not a claim that the
external reviewer reviewed the final numeric results. No fourth review/CLI run.

21 tests pass. This includes every task-required synthetic case, the explicit
tampering and shifted5154 counterexamples, exact11-index identity, frozen asset
preservation and independent expm reconstruction. Historical failed test evidence
is retained. No production build is claimed or needed: no production code changed.

Numerical audit wall6.130110014s includes streaming rehashes, file reads and pure
math; it is NOT an online localization latency. Peak RSS/CPU were not measured,
so no memory/realtime claim is made. No new library/dependency was installed.

## Boundary counters and unique NEXT

GT reads0; ROS localization nodes0; rosbag play0; new NDT align0; EKF runs0;
new localization trajectory estimates0; jet/Hessian0; Coupled/Recovery0.
All existing poses, maps, initialization and source/config assets are unchanged.
Only interval rotations and derived diagnostics were generated.

**Unique NEXT proposal, not executed:** ask research control for a bounded,
independent evidence check of physical LiDAR–Xsens clock synchronization and
R6-bias stability for this SAME window before attributing the discrepancy solely
to Raw NDT. Do not fit shifts/bias to NDT, extend scope, accept Raw or loosen
the limiter. If evidence is insufficient, retain those alternatives unresolved.
No full localization, new candidate or new algorithm starts automatically.
