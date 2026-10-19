# P61-R10 fixed independent gyro increment check

Frozen base: d1f9fa9464dac835598e72d52ea2484fd9aa801c. R9 local/remote HEAD
were both this full SHA at the actual start; R9 was clean. New worktree/branch:
`/home/jian/livox_ws/p61_r10_geode_gyro_increment_audit_ws`,
`research/p61-r10-geode-gyro-increment-audit`. No R9 or runtime asset edits.

## Fixed inputs and scope (before gyro inspection/results)

Eleven intervals ending at scans5155..5165, with5154 only the predecessor.
Endpoints are exact integer scan_end_ns in frozen scan_manifest.csv, not CSV
double seconds. Raw poses are R8 raw_ndt_pose only. Fixed T_I_L is read from the
R8 fullprecision initialization YAML and checked against R6 initialization.json.
R_M_I=R_M_L*inverse(R_I_L), evaluated by a stable right-division solve,
subject to explicit orthogonality/determinant checks. Frozen coefficients are
not replaced with a newly orthogonalized calibration.

Bias is the fullprecision R6 initial_state.gyro_bias_I in rad/s, IMU axes:
[-0.0008725237314739192,0.0023757483405765214,-0.010976615256704694].
Rounding to9 decimals exactly matches the task's displayed vector
[-0.000872524,0.002375748,-0.010976615]. No alternate/fitted bias is used.
If these displayed digits, units, source or axes fail verification, stop.
The R6 bias predates this window and is not a guarantee of no later drift.

Use only /imu/data sensor_msgs/Imu, external Xsens imu_link. Each raw serialized
message hash must match frozen input/imu.csv. No IMU orientation/acceleration,
future bias, EKF prediction or fused pose may contribute to the integral.

## Numerical definitions

Body angular velocity obeys dR_M_I/dt=R_M_I*[omega_I]_x. Thus right-multiply
chronologically: Delta_R=Exp((w0-bg)*dt0)*Exp((w1-bg)*dt1)*... . Piecewise ZOH
uses the last real sample at or before the start and holds each sample only to
its successor or interval end. Do not interpolate with an after-end sample.
Integer nanosecond subtraction precedes conversion to seconds.

Median IMU dt is the median of positive adjacent actual IMU spacings whose
timestamps both lie inside the total target time window. The before-start head
is excluded from that median but included in gap validity. Gap validity checks
the full spacing of every in-window predecessor/successor pair and the age of
the held head at start/end, all <=3*median_dt. No future sample is read to test
the final truncated hold. Nonmonotonic/duplicate/nonfinite input is invalid,
not silently sorted, deduplicated, interpolated or clipped. Preserve invalid rows.

Delta_R_raw=R_M_I(prev)^T*R_M_I(curr).
E=Delta_R_gyro^T*Delta_R_raw; residual=norm(Log_SO3(E))*180/pi.
Check rotations and clip trace only within floating tolerance; stable quaternion
log handles near0 and nearpi. Independently verify with scipy.linalg.logm.
Do not compute an absolute trajectory or a combined translation/rotation norm.

## Unchanged formal decision rules

- NOT_EVALUABLE if any of11 intervals has invalid coverage/time/bias/coordinates,
  input mismatch, or unresolved numerical/contract error.
- LIMITED_ANGULAR_CONSISTENCY_SUPPORTED if all11 valid, residual median<=2deg
  and maximum<=5deg, with no unresolved numerical/transform problem.
- ANGULAR_CONSISTENCY_CONTRADICTED if all11 valid and at least two consecutive
  residuals>5deg.
- Otherwise MIXED_OR_INCONCLUSIVE; acknowledge fixed-bias uncertainty.

P95 is descriptive only, NumPy percentile(method='linear') / default linear
interpolation between sorted order statistics. It is NOT a formal decision gate.
No post-result threshold, bias, interval, integration-rule or extrinsic changes.

## Execution gates

First rehash all inherited R8 assets and R9 archive files. Freeze input manifest
and11 exact intervals. Write/run synthetic tests and independent review before
real calculation. Unit test failures stop formal decision until the actual
implementation fault is fixed and retained in the execution record. Afterward
rehash the same assets and original raw pose bytes. No GT, ROS node, rosbag play,
NDT align, EKF, new localization trajectory, jet, Hessian, optimizer or HIKVISION.

Results cannot certify absolute orientation, translation, correct basin, or
justify accepting Raw/relaxing a limiter. This is diagnostics, not innovation.
