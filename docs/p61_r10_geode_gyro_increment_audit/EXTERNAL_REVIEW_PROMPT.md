# One bounded adversarial read-only review

Review the artifact, not the author's conclusions. Assume the author is
overconfident. Find Required mathematical/implementation/contract failures,
or explicitly state none after examination. Do not approve by summary.

The user authorized requesting a second opinion. Before reviewing, verify
the SHA256 of the archive identified by the outer CLI request against its exact
provided digest. Read only that outer request, this prompt
and files inside that archive using read-only tar listing/extraction-to-stdout.
Do not extract files onto disk. Do not follow asset paths in input manifests.
No GT, sensor bag/PCD access, nodes, playback, NDT/EKF, network browsing,
subagents, code execution, new tests, file edits, CLI follow-on, or optimization.
This is pre-execution review; no real gyro residual has been computed.

ARTIFACT: the enclosed PRE_REGISTRATION.md, inputs_frozen.json, gyro_math.py,
extract_gyro.py, run_audit.py, check_assets.py, freeze_inputs.py and tests.

CONTRACT:
11 exact adjacent scan-end intervals5154->5155 through5164->5165;
only frozen Raw NDT rotations converted L->I with fixed T_I_L inverse.
External /imu/data Xsens sensor_msgs/Imu angular_velocity in rad/s, imu_link;
per-message serialized SHA must equal frozen input ledger.
R6 fullprecision gyro bias fixed; displayed9decimals must match task digits.
ZOH causal real head<=start; right-multiply Exp((w-bg)*dt) in body axes;
no sample after that interval end; no future interpolation or gyro fitting.
Integers for endpoint subtraction. Monotonic finite IMU, gap<=3*median positive
actual target-window IMU dt, held head/tail age bounded too.
Residual normLog(Gyro.T*Raw) degrees; checkSO3 near0/pi, independent matrixlog.
All11 valid + median<=2deg/max<=5deg supports limited consistency;
all11valid + at least2consecutive>5deg contradicts; invalid isNOT_EVALUABLE;
other outcomes mixed. P95 descriptive linear only, not a formal decision gate.
No new localization trajectory or changed runtime algorithm/threshold/asset.
No GT, pose search, align, filter run or new recording is authorized.

OUTPUT: Required findings first, with file/line and concrete counterexample.
Differentiate actual bug, contract ambiguity and documented limitation.
Do not infer absolute pose correctness, physical calibration truth or innovation.
