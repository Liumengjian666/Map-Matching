# P9 Corridor01 robust causal bootstrap V2

This is startup engineering, not a DUAL-U contribution or cross-dataset
ambiguity-evidence validation. The earlier R7-R4 result remains
`BOOTSTRAP_REGISTRATION_QUALITY_FAIL`; none of its artifacts are changed.

## Pre-run frozen protocol

`bootstrap_v2_config.json` and `quality_gate_freeze.json` bind inputs, map,
calibration, sensor-only anchor, numerical dependencies, source, Release binary,
environment and unique persistent output before real V2 GICP starts.
The original first-ten-second window contains 99 scans, ending at
1517157229072630478 ns. There are 98 registration opportunities; TX1 establishes
coordinates but is not an accepted relative-motion observation.

Raw timed scans are streamed, not loaded for the full sequence. Gyroscope
integration consumes only samples at or before the current scan end. The
declared interval limits are 20 ms between samples and 10 ms endpoint hold.
Unsupported TX1 points before the first observed IMU timestamp are omitted only
from the bootstrap cloud, with their count retained; no earlier IMU is invented.
The preserved raw inputs are not rewritten. Causal midpoint SO(3) rotations use
the frozen extrinsic by full SE(3) conjugation, including its lever arm:

`T_Lend_Lpoint = inverse(T_imu_lidar) * T_Iend_Ipoint * T_imu_lidar`.

Provisional gyro bias zero is **not an estimate**. The 0.05 rad/s norm scenario
records bias sensitivity, not a guaranteed physical bound. Translation for the
first two scans is explicitly not estimated. Subsequently, a constant-velocity
predictor may use only the last two accepted IMU-origin poses. Rejected or
prediction-only poses never enter this history or the submap.

GICP matches the current cloud to up to the latest three accepted local clouds,
using the causal IMU/past-velocity pose as its original initial guess. The
rotation/translation conventions are covered by synthetic SE(3) tests. PCL GICP
uses 0.25 m voxel, 0.5--80 m range, 2.5 m correspondence search, 40 iterations and
1e-6 transformation epsilon. These engineering settings were fixed before the
new real results.

The prefit support is all voxelized target points within 2.5 m of any current
point transformed by the **original** prediction. It is fixed before both
solvers and is not recropped after observing results. Forward overlap uses all
current points. Reverse overlap is conditional on this fixed target support,
not the entire full submap. Independent reverse GICP starts from the inverse
original guess; it is not the inverse forward result.

## Quality and trajectory gates

Both 0.50 m nearest-neighbor overlap ratios must be at least 0.55. In each
direction, the smallest ceil(70% of overlap correspondence count) distances
must have RMSE at most 0.25 m. The independently fitted forward/reverse
composition must have translation at most 0.10 m and rotation at most 1 degree.
Per-step translation must be below 3 m and rotation below 45 degrees. A
predeclared heuristic IMU rotation consistency guard is 5 degrees; it is not
statistically calibrated. Original untruncated PCL fitness is retained but is
not the sole gate.

The trajectory requires at least 90 accepted observations of 98 opportunities,
no more than two consecutive failures, all 5--8 s estimation targets genuinely
observed and at least 15 observations in 8 s--boot. The third consecutive
failure stops real registration; later ledger entries are `NOT_RUN`.
`execute_v2_once.py` claims a protocol-wide exclusive start receipt before any
alignment, including a failed run. No alternate output child or silent retry
can turn this into parameter exploration.

Only trajectory admission PASS permits the unchanged motion-fitting gates.
Only a certified complete coupled state covariance and independent validation
permit existing `initializeMoving`, official smoke NDT and formal Run A/B.
Marginal engineering uncertainty recorded for causal predictions is not an
initialization covariance. The motion adapter is explicitly non-accepting until
the full coupled covariance is supplied. It was not invoked in this run.

## Interpretation boundary

This run stopped after three attempts against the partial TX1 anchor. It did
not exercise an established three-frame submap or accepted-history velocity.
Small trimmed residual and inverse closure on a subset do not establish
sufficient full-current-cloud overlap. Conversely, startup overlap failure is
not proof of mathematical state unobservability or whole-sequence
registration infeasibility. Further dataset/protocol decisions belong to the
research controller; this execution does not authorize V3 or threshold tuning.

No GT, oracle263, B12, visual extraction, pose switching, R6 statistics or
DUAL-U/IKFoM innovation is used. The formal NDT and U_obs mathematics remain
unchanged and were not executed on real Corridor01 data in this task.
