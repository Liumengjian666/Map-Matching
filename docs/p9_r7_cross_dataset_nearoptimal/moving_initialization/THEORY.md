# Causal moving initialization, not DUAL-U evidence

This is a startup-module engineering experiment. R4/R5/R6 and the previous
Corridor01 static-initialization failure remain unchanged. No GT, oracle search,
B12 search, visual extraction, or retrospective trajectory correction is allowed.

The configuration is frozen before evaluating any bootstrap registration or
state fit. `bootstrap_config.json` is the gate contract. Use the first ten
seconds only, ending at the latest legal scan end not after the ten-second
deadline. A first-five-second sensor-only anchor defines the normalized map
frame. Its historical first-frame pose is not a moving state at five seconds.

PCL GICP supplies bootstrap-only adjacent raw-scan relative pose observations.
These scans are not deskewed and their poses are noisy end-reference
approximations; this limitation must not be hidden as exact scan-end truth.
Position/rotation noise floors and independent 8--10 second validation prevent
automatic acceptance merely because a registration converged. No new deskew
implementation is introduced. Formal deskew, if reached, uses ScanEndProcessor.

For fixed `T_imu_lidar`, `T_map_imu = T_map_lidar * inverse(T_imu_lidar)`.
This translation changes the IMU-origin positions as well as the orientations.
IMU samples remain in their native Epson frame and are not rotated twice.

Estimate gyro bias from SO(3) relative pose/preintegration residuals. Estimate
per-knot velocities, gravity, and accelerometer bias jointly from the standard
preintegrated position and velocity relations. No velocity-zero or bias-zero
prior contributes information to the observability test. Use QR/SVD and examine
the nuisance-velocity-eliminated gravity/bias profile. Gravity norm is 9.809,
consistent with the actual IKFoM S2 state, but fixing its norm does not by itself
separate accelerometer bias from gravity direction.

The gravity-norm-constrained profile has five coordinates (two tangent gravity
directions and three accelerometer-bias directions). Numerical full rank alone
is insufficient: uncertainty and condition gates are also required. A nearly
singular profile cannot produce an accepted unique state or a deceptively small
covariance. The SVD profile uses fixed physical scales (gravity tangent
9.809*radians(5), bias 0.50 m/s^2), and estimates a conservative lower singular
value after orientation-uncertainty perturbations. Worst-correlation variance
inflation is the number of residual blocks; motion-derived scan distortion is
an engineering estimate, not a provable speed bound. These diagnostics may
reject a candidate but cannot alone authorize an accepted state. Acceptance
would additionally require a full coupled gyro-bias/motion covariance and
boot-time propagation with credible LiDAR uncertainty. All tentative solutions
must be marked UNACCEPTED if any required
state direction fails. Covariance is only injected after state acceptance.

The 5--8 second interval fits the state; the 8--10 second interval is held out
from fitting and tests motion prediction. Only after all gates pass may the
state be propagated to the frozen boot scan-end and passed to initializeMoving.
Formal transactions then start after that committed time. Bootstrap and
boundary-rejected scans remain in the ledger, not silently deleted.

Mature implementations inspected: FAST-LIO2 IMU initialization and IKFoM
propagation, LIO-SAM/SuperLoc GTSAM preintegration usage, the project's
sensor-only first-segment PCL initializer, and the existing P7 scan-end chain.
The available standard FAST-LIO2 and LIO-SAM initial states have stationary or
zero-velocity assumptions; those assumptions are not copied into this moving
initializer. The midpoint preintegration adapter implements their standard
kinematics, not a new full LIO/VIO optimization system.
