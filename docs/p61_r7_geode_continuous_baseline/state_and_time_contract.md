# State/frame/time integration (not an algorithm replacement)

T_A_B maps coordinates from B into A. R5 M is camera_init; I is external
Xsens imu_link; L is livox_frame. The full-precision frozen R6 state supplies
T_M_I, v_M, b_g_I, b_a_I, g_M and fixed T_I_L. The production explicit-state
entry accepts these quantities directly; its velocity is at the IMU origin.

The 23D FAST-LIO covariance is neither reordered blindly nor truncated. Native
FAST-LIO's extrinsic and gravity tangent uncertainties/cross terms are not
part of the present 15D model. The existing fixed diagonal startup prior is
used: diag(.5² I_p, .5² I_v, (5*pi/180)² I_theta, .05² I_ba, .05² I_bg).
These are configured heuristic prior standard deviations, NOT data-estimated
uncertainty or a restoration of the R6 posterior. Extrinsics/gravity are fixed
in the 15D baseline. This task makes no uncertainty-consistency claim.

The actual split baseline propagates p,v,R with IMU and propagates its 15D
covariance. Its external-NDT observation path is a lightweight fixed-ratio
pose correction plus velocity_blend=0.6 from adjacent accepted observation
positions and existing covariance diagonal damping. It is NOT the native
FAST-LIO 23D IKFoM measurement update, and no new full Kalman measurement
model is introduced in R7. This distinction must remain explicit.

Original integer ns are retained in the input manifest and raw offset field.
The existing state kernels use double epoch seconds (around this epoch,
resolution approximately 238ns). R7 computes each exact output header as
uint64 header_ns+max_offset_ns, and converts that same instant to native
double for state arithmetic. The prediction publisher copies the exact
output header; the NDT PointCloud2 entry ignores point-time fields because
the cloud is already deskewed. Internal LiDAR deskew is disabled.

The existing scan-end causal trajectory obtains history bracketing the end,
but only a head measurement no later than the end may drive its boundary
fragment. Receipt of a bracketing IMU after the end is required to establish
history coverage; it is not used to predict an earlier scan end. R7 publishes
the exact reference pose already used in point compensation, transformed
from IMU to LiDAR, then queues the cloud until that same-stamp prediction is
present. The entire source remains real current-frame input.

At NDT reception, T_M_L is converted with inverse(T_I_L) to T_M_I. The original
OOSM rollback snapshot may precede the measurement by up to 20ms; the new
opt-in boundary step advances to the measurement instant before correction.
Then the remaining real IMU samples are replayed to the old current time.
The corrected publication therefore carries current-state time, not the older
NDT measurement time. A same-measurement CSV receipt plus newly received
corrected stream is the replay ACK. Less than 1us timestamp tolerance is only
for corrected-message double→ROS-time representation, not a physical
measurement-alignment allowance. Existing oosm.csv separately reports the
original rollback fragment and the lineage CSV reports actual correction time.

The opt-in source path preserves original formula kernels but splitting an
IMU interval changes nonlinear discretization and dt² process noise; it is
not advertised as byte-identical to the legacy mis-timed branch. No test GT
or future LiDAR is used. A successful twenty-frame run establishes engineering
integration only, not independent map-traversal or absolute-pose accuracy.
