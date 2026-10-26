# R31 implemented contract

Let T_cert be the native terminal admitted by the existing quality function and T_lim the result proposed by the unchanged limiter relative to the last successfully published map observation.

LEGACY (default): existing behavior, including T_lim publication. STRICT_FAIL_CLOSED (explicit research opt-in): no original quality certificate/convergence means no publication; T_lim different from T_cert means `ALTERED_OBSERVATION_NOT_CERTIFIED` and no publication; otherwise publish T_cert exactly.

No refused proposal can update map-observation history or create a pose, TF, Path or EKF measurement. PCL working buffers, candidate logs and attempt counters may change; they are not successful-map history. Real IMU propagation and existing OOSM validation continue. A rejected frame does not manufacture Prediction Fallback as an NDT observation.

Original candidate convergence is not copied to an unoptimized limited pose. A strict refusal does not trigger a newly invented search: C retains the original quality-event definition and its existing yaw starts/budget. The safety policy may therefore cause observation starvation; that is an explicit performance outcome, not a reason to relax it after scoring.

Eigen/PCL terminal rotations have float roundoff. The observation certificate and actual ROS message are compared using the same normalized-quaternion serialization that `publishPose()` already uses. The gate itself is conservative: unchanged original matrix, no limiter-change flag, no numerical-tolerance admission rule. The1e-12 post-run comparison tolerance concerns floating message reconstruction only, not a loosened motion/quality threshold or SE3 alignment.

Every input logs its certificate, proposal, actual sent pose, prior timestamp/age, final publication reason and pre/post publication count. Runtime checks cover all refusals; offline checks also match the actual NDT topic capture and the actual EKF/OOSM receipt. An emitted observation is not automatically a successful EKF update or a correct physical pose.

Engineering invariants and legal timestamp handling are assessed separately from localization error, starvation, recovery and resource cost. Neither test success nor an unchanged certificate proves localization accuracy.
