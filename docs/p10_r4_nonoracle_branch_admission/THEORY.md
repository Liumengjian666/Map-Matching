# P10-R4: three-frame non-oracle admission

Development on frozen Floor01 data, not independent validation. R1/R2/R3
results and the production nominal update remain unchanged.

## Frozen version 0 contract (before any R4 real replay)

The R3 trigger, coupled chart/jet/step/search, candidate ranking and physical
guards are unchanged. Only existing nonlocal PendingCandidate episodes are
eligible for three-frame admission; local R3 recommendations remain shadow
diagnostics and are not directly fed back.

On creation plus two successful future confirmations, use the same current
prepared source and existing map's dynamic PCL value kernel for both poses:

    E_b = -score_b / N
    D_E = sum((E_alt - E_nom) / max(1, abs(E_nom)))

Reuse the already evaluated alternative score. At creation only, explicitly
evaluate the nominal with the same value kernel (one value call, not an align)
to avoid comparing a native Euler jet carrier against a pose value carrier.
Future confirmations already evaluate both scores in R3.

For each of the two intervals, compare branch increments against the causal
IMU propagation increment in the LiDAR frame:

    delta_b = inverse(T_b_prev) * T_b_cur
    delta_imu = inverse(T_updated_lidar_prev) * T_predicted_lidar_cur
    r_b = Log_SE3(inverse(delta_imu) * delta_b)
    M_b = ||r_b.translation / 2m||^2 + ||r_b.rotation / 15deg||^2
    D_M = sum(M_alt - M_nom)
    D = D_E + 0.05 * D_M

The start state is the actually corrected preceding filter state; no later
measurement is used. This avoids interpreting a preceding NDT correction as
IMU motion. R3 alternative prediction itself remains exactly its frozen
formula T_alt_prev * inverse(T_pred_prev) * T_pred_cur.

Reuse the existing so3Log and so3LeftJacobianInverse for the SE(3) logarithm.
The translational logarithm is J_left_inverse(phi)*t, not just raw translation.

Admit only with all existing convergence/rigidity/quality/physical/two-frame
confirmation guards, finite accumulations and D < -1e-6. The fixed dimensionless
1e-6 tie guard conservatively retains nominal near numerical equality; it is
not tuned on GT or on actual R4 outcomes. Invalid admission inputs clear the
pending episode without searching again in that frame.

Snapshot current terminal and all cumulative quantities before clearing
PendingCandidate. A completed episode is consumed once. A candidate, temporal
support, and admission are separately recorded.

## Explicit experiment modes

CONTROL and legacy EVENT remain unchanged by default (admission disabled).
EVENT_SHADOW enables admission receipts but applies nominal only.
GUARDED_FEEDBACK applies only an admitted current map_T_lidar through the
existing lidarMeasurementToImu and applyPoseMeasurement. Noise and IKFoM are
unchanged. Every next scan starts from the actual resulting state; failure is
recorded and stops that experiment, not silently retried with nominal.

The admitted float pose passes through the existing Pose3d normalized-quaternion
carrier before measurement conversion. Receipt parity permits only 1e-6m and
1e-5deg for that carrier round trip; nominal receipt parity remains byte-exact.

No new map, extra search, GT/canonical access or bootstrap work. Normal frames
have zero extra jet/value/align work; pending frames one extra align; search
frames at most two; total at most three per frame. Full fixed sequence in both
modes, with no GT loaded until both blind outputs have been frozen.

## Evaluation / stopping

Reuse immutable R3 CONTROL, verify full EVENT_SHADOW source/measurement/state
parity. Report raw measurement separately from corrected executed IMU trajectory.
GT uses the existing fixed anchor and extrinsic, with no fitted alignment and
no extrapolation. Preserve TX4127 when GT is unavailable. Large jump descriptive
counter: consecutive corrected translation >0.5m OR rotation >10deg; compare
against CONTROL rather than claiming each count is a failure. Also report
maximum update correction and admitted-versus-nominal measurement difference.

Goals: full processing mean <=100ms, P95 <=150ms and corrected translation RMSE
improvement >=5%; accuracy target is not guaranteed. At most one separately
reasoned admission-only improvement is authorized; no parameter grid. Stop
after fixed task delivery. This round is not production deployment.
