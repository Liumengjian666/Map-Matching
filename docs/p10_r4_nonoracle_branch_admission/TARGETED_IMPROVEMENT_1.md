# Single authorized targeted admission improvement

Frozen before attempt 1. Attempt 0 remains immutable, including its full causal
replays and GT counterexamples. Its engineering gates passed, but corrected
translation RMSE worsened by 0.1575516%. This improvement is development after
post-hoc inspection, not independent validation; no per-frame GT switching.

## Non-GT cause and change

Increment consistency alone cannot distinguish a common persistent left pose
offset: (L*T_prev)^-1*(L*T_cur) = T_prev^-1*T_cur. The weighted branch score can
therefore admit persistently rotated alternatives. Attempt 0's accepted poses
included up to 13.7665deg separation. Merely vetoing positive D_M does not address
that ambiguity; most accepted candidates already had D_M <=0.

Add only a three-frame orientation-consistency nondegradation guard:

    rotation_distance(T_alt_i, T_imu_pred_i)
        <= rotation_distance(T_nom_i, T_imu_pred_i) + 1e-6deg

The 1e-6deg carrier tolerance is the existing R3 local rule, not tuned on results.
Accumulate the boolean at creation and both confirmations. Require it in addition
to the unchanged D<-1e-6 and existing quality/continuity gates. Otherwise retain
nominal with ROTATION_PREDICTION_DISAGREEMENT. No absolute translation-to-nominal
or GT criterion. Initial pending ranking and both intervals' motion calculations
stay unchanged. It assumes the causal IMU orientation prediction is a useful
short-window reference, not that IMU absolute orientation is universally correct.

Use explicit experimental mode suffix _rotation_guard. Legacy modes and original
attempt 0 behavior stay unchanged. Candidate generation, objective, Hessian,
lambda .05, 2m/15deg scales, 8/16 search and <=3 aligns remain unchanged.

One additional full EVENT_SHADOW and one full GUARDED_FEEDBACK are authorized
within the original task's single-improvement allowance. Preserve all frames;
freeze both outputs before evaluating GT. No second improvement, threshold grid,
additional datasets or production switching in this task.
