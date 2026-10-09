# P10-R5: short causal inertial anchor (development only)

Start: 65a416ec19069efa0405e9fc56a0505dc9c3018e. Floor01 remains development data.
All R1-R4 archives, coupled objective/pullback/search, deskew, filter and noise
are unchanged. No GT/oracle/canonical data enters this algorithm.

## Version 0 contract, frozen before real R5 replay

Anchor stores map_T_lidar, stamp, last propagated stamp and validity. Establish
from the actually corrected filter state only after an ordinary untriggered,
effective nominal frame with no pending episode. Never reset a valid anchor to
the latest NDT. Propagate once each scan using
inverse(previous corrected map_T_lidar)*current predicted map_T_lidar.
Expire after 2.0s; nonmonotonic time, propagation gap >0.25s, invalid rigid
transform or ineffective nominal invalidate it. A pending episode cannot renew
its anchor. Missing/expired anchor retains nominal with zero search/confirmation
work. Expiration is not bypassed for pending. Seed again only on a later stable
frame. Actual alternative feedback invalidates the anchor; an admitted shadow
episode does not. Ordinary anchor propagation requires only small matrices.

Pending creation uses unchanged R3 deterministic terminal ranking, match quality
and physical guards, and freezes the current Hessian's 1D/2D weak basis W. No
FULL263/canonical information. R4 energy/motion accumulation remains diagnostic;
R4 D<0 and the R4 targeted orientation veto are not R5 admission requirements.
The underlying R3 local-candidate rotation guard is unchanged. Both branch
rotation differences are recorded, rather than using rotation alone to judge
weak absolute position. The full existing R3 quality/continuity/rigidity/search
bounds and two future confirmations remain mandatory.

R3 alternative propagation deliberately remains its frozen prediction-to-
prediction expression; only the new anchor excludes preceding pose corrections
using the actual corrected-to-predicted IMU interval. Do not describe both
tracks as IMU-only. Invalid R4 diagnostic arithmetic is explicitly recorded,
but cannot reject an otherwise valid anchor episode.

At creation and each accepted confirmation, with the same frozen W:

    eta = [(p_candidate-p_anchor_pred)/0.8;
           Log_SO3(R_candidate*inverse(R_anchor_pred))]
    e_w = W^T eta
    A_nom = ||e_w_nom||^2; A_alt = ||e_w_alt||^2

Use normalized rotation carriers (same convention as pose distances). This is
the P9 map-product chart, not a body SE3 logarithm. Record full W and anchor,
the individual/cumulative costs, and W/S fractions of candidate-nominal chart
displacement. W is never recomputed during pending confirmations.

Admit only after exactly three valid contributions and two confirmations:

    (sum A_nom - sum A_alt)/3 > 0.01
    sum A_alt <= 0.90 * sum A_nom

The absolute dimensionless squared-cost guard is a meaningful engineering
margin (pure translation corresponds to 0.0064m^2 per frame), not floating-point
noise; it is not a guarantee of an 8cm position improvement. The relative 10%
guard prevents a negligible advantage on a large accumulated error. Neither
is tuned on R5 outcomes or GT. Zero/invalid information retains nominal.

Anchor is not an independent absolute-position truth: its origin is a previous
filter state and its inertial increment depends on that state's velocity/bias.
The experiment tests short-term non-absorption of NDT pose corrections, not
global localization certainty. Only actual feedback trajectory errors can
establish a development benefit. No posterior/covariance interpretation.

Modes anchored_shadow and anchored_guarded_feedback are opt-in. Feedback uses
existing lidarMeasurementToImu/applyPoseMeasurement, consumes pending once and
affects the next actual prediction. Production and all older modes stay nominal
or follow their unchanged legacy policy. One existing source/map backend only.

Full 4127-frame shadow first; if at least one legal admission, full feedback
immediately. Freeze all outputs before GT. Reuse hash-verified R3 control. Main
metric corrected executed IMU trajectory; raw measurements separately. Keep
TX4127 in all ledgers despite missing GT bracketing. Large-jump definition
unchanged (>0.5m OR >10deg consecutive corrected poses). Report mean/P95/max
full-frame processing including logs except cost row, process wall and RSS.
Goal >=5% translation RMSE improvement, mean<=100ms/P95<=150ms, not promised.
At most one reasoned targeted change; preserve initial results. No grids or
other datasets. Stop after delivery.
