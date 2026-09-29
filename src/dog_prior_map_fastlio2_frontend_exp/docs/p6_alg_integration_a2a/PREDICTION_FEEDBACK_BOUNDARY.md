# Prediction feedback boundary

The adapter uses `WINDOW_15D_PREDICTION`: a successful optimizer transaction
marks the latest optimized `WindowState` as feedback-ready, and the next
causal state prediction starts from that 15D state.  Revision checks in the
existing window prevent a newly appended measurement from reusing stale
feedback.

The adapter does **not** call `FastLio2IkfomFrontend::setWindowPredictionSeed()`
and does not claim that a 15D state is an IKFoM 23x23 posterior.  No old EKF
covariance is used for NDT risk decisions.  Any future caller that needs such a
decision must either derive the required marginal covariance from the joint
window or stop with an explicit mapping blocker.
