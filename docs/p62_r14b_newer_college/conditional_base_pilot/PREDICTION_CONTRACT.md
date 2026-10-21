# Common-Prediction Pilot, not three independent closed loops

Only ordinal767 reference pose initializes map pose. For future t, with two admitted CONTROL_FULL historical states Ta,Tb and positive integer-ns intervals, TP=Tb Exp(((t-tb)/(tb-ta)) Log(Ta^-1 Tb)). This is right/body SE3, with full translation Jacobian.

With fewer than2 states, hold last valid pose. CONTROL_FULL is admitted using the unchanged finite/converged/rigid,0.8m/5degree, CoverageV2/Overlap>=0.50 conditions. Rejection disables extrapolation and holds the last admitted history until another admitted Full result; no rejected pose is inserted. No current/future reference, Split or Proposed update can affect TP.

Actual run: all20 Full outputs admitted; no fallback needed. FINAL_OBSERVATIONS.csv records history count/stamp/hold. All methods share each TP. No NDT/decision feeds a real EKF. Prediction is a baseline estimate, not a rejected observation re-labelled as accepted.
