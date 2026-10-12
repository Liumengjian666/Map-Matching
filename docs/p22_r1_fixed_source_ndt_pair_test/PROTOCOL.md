# P22 bounded fixed-source two-initial test

START_SHA=39c5eee473b379a1962d2333984ebd8492b9f36d.
No runtime-system edit, ROS replay, EKF measurement update or tuning.
Select frame2 and nearest nonvisual P19 scans to+23.5,+24.4,+24.8,+26,+29.9,
+32,+34.8s. Unique nearest, tie lower frame index; neighbourhood bounded
to.15s. Selection and all inputs frozen before GT. No new frame if one fails.

Existing outputs.bag has only odometry/diagnostics, not filtered points.
Attempt source recovery only from the raw frozen bag and recorded P19
published snapshots, using original core deskew/partial-propagation and
byte-identical NDT preprocessing/hash functions. This snapshot view does
not purport to contain unrecorded OOSM-rewritten intermediates. A recovered
source is admitted ONLY if count and original stableCloudHash match;
otherwise stop that frame without align. No approximate cloud substitution,
new fusion update, reconstruction of biases, or full filter replay.
Record ordered XYZ SHA256 on SSD, not in Git. Fewer than4 matched frames
ends with COVERAGE_INSUFFICIENT; no additional recovery method or frame search.

If at least4 pass, each pair uses P19 initial/previous Used/IMU rotation.
EKF alternative from P19's recorded high-rate map_T_lidar/body twist,
latest<=target,age<=20ms, original causal_pose_prediction.hpp. No P21 history.
Without arrival evidence label OFFLINE_COUNTERFACTUAL_ONLY.
Two independent identically initialized NDT instances, order A then B;
exactly two align per admitted frame,<=16 total. A must reproduce P19 Raw,
iterations/converged/fitness/Step Limit/Used before interpreting B.
Predeclared float parity: translation1e-5m,rotation1e-4deg,
fitness1e-6*max(1,abs(score)); iterations/converged/limit flags exact.
No later tolerance relaxation. Nonfinite or parity failure stops.

Freeze actual outputs before GT. Original ControlV2 0–3 full-pose fit and
interpolation, IMU origin, prefix-aligned relative drift. Frame2 diagnostic
only; other selected frames in3–35s. All per-frame deltas B−A, Raw error
reduction>=.1m descriptive marker only. No trajectory-improvement claim.
If early stopped, archive only actual artifacts, no fake pair/GT results.
End initialization-source research after this bounded task regardless of
result; NEXT belongs to the decision AI's Coupled research direction.
