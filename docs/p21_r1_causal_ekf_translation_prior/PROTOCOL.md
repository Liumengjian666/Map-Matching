# P21-R1 frozen translation-initial-source ablation

START_SHA: 98c08420267d9a6dd969e850f3a5f9e640c47cb5.

Only new default-off `lidar_update/experimental_ekf_translation_fallback_enable`.
CONTROL, visual on, camera/geometry off, beta=.6; original map, bag, visual CSV,
initialization, IMU, NDT parameters, step limit, EKF and fixed-lag=.15 unchanged.
Rebuild only the NDT target and focused tests; EKF binary hash must remain
fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a.

Source provenance: frozen EKF current-time high-rate publication follows
IMU propagation and fixed-lag processing, never intermediate rollback;
corrected output is a different topic. Frozen ekf_imu_fastlio converts pose
to map_T_lidar and twist to child LiDAR frame. Release of an observation m
requires m+.15<=sample time. Therefore no correction later than a selected
sample<=scan-start can enter it. Runtime publisher/frames/ordered stream,
selected sample and lineage associations must confirm these source premises.

When enabled only, subscribe existing high-rate topic. Validate publisher,
frames, finite pose/twist and increasing timestamps before watermark/history.
Duplicate/reversed/invalid stream or pending overflow stops the node, never
chooses a nondeterministic sample. Wait using existing pending callback queue
until accepted sensor-time watermark>=target, without blocking under mutex.
Select latest sample<=target; age<=20ms. Reuse original body-twist prediction.
After visual failure, previous pose and valid IMU rotation required; replace
translation only. Visual always has precedence. Missing/stale prediction
retains the original initial pose exactly. No Coupled refinement/Anchor call.

Gates: (1) source/coordinate/causal contracts and focused tests; (2) new NDT
default-off Visual prefix, duration18.5 from bag start:183 NDT,182 APPLIED,
3698 high-rate,106 visual. Compare first180 complete transactions with P19,
positions<=1e-9m, rotations<=1e-8deg; original timestamps/discrete exact,
fitness tolerance1e-12*max(1,abs(score)); exclude wall/runtime/history watermarks
as P19 already specifies. No extra EKF subscription default-off.
(3) one enabled18.5s short run, same gates plus at least one legal EKF
translation use. No time/threshold relaxation or parameter search.
Only after all pass: one full40s enabled run. Require396 NDT/OOSM/corrected,
8004 high-rate,169 visual,0 late/nonfinite/extra-align. No baseline full rerun.

Hash runtime outputs before GT. Reuse original Control V2 0–3 full-pose
transform/interpolation, primary3–35s; diagnostics6–12,26–30,30–35,32–35,34–35.
NDT LiDAR stages convert to IMU origin. PREFIX-ALIGNED RELATIVE DRIFT, not
absolute ATE. Pair actual time arrays, no favourable sample intersection.
Separate direct visual, EKF fallback and original history-source frames;
cross-run histories differ, not isolated framewise contributions.

Engineering reference: global EKF RMSE<=1.3105137419508925m and
P95<=2.680026856318874m; seek32–35s improvement and max<=6.247121759931853m,
stronger max reference5.667156999562539m. These are not significance tests.
Preserve all negative/mixed metrics. No further run or tuning after result.
Default-off/prediction/short gate failure stops the full experiment. Only
actually produced evidence may be archived; no bags/PCDs committed.
