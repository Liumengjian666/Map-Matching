# P61-R11 bounded clock and bias provenance audit

Base a0275decc7862030c3581038ab87d09efe5aaa4e; local and remote R10 both verified
at this SHA, clean local tree. New independent worktree/branch as requested.

Fixed target window1706671804799723787..1706671805899593546ns, scans5154–5165.
Read only original /livox/lidar and external /imu/data messages in that window,
plus the nearest frozen R10 causal-head IMU needed to verify message identity.
Compare exact original header and record time to frozen R8/10 metadata. Describe
recording lag only, never infer physical clock offset from it.

Bias provenance scope: frozen R6 author source/build manifest/config, actual
bridge logs and native state records, and the exact original samples that the
source/logs identify as initialization inputs. No new gyro-bias estimate, no
new static detector and no search for favourable stationary periods. If native
state update attribution cannot be reconstructed without rerunning the filter,
report that limitation instead of executing it.

Evidence search is limited to existing R2/R4 device/config archives, R5/R6
author implementation/configuration/logs, R8 preparation/runtime inputs, R10
frozen evidence, and existing local driver packages identifiable from those
assets. No web search/latest driver substitution, download, dependencies,
external CLI, physical sensor operation, GT, optimization or localization.
Filename metadata may be searched without reading a GT payload; do not include
GT content as evidence. Negative searches are scoped, not proof of universal
absence. No sensor mean is called bias or low angular speed called stationarity.

Separate decisions:
H11-A: CLOCK_SYNC_SUPPORTED_BY_PROVENANCE / CONTRADICTED_BY_PROVENANCE /
NOT_CERTIFIED, using independent actual synchronization provenance, not shared
epoch formats or small recording lag.
H11-B: BIAS_STABILITY_SUPPORTED_BY_INDEPENDENT_EVIDENCE /
CONTRADICTED_BY_INDEPENDENT_EVIDENCE / NOT_EVALUABLE. Initial or later estimator
bias alone is not independent truth. Without independent validation, preserve
R10 conditional interpretation rather than changing its numerical result.

All frozen assets/source identities are checked before and after. Add only R11
code/reports, no changes to production C++, configurations, maps or prior reports.
GT reads0; ROS localization nodes0; rosbag play0; new NDT align0; EKF runs0;
new trajectories0; production changes0; external CLI0. Local commit only.
