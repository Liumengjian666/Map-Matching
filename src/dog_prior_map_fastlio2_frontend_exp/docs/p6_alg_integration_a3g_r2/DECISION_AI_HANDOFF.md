# Decision-AI handoff

```text
PAPER-P6-ALG-INTEGRATION-A3G-R2 执行回报

Git:
START_SHA=283be8cf8298e24200099c4a1e51eb3ed1c2d0f7
CODE_SHA=NONE (production code unchanged; report-only changes)
END_SHA=见本轮 analysis commit
remote HEAD=见提交后核验
worktree=提交后核验

Input:
dataset=SuperLoc Corridor01
SHA gate=PASS (22/22 frozen external artifacts; relevant raw/map/calibration/IMU hashes match)
selected raw records=19/19 exact count and point-time extrema match catalog/logs
REAL_REPLAY_COUNT=0
GT_USED=false

Reconstruction:
offline deskew reproduction=BLOCKED_BEFORE_RECONSTRUCTION
reason=run artifacts omit exact scan-start velocity, gyro bias, accel bias, point-level deskewed cloud, and IMU pose knots
production geometric-support reproduction=NOT RUN (strict deskew prerequisite failed)
OFFLINE_GEOMETRY_REPRODUCTION_MISMATCH=NOT APPLICABLE

Selected geometry:
map overlap / nearest-neighbor distributions=NOT COMPUTED
production 0.8 m support layers=NOT COMPUTED
predicted-pose counterfactual=NOT COMPUTED
tx173/tx174 recovery comparison=NOT COMPUTED
tx182/tx183 paired geometry=NOT COMPUTED

TX166 logged capsule (telemetry only, not reconstructed geometry):
stamp_ns=1517157235829884363
raw/deskew point count=28692 / 28692 (saved summary)
saved time interval=1517157235729044914..1517157235829884363
saved provenance=WINDOW_OWNED_SE3_DESKEW
saved displacement mean/P95/max=0.3145528608 / 0.6693454877 / 4.1230548848 m
saved NDT converged/iterations/objective=1 / 0 / 0
saved U_obs status=NO_VALID_GEOMETRIC_CORRESPONDENCES
saved selected NIS=NOT_REACHED
map neighbors and accepted-observation counts=NOT COMPUTED

PRIMARY_ROOT_CAUSE_CLASS=A3G-R2-F ROOT_CAUSE_NOT_ISOLATED
UPSTREAM_REPAIR_TARGET=NONE established; missing evidence is point-level deskew/state capture
production changes=0
REAL_REPLAY_COUNT=0
A3G_R2_LIDAR_SUPPORT_ROOT_CAUSE_ISOLATED=FAIL
A3G_R2_STOP_GATE_COMPLIANCE=PASS
READY_FOR_FORMAL_EXPERIMENT=NO
```

Do not infer an algorithm repair from this stop. The next evidence-gathering step should only add/locate the missing frozen per-scan state or point-level deskew product, then rerun the offline gate. No new replay is authorized by this handoff.
