# Outdoor readiness: data admission only, not a formal-run PASS

|Required answer|Actual R30 finding|
|---|---|
|Does current code have a deterministic foundation error?|Yes: first global-PCD Gaussian target can retain1m cells despite.8m parameter. Reproduced and minimally repaired. A separate pre-quality/post-limiter observation-identity risk is directly demonstrated and left open.|
|Does it affect MID360?|Grid order is sensor-independent on the affected global-PCD path; R29/P45 tile path already establishes.8m before loading. Post-admission pose alteration is common and could affect MID360 after sparse/rejected observations. Azure-specific small Source and bursty IMU observations cannot be assumed to recur identically.|
|Can NDT match near correct initialization?|8/8 frozen real inputs, max.047317m/.992850deg distance from diagnostic reference seed. This is oracle-seeded diagnosis, not complete causal localization.|
|Are IMU/EKF propagation and observation admission correct?|Finite-interval/frame tests pass, timestamp-qualified updates and proper incomplete-replay rejections are observed.3.17164625s of large intervals are not integrated. raw62 shows that the actually published pose can differ from the quality-approved native candidate. Whole-chain semantics are NOT fully accepted.|
|Does repaired code pass independent regression?|6/6 core tests,2/2 supplied Python tests, repaired native-node Release build and actual1/1 CTest pass. No repaired full trajectory has run.|
|May Outdoor01 input admission begin?|Recommended YES for controller-approved read-only asset/unit/time/calibration preparation; NO automatic complete localization/accuracy run and NO claim the foundation is fully closed.|

MID360 admission must confirm actual acceleration units rather than assume SI from ROS Imu; actual point offset/header/timebase and scan reference; official T_lidar_imu direction; initial gravity/bias/velocity assumptions; native target Gaussian size; raw gaps including bag boundaries; and observation identity through NDT→step limiting→EKF. These are minimum contracts, not permission to tune C+G from GT.

No Outdoor download, ROS run, future-GT prediction, new threshold or new search direction occurred. R29 negative results remain unchanged. The global-PCD repair is a new code version and cannot be labelled the exact same executed R29 system. Newer College coordinate ambiguity is not used to block this decision.

Unique next decision: freeze the map-observation identity/quality contract exposed by raw62, then have the controller decide the minimal closure test and Outdoor01 admission order. Do not launch a second repair or a complete sequence automatically.
