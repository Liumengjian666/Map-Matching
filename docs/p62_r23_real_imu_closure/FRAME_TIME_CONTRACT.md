# R23 frame/time contract

T_A_B maps B coordinates to A. Original T_imu_lidar rotation diag(-1,-1,1), translation[-.006253,.011775,-.028535]m. EKF state is IMU pose; published high-rate odom and NDT measurement are T_map_lidar. Existing EKF observation callback converts using inverse(T_imu_lidar). Input/config exact matrices and anchor boundary IMU values are in input/INITIALIZATION_AND_TIME_CONTRACT.json; no evaluation GT is runtime input.

LiDAR filename/header is scan start. Original Ouster point t is uint32 nanoseconds from scan start; existing publisher derives float time seconds. Existing ekf_imu_fastlio Deskew uses scan-start reference and IMU-frame state; R23 does not change geometry, interpolation or point values. The NDT record now extracts the preserved time field to record true scan end; its measurement timestamp remains scan start.

NDT TP comes from latest received valid EKF sample at/before scan start, propagated to that time by existing bounded body-twist interface (<=20ms). Provenance, sample acceptance wall time, query wall time, align times and actual map publication are recorded. This is an IMU-EKF-driven prediction interface, not R14B two-pose predictor.

OOSM preserves original20ms alignment: update at the recorded rollback historical state, then replay real IMU to current state. No new exact-time interpolation is introduced. Actual application time is rollback_stamp from oosm_statistics.csv, with alignment_error_ms; do not label it exact measurement time. Wall-clock completion is distinct from historical measurement time. Invalid ordering/frames stop the run.

No extra prewarm or GT-derived velocity/bias; replay every real IMU after anchor boundary in chronological order. The moving anchor and zero velocity prior are an explicit limitation.
