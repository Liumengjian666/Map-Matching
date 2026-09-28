# P6-I6B 100-frame Floor01 smoke

- All four modes replayed 100 aligned scan/IMU events through the real closed-loop runner.
- Each mode has finite normalized predictor/corrected poses, valid quaternions, increasing timestamps and matching event rows.
- M0 converged on all 100 frames per mode; every executed M+/M- probe also converged. STRICT_BASELINE and UOBS_ONLY each used 100 NDT calls; UNONLOCAL_ONLY and DUAL_RELIABILITY each used 108 calls (4 probed frames × 2 extra calls).
- STRICT parity versus the frozen I6A trajectory prefix: max translation delta 0 m; max rotation delta 1.98790734e-16 deg.
- Position response diagnostics identify map_T_imu origin after applying the frozen imu_T_lidar lever arm; raw M0/M+/M- map_T_lidar poses remain recorded separately.
- GT was not loaded or used for smoke validation.
