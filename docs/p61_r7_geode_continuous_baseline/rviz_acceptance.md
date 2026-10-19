# Actual live ROS/RViz acceptance

- CODE_SHA17922f2b63dae1f2af34f353287728d50a312f73 launched real NDT PID98013,
  EKF PID98014, RViz PID98018 on private master11398.
- Twenty scans,208 real Xsens messages,20 distinct preprocessed source hashes,
 20 real align and20 OOSM accepted updates; no prerecorded pose injection.
- `rviz_live_acceptance.png`: actual own-window capture immediately after live
  twenty-frame processing, with all dynamic trajectories and clouds retained.
  Complete prior map not yet visible: nonlatched one-shot publisher connected
  too late. This deficiency is preserved, not hidden.
- Only visualization map publisher was then restarted with latch=true. The
  actual topic `/geode/frozen_map` width was received as321053. Final capture
  `rviz_frozen_map_and_real_tracks.png` shows frozen gray map, real yellow
  current deskewed scan, red NDT-aligned scan, cyan prediction trail, green NDT
  trail, orange corrected trail and real EKF TF; global status is green.
- Capture used Qt grabWindow of the actual RViz window, whose _NET_WM_PID was
  independently checked as98018. No screenshot synthesis or R6 snapshot reuse.
- Complete map publisher is visualization-only. Restart did not publish raw
  scan/IMU inputs, reinitialize NDT, modify the map, or add align calls.
- During-run RViz VmHWM177.363281MiB; after complete map display214.796875MiB.
  Neither is attributed to the NDT node. See visualization_resources.json.
- Archival launch includes map latch plus safe fresh default output directory;
  subsequent full replay is not authorized. Actual/safe commands are in REPRODUCE.md.
- All task processes were shut down normally after capture; master11398 closed.
