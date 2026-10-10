# Experiment protocol

- Dataset: SuperLoc Corridor01 prospective input; not a historical P2B v1
  reproduction.
- Window: 5 s startup plus 35 s evaluation. The frozen slice has 396 scans,
  346 in the evaluation interval, and 7,989 original IMU messages.
- V2 replay input: all V1 records preserved, with 16 genuine trailing IMU
  samples appended so the final fixed-lag deadline is covered. V2 has 396
  clouds, 8,005 IMU messages, and a 40.0 s sensor-time span. No point cloud,
  map, original IMU sample, initialization, or NDT/EKF parameter was changed.
- Playback: same ROS Noetic nodes and task-clone binaries for all runs;
  `rosbag play --clock --rate 0.25`; one mode per fresh run.
- Fixed-lag parameter: enabled only for this experiment, `0.150 s`, queue
  capacity 64. NDT timestamps and IMU timestamps remain sensor timestamps.
- Modes: `CONTROL_V2`, `COUPLED_SHADOW_V2`, `WEAK_ONLY_FEEDBACK_V2`, and
  `COUPLED_FEEDBACK_V2`.
- Control/Shadow consistency gate: same NDT objective, point-cloud identity,
  deskew results, OOSM release nodes, replay counts, and output topic poses.
- Posthoc transform: one full-pose SE(3) transform fit only to
  `CONTROL_V2` corrected IMU states in the first 3.0 s of the 35 s evaluation
  interval. The same transform is applied to every method. The primary metric
  window is evaluation time 3–35 s, outside the fit interval.
- Metric meaning: `PREFIX-ALIGNED RELATIVE DRIFT`, not absolute map-frame ATE.
  The position-only Kabsch fit is retained solely as a conditioning diagnostic.
- GT remained unopened until `pre_gt_freeze.json` had frozen inputs, runtime
  sources/binaries, and all run outputs. The only subsequent code correction
  was to read the actual `oosm_result` CSV column in a posthoc counter; it did
  not change replay data, trajectory values, evaluation transform, or metric
  thresholds.

## Frozen artifact identities

- V1 input bag SHA256:
  `f6e77c96d97402e551835e81a1ce8d780d075da2f18a9cb5e4fc0646628b86d6`
- V2 fixed-lag input bag SHA256:
  `7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e`
- V2 input manifest SHA256:
  `bd2c13a49a50d07ac0009e40ef25986d3cd538bec4c9108d51b917f69f99367e`
- Normalized map SHA256:
  `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`
- LiDAR→IMU extrinsics SHA256:
  `59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d`
- NDT runtime binary SHA256:
  `d26fb63cb550b3ca4238b24c405c22d1bde0a677c8fdad429985c046c33c12cb`
- EKF runtime binary SHA256:
  `b8359c3251daf73deb93754421a6d67fa79e2af3404c60dd8375c9d529a1993e`
- GT SHA256 (opened after freeze):
  `3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03`
