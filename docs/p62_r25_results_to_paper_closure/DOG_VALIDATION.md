# R25 dog validation: BLOCKED, not a measured deployment comparison

## Verified local assets

The original loop2 bag exists at the protected path, 19,677,401,800 bytes. Read-only indexed ROS bag inspection reports1090.511421s,10905 livox_ros_driver2/CustomMsg scans and218101 sensor_msgs/Imu measurements. Archival expectedSHA is d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae; a new whole19GB digest was not needed after the coordinate block, so this task does not claim a fresh full-payload verification.

The matching FAST-LIVO2 PCD and historical causal initial-guess record are present and freshly hash-verified:
- map016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca;
- initial record e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a.

Stable runtime actually uses /livox/lidar CustomMsg, /livox/imu, camera_init map and livox_frame output, .25m Source/cap1400, configured .8/.08/.001/40 NDT, original midpoint reference, Deskew OFF, IMU gravity initialization200 samples and OOSM. Stable launch is dog_prior_map_localization_split.launch. Read-only snapshots are in dog/; nothing was written to the stable workspace.

## Unique execution blocker

The actual loop2 mapping config exists and was read:
/media/jian/HIKVISION/Jian_FASTLIVO_Data/dogrosbag/loop2_map_match_experiment_2026_08_08_17_36_19/config/loop2_config.yaml
SHA1cde331d3b092629e1c2b06ebf218a9c51122fe4f8feeef3ea6e9ef2fafae76c.

It explicitly sets extrinsic_R=I and extrinsic_T=0 with a comment saying to use identity/zero unless a separate LiDAR–IMU calibration is available. The inherited own_loop2.yaml independently records UNAVAILABLE_NOT_PHYSICALLY_CALIBRATED. Identical topic frame names do not establish the physical SE3 chain. The local avia.yaml has nonzero values but is explicitly for a different Livox Avia setup and cannot qualify Mid-360. No Ouster extrinsic, identity assumption or reference-fitted transform was substituted.

Therefore the prerequisite “assets complete AND coordinate chain established” is not met. DOG_BASELINE and DOG_CG_EXPERIMENTAL were NOT_RUN; both have zero new NDT calls. No raw rosbag replay, no new dataset/calibration search or second hardware experiment was started. An original node can technically start under its historical assumption; that is not the calibrated deployment comparison requested here.

## Reference and quality-scope limitations

lidar_poses.txt is a same-system FAST-LIVO2 mapping trajectory, not external GT. Existing reference_deviation_metrics.json used timestamp interpolation and subtraction at the first common position; its historical0.085809m high-rate RMSE is NOT a new R25 absolute-accuracy result. It does not establish a calibrated LiDAR–IMU transform. No dog rotation/RMSE improvement, recovery count, latency or RSS is fabricated for the unrun pair. The .30 Coverage threshold has not been qualified as a general Mid-360 threshold. No threshold was tuned.

## RViz/configuration handoff

A new independent visualization-only template is supplied:
`src/dog_prior_map_localization/launch/p62_r25_dog_asset_view.launch` and matching RViz configuration. It configures the protected map, filtered current scan, aligned scan, NDT pose/path and EKF paths with camera_init fixed frame. An optional same-system reference display is disabled until its frame provenance is qualified. No fake map-to-LiDAR TF is created.

**NOT EXECUTED / NOT LOCALIZATION ACCEPTANCE:**
```bash
source /home/jian/livox_ws/p62_r25_results_paper_ws/devel/setup.bash && ROS_MASTER_URI=http://127.0.0.1:11386 ROS_IP=127.0.0.1 roslaunch dog_prior_map_localization p62_r25_dog_asset_view.launch
```
This viewer alone performs no localization and cannot turn raw CustomMsg into an accepted trajectory; live filtered/path topics must come from a separately qualified estimator or a correctly framed recorded result. Merely replaying the raw bag is not a completed CG validation.

**Historical original startup form, documented but not executed in R25:**
`roslaunch dog_prior_map_localization dog_prior_map_localization_split.launch config:=/home/jian/livox_ws/p62_r25_results_paper_ws/docs/p62_r25_results_to_paper_closure/dog/STABLE_RUNTIME_CONFIG.yaml rviz:=false`, followed by a clock-consistent raw rosbag replay. No runnable CG deployment command is certified while the physical chain is missing. The R23 one-command RViz acceptance remains Newer-College-only; it is not borrowed as dog acceptance.

This block does not delay the G-only result, resource comparison, method summary or complete English manuscript draft.
