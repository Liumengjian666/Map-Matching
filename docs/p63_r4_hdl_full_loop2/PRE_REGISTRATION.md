# P63-R4 full loop2 frozen engineering baseline

START78dfc7ce6649b4c8fb25626e55a8e3efd82ef70b;native execution specification R3 CODEeb6ff37ab791193ff1a803952d79e148b9ecea6c. Only existing release libraries and four pinned official sources. No recompilation, no map/parameter/IMU/UKF/NDT changes, no extra align. New diagnostic/scheduler CODE will be the commit containing this registration and SOURCE_FREEZE.json before input.

## Full input fixed before trial

Original loop2_raw.bag, fresh full-file SHA d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae. Preflight reread all10905 LiDAR serialized payloads and current bag index;verified monotonic header stamps,MD5,frame_id,point layout,finite XYZ and per-frame hashes. INPUT_IDENTITY.json and INPUT_FRAMES.csv give actual current count/time ranges. First400 bytes/XYZ match R3. No source bag modified or copied. Map SHA016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca;original459154 points;native0.1m target424323 points,XYZ SHAd4ae17e75e22b2c660df1ca08140ad3675479c79f214da6253706c239ca4325e.

Observe fixed600<=relative_sensor_s<=650, where relative_sensor_s=(header_ns-first_LiDAR_header_ns)/1e9. Bag record start is separately reported;it is not used to shift scans. No GT/old trajectory consulted. If stop before600,interval NOT_REACHED;do not resume or select a new window. Around619s receives the same rules as every frame.

## Native method unchanged

NDT_OMP,UKF,resolution1m,DIRECT7,epsilon.01,constructor max35/step.1/outlier.55,source/map voxel.1m,cool time2s. use_imu=false,use_global_localization=false,robot odometry prediction=false,convert_utm_to_local=false. Manual T_map_lidar=I4. Raw bridge bytes are exactly R3 actual code. No T_imu_lidar,deskew,new geometry gate,recovery or alternate optimizer. Source/child frame both livox_frame;map coordinates are not transformed.

The original UKF updates from NDT unconditionally along its fixed correct() path. Native convergence,actual completed callback/odom update,external validity and correctness remain distinct. The external R3 screen is unchanged:finite raw/filtered state,filtered quaternion norm in(.99,1.01),native and status converged,finite all-NN fitness and native inlier mean error,and native inlier_fraction>=.50. It only controls statistics and end of this experiment;it never rejects a UKF update or writes a fallback pose.

## Scheduling and stop rules

One fresh output full_trial_01;exclusive mkdir prevents overwrite. For each original scan,verify payload before publishing,wait for same-stamp native NDT/callback/bridge/status/odom,then release next scan. Transport Header.seq replacement is verified against known rospy sequence;XYZ/other data and header time stay exact. The new incremental CSV reader replaces only R3's repeated whole-file parsing;offline tests reproduce all400 old screen/step results without an align.

Stop immediately for nonfinite state;stop at20 consecutive external-invalid updates;stop at3 consecutive output increments satisfying translation>2m OR rotation>20deg (same R3 thresholds). Do not delete the failed20 or stitch later data. Target change,identity/link mismatch,monitor failure,disk reserve<20GiB,missing/invalid diagnostic association,30s response timeout are EXECUTION_CONTRACT_FAIL rather than fabricated tracking loss. Whole-bag EOF must match10905. No restart,parameter adjustment,skip or second formal run.

A requires all10905 completed,immutable target,no stop and complete native NDT/callback/UKF receipt linkage. B is a real nonfinite/step/external-validity stop under these rules;it does not prove physical position error or an author-native rejection gate. Other contract failures are C.

## Read-only diagnostics and resources

Original R3 C++ timing/target probe binary stays identical. New Python recorder saves existing /status prediction_errors['motion_model']=T_prediction^-1*T_raw_NDT,reporting translation norm and quaternion angle only. No prediction pose is written back. Native /odom and full-callback receipt after each align plus frozen unconditional UKF correct() source establish update lineage;no new internal UKF mathematical counter is inserted.

Record all-frame native compute/callback/diagnostic/bridge time,Mean/P50/P95/P99/Max;50ms RSS sampling includes manager,map server,RViz,ROS children and input recorder with scopes separated. Native getrusage peak is preserved. Screenshot atscan20,6201,and10905 (if reached),plus stop;GUI failures are not localization decisions. The only visual difference from actual R3 is scoped Noetic dialog suppression and a static wider map camera;neither changes coordinates or target selection.

The full sequence is offline handshaken,not proof of10Hz. No officialATE/RPE,GT feedback or assumed physical loop closure. Keep original start/end estimates without calling them closure error. Reference correctness and repeated-structure mismatch remain UNKNOWN unless actual records support a narrower statement. No novel module or second framework.
