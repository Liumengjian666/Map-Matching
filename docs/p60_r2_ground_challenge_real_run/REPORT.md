# P60-R2 Ground-Challenge Corridor2

## Result

Primary: **FUSED_LOCALIZATION_INPUT_CONTRACT_BLOCKED**.

Secondary: **RAW_NDT_20_FRAME_ENGINEERING_PROBE_COMPLETED**.

The downloaded Corridor2 bag is usable and has been copied to the internal disk with independently equal source/destination SHA256. The actual production CONTROL NDT node executed 20 consecutive raw-cloud registrations. The requested Deskew + IMU prediction + EKF/OOSM experiment has **not** passed its Smoke test and was not started. Its immediate blocker is the missing, traceable `T_imu_lidar` connecting cloud frame `sensor` to Xsens frame `imu`.

All reported matching numbers below have the label **SAME_SEQUENCE_SELF_MAP_ENGINEERING_ONLY**. They describe a 1.916-second, raw-LiDAR registration check against a fixed first-scan target. They are not full-corridor localization accuracy or independent-prior-map evidence.

## 1. Actual assets and storage

| Asset | Actual observation |
|---|---|
| Corridor1 external bag | 2,932,364,225 bytes; ROS bag V2.0; indexed; 81,618 indexed messages; 998 PointCloud2 and 40,281 Xsens IMU records |
| Corridor1 record interval | 1658917371.639781–1658917472.336074 s, duration 100.696292 s |
| Corridor1 verification limit | Header, index and topic metadata checked; not copied, not hashed or fully traversed in this task |
| Corridor2 internal bag | `/home/jian/ground-challenge/corridor2/corridor2.bag`, 2,871,922,599 bytes |
| Corridor2 SHA256, source and destination | `6ef85c76a36bdcef4811d411ac0f09219803377f5b7b9bd0cf2a5258cc826cea` |
| Corridor2 record interval | 1658917771.516684–1658917869.797716 s, duration about 98.281032 s |
| Corridor2 full raw traversal | All 79,658 messages read successfully; observed topic counts equal the bag index |
| External device | `/dev/sda2`, UUID `F627-4133`, mounted through `fuseblk` in this session |
| Transfer | New internal `.part`, rsync `--partial --append-verify --bwlimit=16384`; success, 175 s, reported about 15.61 MiB/s |
| Independent source hashing | Sequential read capped at 16 MiB/s; 171.181 s; same SHA256 as internal copy |
| Storage monitoring | No new matching USB reset/disconnect, UAS timeout, block I/O or filesystem error observed during the checks; an unrelated `kworker ... mt76` transient was not treated as a storage failure |
| Disk space | About 83 GiB available before staging, about 79 GiB after staging/build; no external writes |

Both bags' topics, timestamps, driver caller IDs and durations are consistent with the intended Ground-Challenge Corridor1/2 recordings, despite their misleading external parent folder `SuperLoc/Corridor02/raw`. No official publisher checksum was available; local SHA equality certifies the copy, not publisher-byte provenance. No bag was downloaded again. All calculations and ROS outputs used the internal disk.

## 2. Corridor2 sensor contract

- `/velodyne_points`: `sensor_msgs/PointCloud2`, 974 records, frame `sensor`, caller `/sensor_nodelet_manager`, timestamps strictly increasing from `1658917771606140375` to `1658917869740242958` ns. Median period 0.100857734 s. Raw counts 20,430–29,053 points.
- XYZ/intensity are FLOAT32, ring UINT16, time FLOAT32; point stride is 22 bytes and `time` offset is 18. Point times are finite; across the sequence their stored range is −0.10726594924926758 to +0.001306368038058281.
- These are **not** ordinary nonnegative start-relative offsets. No time rebasing or firing-time reconstruction was performed. The raw registration probe retained the original cloud bytes and header and did not interpret these offsets as a Deskew trajectory.
- `/imu/data`: `sensor_msgs/Imu`, 39,315 records, frame `imu`, caller `/xsens_driver`; strictly increasing from `1658917771516438961` to `1658917869797182083` ns. Median sample spacing 2.375126 ms; maximum 12.513876 ms.
- If the cloud's `time` is confirmed to mean header-relative seconds, the first cloud starts at `1658917771506630421` ns, before the first IMU by about 9.809 ms. The remaining 973 clouds have gross IMU interval coverage. This is an interval check, not a completed Deskew validation.
- `/camera/imu` contains a 299.015338421 s timestamp gap near the beginning. It was not selected as an alternative IMU.

The full input inspection remains at `/home/jian/livox_ws/p60_ground_challenge_runs/corridor2_input_contract.json`; its SHA is in `output_hashes.json`.

## 3. Missing extrinsic and map contract

The inspected official Ground-Challenge repository commit is `46a7c19a8672cc9246ad8b14815b4006a374fae0`. Its `calibration.txt` is 932 bytes, SHA256 `f8f591c46fa21c88d3986e65bf0558c7f79f2830b017165ec86b87190f745e08`. The file and supplied VINS/VIW configuration files contain camera/wheel relationships; no LiDAR–Xsens rigid transform was found. See `calibration_contract.md`.

The bag's complete TF inventory has 12 distinct parent/child edges: wheel joints, `odom -> base_footprint`, and `world -> map`. It has no edge touching `sensor` or `imu`, and no `/tf_static` topic. Thus the two sensor frames are disconnected. No Livox, SuperLoc or Newer College extrinsic was substituted.

No existing traceable Ground-Challenge prior map was found. No proof that Corridor1 and Corridor2 share a world frame or physical corridor was established. They were not combined. The bounded engineering target contains **only raw original scan 1**, 24,781 XYZ points, written without a coordinate transform and frozen before testing original scans 2–21. Production's unchanged two 0.15 m VoxelGrid passes produced 1,827 target points.

| Transform, `p_A = T_A_B p_B` | Status |
|---|---|
| `T_map_lidar` at raw map scan 1 | Identity by coordinate construction of this raw first-scan target; not an official world pose or a validated instantaneous Deskew frame |
| `T_imu_lidar` | **UNKNOWN**; no justified rotation or translation |
| `T_lidar_imu` | **UNKNOWN** |
| `T_map_imu = T_map_lidar * inverse(T_imu_lidar)` | Cannot be constructed |
| `T_map_pseudoGT` | Unconfirmed; not fitted |

This first-scan target is not a full-route map. The allowed self-map engineering path does not remove the need for a correct sensor transform in a fused, motion-compensated run.

## 4. Actual production NDT execution

The unchanged executable `dog_prior_map_ndt_node_cpp` was rebuilt in the isolated P60 worktree from the frozen production sources. Its SHA256 is `5a04df7401b58e8f1ac60e278e4085589bf9ac5d50b1a50b8cfe725323b85085`.

The optimizer/preprocessing configuration retained source voxel 0.25 m, cap 1,400, range 0.5–80 m, two target voxel passes at 0.15 m, NDT resolution 0.8 m, step 0.08, 40-iteration cap, epsilon 0.001 and existing 0.5 m/5° step limits. Source counts were actually 968–1,082, not 1,400 every frame.

The bounded probe deliberately has **no IMU prior, no Deskew, no EKF/OOSM**. Initialization is identity in the constructed map coordinates, then the existing NDT previous-pose/increment initial-guess path. It uses no GT. Therefore it is a production matcher check, not a claim that the full frozen fused CONTROL chain ran.

Actual call chain in `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`:

- `loadMap()`, lines 1620–1653: existing map conversion, target preprocessing and parameter setup.
- `pointCloud2Callback()`, lines 1997–2002: existing XYZ conversion; raw header used as a registration label, no point-time compensation.
- `handleCloudLocked()`, lines 2090–2115: existing initial guess.
- Lines 2295–2309: `setInputSource`, **one `ndt_.align` per input**, convergence, fitness and raw transformation.
- Lines 2328–2349: existing read-only native derivative/eigenbasis diagnostic.
- Lines 2550–2572: existing final-used state and real `/dog_livo/ndt_odom` publication.

The publisher sent exactly original scans **2–21**, with no duplicates or gap, and waited for the corresponding real NDT odometry stamp before sending the next frame. All 20 original PointCloud2 data SHA256 and timestamps match the input inspection. Scan 1 was not counted as matching itself.

| Measured item | Result |
|---|---:|
| Sensor interval | `1658917771706929684`–`1658917773623294353` ns, 1.916364669 s |
| Real align calls / output / converged | 20 / 20 / 20 |
| Geometry gate passes / step-limited frames | 20 / 0 |
| PCL fitness mean / max | 1.272658228 / 1.921943408 m² |
| Iterations | 1 for each tested frame |
| Align mean / P95 / max | 3.034667 / 3.475593 / 5.992570 ms |
| Post-match NN median, mean of frame medians | 0.060187746 m |
| Post-match NN P90, mean of frame P90s | 0.528286734 m |
| 0.5 m overlap mean / minimum | 89.922504% / 87.360595% |
| Maximum adjacent estimated translation | 0.019362803 m |
| NDT maximum RSS | 61,332 KiB = 59.894531 MiB |
| NDT CPU user / system | 0.27 / 0.09 s |
| NDT process wall time | 2.42 s, including startup/shutdown |

The RSS applies only to this tiny first-scan target, not to a full-map fused deployment. The runtime is a bounded ACK-paced diagnostic, not a 10 Hz real-time certification.

## 5. What the Hessian does and does not show

All 20 native derivative evaluations were valid and their normalized Hessians positive definite. The existing map-product chart is `p=p0+0.8*eta_t`, `R=Exp(eta_r)*R0`, with rotation in radians. It differentiates negative native PCL score per source point, with the existing nonlinear chart pullback. It is not assumed to be a Fisher information matrix.

`lambda_min/lambda_max` ranged from 0.009711641 to 0.011259565; condition number ranged from 88.813375 to 102.969214. The existing R6 relative split would label two low-curvature directions in all 20 cases. This arithmetic split does **not** certify two-dimensional geometric degeneracy. In the first tested frame, the weakest chart eigenvector is approximately `[0.06636, 0.04967, 0.95064, 0.29757, 0.02886, 0.00627]`; it is not evidence that corridor-axis drift has been identified.

Read-only diagnostic native jets: 20. Refinement jet/value calls, extra aligns, Coupled attempts, Recovery and candidate feedback: all zero. Hessian eigenvalues/bases and all initial/raw/used poses are archived. No failure basin, nonlocal ambiguity or severe long-corridor failure was established by this short probe. Small changes in an estimated trajectory are not position-accuracy evidence.

## 6. Accuracy, limits and audit

Translation RMSE/P95/Max, rotation RMSE/P95/Max and 1-second reference RPE are **not evaluated**, rather than zero. No pseudo-GT pose was parsed, selected, used for map construction, initialization, feedback or evaluation. The metadata repository checkout includes its published `psudo_gt` directory, but those files were not opened for pose analysis. No whole-trajectory alignment was performed.

One initial launch failed to resolve the publisher outside the ROS package; **zero scans/aligns** occurred. That output folder was preserved. Moving the publisher into the package and stripping ROS remap arguments fixed the launch; a package lookup and remapping regression test passed. The subsequent single 20-frame registration execution is the reported run. No optimizer setting was tuned. After it, a fail-closed map/bag identity guard was added to the publisher and unit-tested; it was not an additional matching experiment. `output_hashes.json` records immutable run outputs and the helper/config files as present during summarization; those are not claimed to be pre-execution publisher hashes.

Four focused adapter regressions pass. The actual 20-frame timestamp/data-hash parity check passes. The rebuilt production node succeeds. Production NDT, EKF, Deskew and frontend mathematics have no diff from baseline `a93d41d46ac60adc04552d76a09861e08f7ed87d`. No raw ROS environment logs are included in Git. Archived publisher CSVs use LF line endings; immutable original run CSV SHA256 remain in `output_hashes.json`.

## 7. Git and next step

- START_SHA / production algorithm source: `a93d41d46ac60adc04552d76a09861e08f7ed87d`.
- CODE_SHA (adapter archive commit, created after execution): `959365743368c3ebe2f4d9897738ec2c1a8dbc27`.
- Branch: `research/p60-r2-ground-challenge-real-run` in `/home/jian/livox_ws/p60_ground_challenge_ws`.
- Remote baseline when checked: `a93d41d46ac60adc04552d76a09861e08f7ed87d`; the new P60 branch did not yet exist remotely.
- PUSH_EXECUTED: **NO**. END_SHA is provided in the final handoff after the report commit; a commit cannot embed its own resulting hash.
- Existing dirty research/stable workspaces were not edited. Commits use the actual user name/email previously provided by the user, per command only; global Git configuration was not changed.

**Unique NEXT:** obtain a traceable full LiDAR–Xsens transform, with source/target axes and direction, for this Ground-Challenge rig. Then the controller can authorize a properly timed Deskew/fused Smoke and the permitted no-GT frozen-map preparation. No automatic next experiment was started.
