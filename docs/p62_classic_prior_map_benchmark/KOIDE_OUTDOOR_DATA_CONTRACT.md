# Koide Outdoor input contract: official metadata prepared, actual payload not admitted

Only Outdoor01 and Outdoor02 are primary. No Indoor result is renamed, and no SuperLoc/Newer College asset expansion is performed. Official source: https://zenodo.org/records/10122133 (v1). Fresh metadata, bounded ZIP directory tails, metadata.yaml entries and CRC receipts are retained in sources/. No large sensor payload was downloaded this round.

## Actual asset identity

|Asset|Bytes|Official MD5|
|---|---:|---|
|outdoor_hard_01a.zip|1642263643|d126024c8f310c2c48239f136d7e0ed0|
|outdoor_hard_01b.zip|1345802065|d397c28cca763d34844189cb73de8e02|
|outdoor_hard_02a.zip|1507761853|8f56fdfa93f4fa456e234f58e193a08c|
|outdoor_hard_02b.zip|1298771600|267377a88402f825ab70a161cc48983c|
|map_outdoor_hard.ply|23780722|562b607124bc31bbfc4c96eaaf47bbfb|
|gt.zip (already available from R29)|1036441|ab2a80dc1d06767b7c3ce1893f3020b0|

Expected hashes are not actual payload verification. Actual Outdoor bag/map SHA and publisher-MD5 PASS remain pending. Existing outdoor GT file identities and timestamp-only inventories are in REFERENCE_TIME_INVENTORY.json; reference poses were not used to evaluate or select any new output.

|Metadata aggregate|Outdoor01 a+b|Outdoor02 a+b|
|---|---:|---:|
|/livox/lidar CustomMsg|5225|5206|
|/livox/points PointCloud2|5147|5127|
|/livox/imu Imu|136422|132454|
|Bag-record span, s|683.657820601|663.567522902|
|a/b bag-record gap, s|0.889904866|1.285481515|

These are ZIP metadata counts/times, not decoded header validation. Custom and PointCloud2 counts differ by 78/79. The player must not quietly choose the shorter successful subset, concatenate time across gaps or pretend that a missing/empty scan produced a measurement. Record-time gaps do not yet prove identical header-time gaps.

Primary adaptation shall preserve the complete CustomMsg stream, including declared empty/occluded input, and all actual IMU messages. Verify point numbers, XYZ, offset_time units, header/timebase relation and scan reference semantics from the actual messages before admitting it. Where both point-cloud encodings exist, cross-check deterministic equivalence rather than assuming equal count. Decode ROS2 CDR using the actual type definition; conversion may not modify numerical measurements or time.

## Coordinates and units

T_A_B maps B into A. M=official map, L=official cloud, I=official IMU. The dataset publishes T_L_I=[0,0,0,0,0,0,1] in xyz/xyzw for Outdoor. Thus T_I_L=inverse(T_L_I), T_M_L=T_GT and T_M_I=T_GT*T_L_I. The unit transform is official calibration, not an assumed equality chosen to minimize error. Runtime frame IDs and the published fields still require actual-payload verification.

Reference is TUM [time, xyz, xyzw] and denotes LiDAR pose in the map frame, produced by batch scan-registration and IMU optimization. No fitted SE3/time alignment is allowed. ROS header sec/nanosec and gyro rad/s, acceleration m/s² must be checked against raw data; intra-scan point timing must not reuse the Azure t=0 approximation. Extrinsic, gravity, state/output frames and scan-start/end conventions must close before NDT execution.

## Minimum-chain gate (not a new algorithm)

R30 acceptance must identify its START/CODE/END, diagnostic source, reproduction evidence and any independently verified minimal infrastructure repair. At minimum it must show lawful sensor conversion and time handling, coherent initialization/IMU propagation, correct map-to-LiDAR/IMU transforms, native NDT against the proper map, and timestamp-correct accepted measurement/OOSM use. A ROS process starting or a finite state is insufficient; R29 already demonstrates that distinction.

The controller explicitly confirmed R30 has not been executed and has no local task/report yet. No further R30 file search is needed. This round is authorized only for literature checks and data-contract preparation; **formal Outdoor localization and large sensor downloads are not authorized**. No R30 PASS is claimed and no repair is implemented. R30 minimum-chain acceptance and subsequent full experiments will be commissioned separately.

GT row counts are5147/5127, matching the metadata PointCloud2 counts and smaller than the5225/5206 CustomMsg counts. This is count agreement, not proven per-message timestamp identity. Preserve all input and all propagated output; explicitly mark reference-unavailable frames rather than silently shrinking the dataset.

Current official Livox CustomMsg/CustomPoint schema snapshots confirm timebase, per-point offset_time and XYZ metres. They do not identify the exact recording-driver build. The retained IMU-publisher excerpt copies acceleration/gyro fields without visible conversion in that function. Therefore a ROS Imu class alone does not certify raw acceleration SI units: actual raw values, upstream source/official sensor unit evidence and any required deterministic conversion must be checked before R30/Outdoor admission. No g-to-SI factor, clock offset or calibration is inferred from localization error.

## Storage

The four ZIPs total 5794599161 bytes; their declared expanded contents total11244273456 bytes. Root free at inspection was35279790080 bytes, so retaining all compressed+expanded copies would violate the20GiB reserve even before converted clouds/results. The existing external filesystem had449049264128 bytes free. Plan large assets under `/media/jian/HIKVISION/paper rosbag/koide_classic_prior_map/`, retain20GiB reserve, avoid a second full extracted-PCD copy if a verified streaming player suffices. This path is a plan, not a claim of downloaded files. No historical assets were deleted.
