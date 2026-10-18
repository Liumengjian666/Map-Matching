# P61-R6 pre-registered causal initialization bridge

START_SHA `610f74e6503edad2c3bbf4eb865f0076b0d447eb`.
R5 map SHA256 `4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4`.
Test-start exact time `1706671587829840123` ns.

## Input and causality

Use only internal original bag, first complete LiDAR interval through test start, and external Xsens `/imu/data` measurements with header <= test start. All admitted serialized LiDAR messages must match the R2 ledger. This admits scans1–2984. Scan2442 is allowed in this **temporary state bridge**, but remains excluded from the immutable R5 map. Scan2985 straddles the test-start boundary and is excluded whole. Last admitted raw point:1706671587799724734ns; final IMU-only propagation spans about30.115389ms to the exact test boundary.

Physically create a bridge-only bag. No GT, built-in IMU or test LiDAR is delivered to the bridge. The author time model remains ROS header+offset_ns; time_sync=false and offset=0. All source/configuration/voxel/filter/noise/extrinsic settings remain R5 values.

## Execution gates

Start one fresh, isolated FAST-LIO instance/master/output directory. Preserve the complete native filter and IMU-process state by replaying from scan1, not restoring CSV.
After real ACK2441, pause **before** scan2442 and compare the recorded LiDAR pose against R5 at the same consumed scan-end state. Direct differences, no alignment: translation<=0.20m AND rotation<=2deg. Failure stops bridge certification and later playback. On success, process543 additional complete scans, through2984, using only that instance's temporary mapping state.

After the final scan ACK, publish only remaining measured IMUs <=test start. A read-only-arrival check must verify the exact final IMU header has reached the node before requesting the terminal propagation. No future sample is used for interpolation. Use the author's existing integration loop and its final held pair-average input via a shared helper; do not introduce CV, new dynamics, synthetic scans or a new filter. Record actual filter covariance as well as state. Freeze initialization and hashes **before** opening any test LiDAR for geometry.

The bridge's temporary map is not the prior map and is never substituted for R5 PCD. SHA-check the R5 PCD before/after.

## Bounded geometry probe

Primary deliverable is causal `T_M_L(test_start)`, not its independently certified accuracy. After freeze only, use the first fully test-window scan (2986) for one fixed raw-native LiDAR single-frame NDT probe against the unchanged R5 map. At most **one align**. Resolution0.8m actual Gaussian leaf size, step0.08, epsilon0.001,40 iterations,outlier0.55; inherited source preparation0.25m voxel,0.5–80m range and1400 point cap, target preparation0.15m twice. These values are verified from the actual inherited `cloud_contract.hpp`, not assumed from Newer College or mapper settings. No result-dependent changes.
The initializer time, raw scan start and scan end differ; report these intervals. The probe is explicitly **raw, not deskewed** and does not certify dynamic/fused localization. Any pose correction belongs only to the diagnostic output and never overwrites the earlier initialization. NN thresholds fixed in advance:0.5m,1m,2m; report median/P90 and valid/source counts, not accuracy.

Do not process the full test segment merely to fill50 coverage rows. Keep the R5 fixed50 indices; without same-frame pose estimates at those times, keep POINTCLOUD_COVERAGE_NOT_CERTIFIED. No GT evaluation, full localization, Coupled, Recovery or parameter search.

Record source/binary/config/input/state SHA, actual data counts/time bounds, complete bridge trajectory, covariance, map consistency, resources and all failures. One actual-root-cause engineering repair is allowed if runtime fails; no scientific retuning. Internal disk reserve>=20GiB. No external-disk access, no changes to protected workspaces or R5 files.
