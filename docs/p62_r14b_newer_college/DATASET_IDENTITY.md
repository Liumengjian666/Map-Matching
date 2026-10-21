# Actual existing assets, no new sensor download

Only Newer College2020 `01_short_experiment` was inspected. Official provenance
was traced through the retained P40/P59-R3 source manifests and current official
website; no prior localization output or score was used for asset/frame selection.
Existing dataset root:

`/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/`

The user-authorized new-data destination remains
`/home/jian/rosbag/public_datasets/newer_college_r14b/`; no sensor assets were
downloaded, copied, or moved there, since sufficient local assets exist for the
input/source inspection. Old R14/M3DGR access was not attempted.

|Existing asset|Actual bytes|Actual SHA256|
|---|---:|---|
|ground_truth/registered_poses.csv|1330464|b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e|
|prior_map/new-college-29-01-2020-5cm-resolution.ply|359695676|e8bf6c7f2ddacde4f6f65ba7759b00bdca5c6bf5e706344ce8c3592529f04ab0|
|prior_map/prior_map_xyz_binary.pcd (historical derivative)|369976348|57da358a7c3f52a132fd06afba69f2c3eff2e980b7dbad5b8751548665104e25|
|calibration/kalibr_output/cam-ouster-imu/camchain-ouster_imu-cam-rooster_2020-03-11-10-05-35_0.yaml|1736|3bc13fa24af9ef8ca888c5523b1df57d1c53d0ca4052b818b7e02310fa781feb|
|calibration/kalibr_output/ouster_imu_lidar_transforms.yaml|300|8fe56ff808c6c900db52ac1545819ef3da979818bfa392ba00aef43daf59a037|

ASSET_MANIFEST.json lists all20 actual inspected side assets and3 sampled PCD
hashes, with pre/post verification. All19 side files having historical expected
identities matched. The derived PCD has no expected identity in that download
manifest; it is not certified as a new lossless conversion in this task.

LOCAL_SCAN_INVENTORY.csv inventories15,301 actual canonical PCD paths, with total
logical size29,142,896,640 bytes. This is a filename/size inventory, not SHA or
decode verification of all scans. Only3 chronological header probes were hashed;
they are not the selected pilot. Their noncanonical length remains explicit.
The first sample also matches the existing ZIP entry SHA; no publisher SHA list
was found, so historical locally computed hashes must not be called publisher
authentication. Existing ZIP archives were not all rehashed or re-extracted.

SOCC-ICP data preparation matches the existing `raw_format/ouster_scan` plus
`ground_truth/registered_poses.csv` structure. Only its small source files were
downloaded for inspection, not installed or executed. Sources/manifest.json gives
actual URLs, pinned code commits, bytes and SHA. The website pages are actual
current snapshots, not a claim about a historical website revision.

Initial filesystem available space:91,429,003,264 bytes. Final actual df output
is STORAGE_FINAL.txt. Minimum reserve20GiB was never approached. The roughly1.5GB
new Git worktree checkout is distinct from dataset payload; there was no cleanup,
large download, map conversion or second dataset. Existing PCD bytes are not
newly allocated R14B sensor storage.

Dataset use remains subject to the author's CC BY-NC-SA4.0/noncommercial terms.
