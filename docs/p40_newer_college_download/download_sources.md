# P40 official sources

Dataset: **Newer College 2020, Ouster OS1-64 + RealSense,
01_short_experiment**. This is not the 2021 OS0-128 / AlphaSense release.

- Official website: https://ori-drs.github.io/newer-college-dataset/
- Download entry: https://ori-drs.github.io/newer-college-dataset/download/
- Dataset and raw-format definitions: https://ori-drs.github.io/newer-college-dataset/stereo-cam/
- Ground-truth and Leica map construction: https://ori-drs.github.io/newer-college-dataset/ground-truth/
- Calibration and clock-drift correction: https://ori-drs.github.io/newer-college-dataset/stereo-cam/calibration-stereo/
- Platform reference frames: https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/
- Official robot description: https://github.com/ori-drs/frontier_description/tree/newer-college-dataset
- DRS Ouster point type: https://github.com/ori-drs/ouster_example/blob/master/ouster_ros/include/ouster_ros/point.h
- DRS Ouster scan-time assignment: https://github.com/ori-drs/ouster_example/blob/master/ouster_ros/src/ros.cpp
- DRS Ouster nanosecond timestamp type: https://github.com/ori-drs/ouster_example/blob/master/ouster_client/include/ouster/lidar_scan.h
- SOCC-ICP format reference only: https://github.com/josch14/socc_icp/blob/main/DATASETS.md

The user authorized submitting their supplied email to the official access form.
It was submitted once, with no login or CAPTCHA bypass. Neither the address nor
form submission data is included in this archive. The confirmation supplied the
current public shared directory below. Some old individual calibration links
on the website return 404; the files used here come from the current directory.

## Actual directory lineage

1. Official form result: https://drive.google.com/drive/folders/15lTH5osZzZlDpcW7oXfR_2t8TNssNARS
2. `2020-ouster-os1-64-realsense`: https://drive.google.com/drive/folders/1y3IWSYhHWnpUsleXE0q23lnj0WCVo9ig
3. `01_short_experiment`: https://drive.google.com/drive/folders/1VFwyApa7EEvN-pVTq67t1isbQNdAEfou
4. `ground_truth`: https://drive.google.com/drive/folders/14Hh135RkontHVJfHucJ2zV-EZ9MEAsbu
5. `raw_format`: https://drive.google.com/drive/folders/1BgZOWTzpiXt5O0gf0zmjeXvIxjAe5dG3
6. `ouster_zip_files`: https://drive.google.com/drive/folders/1NoFkQNqlGFIltvaXvouAbNp40JrkWIW5
7. `03_new_college_prior_map`: https://drive.google.com/drive/folders/19KryKoqjy3sfKs_kT1tdGAQW3JDImWDP
8. `04_calibration`: https://drive.google.com/drive/folders/1DYLZUHSX1FkNZ9VZLWI3rI4L-pB_Ahdl
9. `kalibr_output`: https://drive.google.com/drive/folders/1Ts63Ate0vxI-92qYzSNWwPCm9UfMYpXe

`download_plan.json` records every exact file ID, original name and byte size
read from these actual directory listings. A file's source page is
`https://drive.google.com/file/d/<drive_id>/view`. Public content links are
`https://drive.usercontent.google.com/uc?id=<drive_id>&export=download`.
For larger files the tool follows only the ordinary public Drive
`download-form` confirmation on `drive.usercontent.google.com`, not a login.

## Selected scope and estimated storage

The explicit plan contains 31 files, **19,370,094,732 bytes** of network payload.
The ten official Ouster ZIPs account for **18,914,926,892 bytes**. The selected
prior is the official full-area **5 cm PLY**, 359,695,676 bytes; it is not the
original 290-million-point E57, nor the recommended 1 cm filtered alternative.
The original filenames and payloads are preserved. Compressed archives are
retained. Extraction, if performed, must separately check the actual ZIP
uncompressed totals against free space before writing any scans.

Included small supporting data: both IMU CSVs, camera clock offsets, actual
Kalibr outputs/reports, beam intrinsics and map README. Excluded: camera image
ZIPs, duplicate sequence rosbags, calibration rosbags, alternate map resolutions,
mesh, E57, all other sequences. Thus this is the complete selected LiDAR/GT/
calibration/5cm-map bundle, not every sensor product on the website.

No publisher SHA256 list was found. Download validation uses the actual public
listing byte sizes and **locally computed** SHA256; that is not publisher hash
authentication. ZIP CRC checks and dataset field inspection are separate gates.
The website distributes the data under CC BY-NC-SA 4.0; future use must retain
the dataset attribution and comply with the noncommercial/share-alike terms.

## Bounded sensor-definition findings

The actual first ZIP has binary PCD fields
`x y z intensity t reflectivity ring noise range`, with `t` an unsigned 32-bit
field. One actual scan has 65,536 points (1024×64), 1,024 distinct timestamps,
and `t` range 0–99,879,680. The official group's driver assigns the nanosecond
column timestamp minus the scan timestamp to this field (`ros.cpp:77–82`,
`lidar_scan.h:34` defines `ts_t` as `std::chrono::nanoseconds`). The exact
historical PCD exporter commit was not identified. This source/value agreement
supports interpreting `t` as scan-relative nanoseconds; the download does not
deskew or otherwise rewrite it. The archived PCD field name is `noise`, whereas
the currently visible driver calls that field `ambient`; do not rename it here.

Official GT generation registers Ouster scans to the independently acquired
Leica prior, rather than constructing an oracle map from this test trajectory.
The stereo GT page identifies RS_C1, while the official description README
specifies a left-camera-position base with robotics axes. Sensor-reference and
axis conversion must be applied explicitly in a future input adapter. This task
does not fit any map↔GT alignment or certify an untested pose-frame adapter.
