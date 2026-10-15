# P40 — Newer College 01_short_experiment acquisition

Status: **SELECTED_LIDAR_GT_CALIBRATION_PRIOR_BUNDLE_DOWNLOADED_AND_VERIFIED**.

The selected sequence's LiDAR, GT, calibration, IMU CSVs and official 5cm prior
are downloaded and checked. This is not a claim that every camera/raw-packet/
alternative-map product has been downloaded, or that localization has run.

## Scope

Only download and integrity/data-definition checks. No NDT, localization,
registration, algorithm changes, SuperLoc edits or historical cache deletion.
All dataset payloads reside on the external HIKVISION filesystem:

`/media/jian/HIKVISION/paper rosbag/NewerCollege/`

The external filesystem is exFAT (mounted by fuseblk), UUID `F627-4133`, device
observed `/dev/sda2`, rw, initially about 326 GiB free. It supports large files.
The destination was absent before this task; no preexisting downloads were
overwritten. The tool pins cwd to the external filesystem, rejects symlink and
cross-filesystem destinations, serializes writers, preserves `.part` files,
uses curl resume and checks sizes before computing local hashes.

## Final inventory

31 official downloaded files: **19,370,094,732 bytes**. Ten ZIPs were extracted
without rewriting PCD contents; every ZIP passed CRC during extraction. PCD
payload totals **29,144,801,280 bytes**. Retained compressed plus extracted data
and supporting assets occupy about **48 GiB** by `du -sh`. External free space
afterwards is **277.475 GiB** (rounded `df -h`: 278G).

| Asset | Actual result |
| --- | --- |
| Ouster | 15,302 original PCD files; **15,301 unique scan timestamps** |
| PCD organization | Every file: 1024×64, 65,536 points, binary PCD |
| Unique-scan points | 1,002,766,336; counting duplicate files gives 1,002,831,872 |
| Scan period covered | `1583836591182590976`–`1583838121156355584 ns` |
| Scan span | 1529.973764608 s |
| GT | 15,301 records, **15,301 exact timestamp matches** to unique scans |
| Ouster IMU | 153,014 records; 1530.129787838 s |
| RealSense IMU | 382,114 records; 1530.159365654 s |
| Time correction | 49 records in official `time_offsets.csv` |
| Calibration | 15 files, including seven parseable YAMLs, JSON and Kalibr reports |
| Prior | Official 5cm full-area PLY, 359,695,676 bytes, 10,277,007 vertices |

Exact paths, file sizes and all 31 local SHA256 values are in
[download_manifest.json](download_manifest.json). Full directory summary:
[dataset_layout.txt](dataset_layout.txt). Dataset fields and small-file checks:
[data_definition_checks.json](data_definition_checks.json).

### GT integrity

Official `registered_poses.csv`: 1,330,464 bytes, 15,301 pose records.

- First timestamp: `1583836591182590976 ns`.
- Last timestamp: `1583838121156355584 ns`.
- Span: `1529.973764608 s`.
- SHA256: `b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e`.
- Nine CSV columns, finite values and strictly increasing integer timestamps
  checked. This hash is local, not an official checksum.

### Official duplicate preserved

ZIP009 contains both `cloud_1583837971_358315008.pcd` and
`cloud_1583837971_358315008(1).pcd`. They have identical size, CRC and SHA256:
`206d99d4f693318a609c1f5a16d349eb94083264572751a6d33a90ce89e0dd93`.
Both original names/payloads remain present. No dataset file was deleted or
deduplicated. A future reader must not count that timestamp twice; the unique
scan count is not the raw file count. Evidence:
[duplicate_scan_check.json](duplicate_scan_check.json).

## Reproduction

The live per-file manifest is on the external drive:

`downloads/download_status.json`

It changes only after a file passes byte-size and local SHA256 verification.
All 31 entries completed, with no download `.part` left. The planned scope and exact remote IDs
are in [download_plan.json](download_plan.json); provenance is in
[download_sources.md](download_sources.md).

Re-run safely from the research worktree to resume (existing final files are
validated, never downloaded over):

```bash
python3 docs/p40_newer_college_download/download_assets.py /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p40_newer_college_download/download_plan.json
```

Small offline tool tests:

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p40_newer_college_download
python3 -m unittest -v test_download_assets.py
```

All **8** tests passed. They cover size/hash, HTML/ZIP rejection, mount and
symlink guards, mount replacement, safe archive members and plan uniqueness.

Extraction command used, only after full receipt/hash validation and free-space
admission (it intentionally refuses an already existing extracted directory):

```bash
python3 docs/p40_newer_college_download/prepare_pointclouds.py
```

[pointcloud_validation.json](pointcloud_validation.json) records all ten CRC
results, every PCD header's common schema, counts and timestamps. Extraction
plus header checks took 365.376 s, **excluding** the preceding receipt/hash
revalidation. No online performance or localization claim follows from this.

One transient HTTP 503 occurred for a calibration report; bounded retry
subsequently downloaded and validated it. One initial extraction admission
stopped before writing PCDs because the official `(1)` filename was unexpected.
The parser was adapted to preserve that exact official suffix, tested and
re-run. No extra data downloads or filename/content changes were made.
There is no authentication bypass.

## Data definitions and next-task boundary

Official raw PCD scans are **not motion corrected**. Calibration includes
Ouster LiDAR↔IMU transforms, camera↔Ouster IMU transforms, camera intrinsics,
distortion and timing calibration. All PCD headers have
`x y z intensity t reflectivity ring noise range`; `t` is unsigned 32-bit.
An actual scan has 1,024 distinct times, spanning 0–99,879,680; source/value
agreement with the official group's driver supports scan-relative nanoseconds.
Details and driver references are in `download_sources.md`. These PCDs have not
been deskewed by this task.

Official GT documentation locates poses at the stereo setup's RS_C1 reference;
the official URDF repository additionally distinguishes its robotics-convention
base axes from optical camera axes. Do not assume this CSV is already an IMU
or LiDAR trajectory. Official GT construction registers Ouster scans to the
independently captured Leica prior. Exact operational reference conversion is
a next-task sensor adaptation contract, not established by this download task.

The official map README confirms the selected file is the downsampled 5cm
Leica prior. Its header reports binary little-endian PLY and 10,277,007 vertices.
This is **PLY**, not a PCD file; no conversion or point-coordinate change was
performed.

No publisher SHA256 was found. Local hashes plus CRC are recorded as such;
this is not an official hash authentication claim. The archive retains the
official filenames and the unmodified map, calibration, GT and scan data.

The data are ready for the **next input-adaptation task**, not an already
validated NDT run. The NDT loader needs an explicit PLY/PCD input adapter or
separately authorized format conversion; GT camera/base↔LiDAR reference,
point-time interpretation and non-oracle initialization must be set in that
task. Camera image ZIPs have not been downloaded, so this bundle alone does
not establish readiness for a full visual-fusion experiment.

Only download tools, tests and small reports/manifests were added to Git.
NDT, EKF, IMU, CONTROL, old SuperLoc data and protected worktrees were not
changed. NDT align=0, localization replay=0, registration/search=0.

An independent bounded read-only review found path/status-file guards and
resume accounting issues; these were fixed and covered by targeted tests.
The user requested an additional Codex CLI review. Its exact invocation and
input SHA256 `788cd6f0719f14fba91be5ca38d5c5a0732dbf3be96883f8e068f8bf30063d43`
were presented separately; **execution is awaiting that exact-command
confirmation**, not claimed completed here. No external CLI invocation has
been made without confirmation.

## Git

Repository: `https://github.com/Liumengjian666/Map-Matching`.
Branch: `research/p9-r4-heldout-visual-evidence`.
START_SHA: `4a0c2ac90296638f89b21aff7e1bc3896ea50447`.
CODE_SHA: `2ac40b702e662b3a9cf7452b728931e67aac5808`.
END_SHA: the commit containing this final report; full SHA is in final receipt
(`git log -1 --format=%H -- docs/p40_newer_college_download/REPORT.md`).
REMOTE_HEAD rechecked after downloads (unchanged):
`6382346e2b407fae382b56cbb0643394d1391968`.
PUSH_EXECUTED: false. No statement that P40 is present remotely.

Unique NEXT: obtain the decision AI's contract for this dataset's sensor/input
adapter and non-oracle initialization. Do not start localization or another
algorithm experiment automatically.
