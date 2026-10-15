# P40 — Newer College 01_short_experiment acquisition

Status: **DOWNLOAD_IN_PROGRESS**. This document is not a completion claim.

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

## Confirmed first result

Official `registered_poses.csv`: 1,330,464 bytes, 15,301 pose records.

- First timestamp: `1583836591182590976 ns`.
- Last timestamp: `1583838121156355584 ns`.
- Span: `1529.973764608 s`.
- SHA256: `b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e`.
- Nine CSV columns, finite values and strictly increasing integer timestamps
  checked. This hash is local, not an official checksum.

## Progress and reproduction

The live per-file manifest is on the external drive:

`downloads/download_status.json`

It changes only after a file passes byte-size and local SHA256 verification.
`.part` files are not marked complete. The planned scope and exact remote IDs
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

One transient HTTP 503 occurred for a calibration report; bounded retry
subsequently downloaded and validated it. There is no authentication bypass.

## Data definitions and remaining gates

Official raw PCD scans are **not motion corrected**. Calibration includes
Ouster LiDAR↔IMU transforms, camera↔Ouster IMU transforms, camera intrinsics,
distortion and timing calibration. Per-point time fields still require actual
PCD header inspection once the first ZIP is complete.

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

Pending: completion of all ten LiDAR ZIPs, CRC checks, actual scan/time-field
inventory, final per-file manifest/layout and final disk-space check. Do not
claim readiness for a full localization experiment until those gates finish.

## Git

Repository: `https://github.com/Liumengjian666/Map-Matching`.
Branch: `research/p9-r4-heldout-visual-evidence`.
START_SHA: `4a0c2ac90296638f89b21aff7e1bc3896ea50447`.
END_SHA: pending task closeout.
REMOTE_HEAD last observed before downloads:
`6382346e2b407fae382b56cbb0643394d1391968`.
PUSH_EXECUTED: false. No statement that P40 is present remotely.
