# P42 — Newer College 2020 stereo image acquisition

**Result: `STEREO_IMAGE_BUNDLE_PARTIALLY_DOWNLOADED`.** The external drive and official dataset lineage were verified, and three complete left-camera archives were downloaded, CRC-checked and extracted. Google Drive's public download quota blocked the fourth left archive and the first right archive. The full sequence and stereo pair set are therefore not available yet; no quota bypass or alternate unauthorized access was attempted.

## Dataset and official source

This is the 2020 `2020-ouster-os1-64-realsense / 01_short_experiment` sequence, not the 2021 release. P40's recorded official directory lineage and file IDs are reused in [download_sources.md](../p40_newer_college_download/download_sources.md) and [download_plan.json](download_plan.json). The official [Stereo-Cam page](https://ori-drs.github.io/newer-college-dataset/stereo-cam/) identifies `/camera/infra1/image_rect_raw` as the 30 Hz left stream and `/camera/infra2/image_rect_raw` as the 30 Hz right stream, and describes the non-ROS images as timestamp-named PNG files. Its [stereo calibration page](https://ori-drs.github.io/newer-college-dataset/stereo-cam/calibration-stereo/) documents the sequence-specific camera/LiDAR clock correction used here.

The archive plan contains eight ZIPs with a listed total of **18,308,002,548 bytes**. Three left archives totaling **7,499,561,982 bytes** were obtained. No publisher SHA256 values were available; recorded hashes are local SHA256 values, not publisher-authenticated checksums. Each successful archive matched the observed official listing size and passed ZIP CRC validation.

## Storage and layout

The HIKVISION disk was mounted read/write as UUID `F627-4133` (`/dev/sda2`, exFAT) at `/media/jian/HIKVISION`. It had 482,407,350,272 bytes free before the P42 download and **465,042,407,424 bytes** free at final validation. This leaves **432,830,152,704 bytes** above the required 30 GiB reserve. Storage capacity was not the blocker.

Downloaded ZIPs are under:

`/media/jian/HIKVISION/paper rosbag/NewerCollege/downloads/stereo_camera/`

Extracted, original left PNGs are under:

`/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment/raw_format/infra1/`

There are **37,480** left PNGs, with **7,505,286,827 logical bytes** (9,863,692,288 bytes allocated by `du`). `infra2/` has no extracted right-camera frames. Existing P40 LiDAR, IMU, GT, calibration and map data were left untouched. The per-archive inventory, byte counts, local SHA256 values and preserved failed HTTP response hashes are in [download_manifest.json](download_manifest.json); the external-drive receipt is `downloads/p42_stereo_download_status.json`.

## Image integrity and coverage

All three available ZIPs passed CRC checks. Nine representative images (first, middle and last from each ZIP) decoded successfully as 848×480, one-channel `uint8` PNGs. The extracted filenames retain the official `infra1_<sec>_<nsec>.png` timestamps; no image was re-encoded and no timestamps were rewritten. The three archive ranges join in increasing time order without duplicate timestamps:

- First: `1583836591152386717 ns`
- Last: `1583837841149698525 ns`
- Span: **1249.997311808 s**
- Median image period: **0.03333634 s** (29.9973 Hz)
- Duplicate timestamps: **0**; observed intervals over 1.5× the median: **0**

This is partial left-camera time coverage, not the full approximately 1530-second sequence. Since no right images were obtained, left/right synchronization, pair counts, pairwise timestamp skew and stereo completeness cannot be evaluated; verified stereo pairs: **0**.

The available left-camera timestamp range overlaps the P40 LiDAR and RealSense IMU ranges. The official clock-offset table supports interpolation for **37,029 / 37,480** available left frames; the verifier did not extrapolate. The prescribed correction is `camera_corrected = camera_timestamp - time_offset_nsec`. Stored PNG timestamps remain unchanged. Exact overlap and source ranges are recorded in [image_validation.json](image_validation.json).

## Calibration availability and limits

The P40 acquisition already contains stereo Kalibr intrinsics, distortion, stereo transform, both camera-to-RealSense-IMU and camera-to-Ouster-IMU calibration chains, Ouster IMU/LiDAR transforms, and the sequence-specific camera/LiDAR clock-offset table. The concise values and file paths are in [stereo_calibration_summary.md](stereo_calibration_summary.md); source file hashes remain in the P40 manifest. This task did not reinterpret transform direction or synthesize a missing calibration. There is no Camera-LiDAR transform file asserted as a separately calibrated direct edge; a future adapter must compose a documented chain. The right image stream is still needed for stereo use.

## Download block and reproduction

The public Drive endpoint returned its quota-exceeded HTML page for `infra1_04.zip` (twice) and `infra2_01.zip`. The downloader rejected the response as non-ZIP content and preserved it as a `.part`/`.invalid_response.*` diagnostic instead of treating it as image data. The remaining two right-camera ZIPs were not attempted after the quota block. No user credentials, login, or quota-avoidance technique was used.

After public access becomes available again, the official full plan can be retried with the existing resumable downloader:

```bash
python3 /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p40_newer_college_download/download_assets.py /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p42_newer_college_stereo_download/download_plan.json downloads/p42_stereo_download_status.json
```

The current extraction script is deliberately fail-stop if a pre-existing extracted directory does not exactly equal the currently verified archive inventory. Because `infra1/` is presently partial, a later completion step must safely extend/reconcile extraction before claiming full image coverage.

Offline checks run in this task:

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p40_newer_college_download
python3 -m unittest -v test_download_assets.py
```

Result: **11/11 passed**. The P42 extraction/validation script passed `py_compile`, and its final validation completed with exit code 0. No VO, NDT, FAST-LIVO2, localization replay or algorithm changes were performed.

## Git handoff

Repository: `https://github.com/Liumengjian666/Map-Matching`

Branch: `research/p9-r4-heldout-visual-evidence`

START_SHA: `bcc58c1309cc5efac80ade02d32dc6901275e46c`

CODE_SHA: `d8eacf4691b1aa228f2b488aff481f0194f45ede` (download/extraction tooling commit).

END_SHA is the commit containing this report and is given in the final execution receipt. The remote branch was `bcc58c1309cc5efac80ade02d32dc6901275e46c` at final pre-commit check; P42 was not pushed. The stereo ZIPs and extracted PNGs remain only on the HIKVISION disk and are not Git artifacts.

**Final result:** `STEREO_IMAGE_BUNDLE_PARTIALLY_DOWNLOADED`. A left-camera partial was preserved and validated; complete stereo acquisition is blocked solely by the official public-download quota. Stop here pending a later authorized continuation or quota recovery.
