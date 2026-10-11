# P17 partial execution receipt

FINAL_RESULT: **VISUAL_INPUT_GENERATED_CAUSAL_REPLAY_BLOCKED_STORAGE_IO**

The requested visual-feedback comparison is **not completed**. This is an
external map-storage failure, not a visual-localization scientific failure.
No P17 accuracy advantage is claimed; no raw GT was opened.

## Completed

- Independent sensor-local depth generated from the frozen P10 V2 timed
  LiDAR and raw IMU; no old P6 depth or 482 visual measurements were reused.
- Unchanged P4 MEI/KLT/FB/depth-PnP produced 396 records, 169 valid pairs out
  of 395 possible pairs (**42.7848%**). Invalid: 211 insufficient depth
  correspondences, 14 image-to-scan lag failures, one missing reference,
  one missing completed past scan.
- Fixed evaluation risk windows: **35/59** valid pairs at 6–12 s,
  **0/40** at 26–30 s, **0/48** at 30–35 s. Late windows lack depth
  correspondences (plus four lag failures at 26–30 s). This is a real input
  coverage limitation, not evidence about visual GT accuracy or complementarity.
- Valid-row extraction: mean **74.499 ms**, P95 **84.215 ms**, max
  **251.563 ms**. Depth transport alone mean **54.716 ms**, P95 **59.542 ms**.
  This includes the declared gyro-only depth approximation and short endpoint
  translation bridge. It is offline precomputation, not realtime deployment.
- Valid PnP reprojection RMSE mean **0.562 px**, max **1.035 px**; this is not
  translation/rotation GT error. Proposed scan displacement mean **0.199 m**,
  max **0.374 m**. Oldest depth points can be **201.659 ms** before the image.
- Default-off visual initial-guess adapter and read-only shared-backend
  geometry diagnostics compiled in Release. No NDT parameter, EKF/OOSM,
  velocity blend, Anchor, R6 math or step-limit change.
- Five Python time/causality/frame tests; C++ guess/fallback/future/NaN/stale
  depth checks; two-frame NDT readonly exact parity; existing fixed-lag,
  deskew, frame conversion and PCL objective tests passed.

## Actual ROS attempt and why it is invalid

Loopback TCP works. The isolated short CONTROL launch processed **44 raw
scans through Deskew** and published **899 high-rate IMU states**, but
**0 NDT measurements and 0 OOSM applications**. The NDT process was registered
with ROS while still blocked loading the unchanged external normalized map.
The initial runner incorrectly regarded registration as readiness. Its old
zero-exit receipt is preserved but explicitly **not a successful baseline**.

The repaired runner checks NDT's actual IMU/deskew subscriptions (created
after `loadMap`), both node PIDs, and output frame count. It does not change
localization math. No second replay was started against the still-failing
disk. All task-owned ROS processes have stopped.

Kernel evidence identifies `/dev/sda` USB/UAS resets, commands waiting
**180 seconds**, and `blk_update_request: I/O error ... (READ)`. HIKVISION was
mounted fuseblk `rw`; this is not a sandbox/socket or chmod problem. No same
Corridor normalized-map PCD copy was found on the local research filesystem.
No remount, privilege bypass, alternate map or regeneration was attempted.
The camera full-bag hash recheck blocked after `visual.csv` was already
written; that task process was terminated. Historical raw-bag SHA is retained,
not falsely represented as rechecked. The final extractor avoids unnecessary
full-bag rereads and retains that explicit historical-hash provenance.

## Not evaluated

Actual visual-assisted NDT frames: **0**. Full visual-feedback replay, real
Control parity, initial-guess/raw/EKF error changes, NDT weak-direction
complementarity, trajectory jumps and full-system performance: **NOT_RUN**.
Visual peak RSS was not captured; sampled RSS was 199132 KiB, not a peak.
Failed-start NDT/EKF resource figures do not represent loaded-map performance.

For context only, frozen historical Control V2 primary 3–35 s translation
RMSE/P95/max is **1.887877/5.011039/5.667157 m**; rotation is
**3.026246/3.705965/16.393782 deg**. These are not new P17 scores. The original
Control 0–3 s full-pose transform must be shared unchanged by any resumed
evaluation, which remains prefix-aligned relative drift, not absolute ATE.

## Handoff

Source milestone: `4ec8b30d136bf86cd6969670b8e47afca7513315`.
Start/verified remote: `5453aec8ad6158d41716ca3b6c5f805f1381c96d`.
PUSH_EXECUTED: NO. Main dirty workspace and stable robot workspace untouched.
Independent review fixes: camera sensor-header monotonicity, caller-side
causal IMU clipping, depth-age bounds, actual node readiness/liveness, and
default-off diagnostic `is_open()` guard. No external-model CLI was used.

Unique NEXT: **RESTORE_FROZEN_MAP_STORAGE_AND_RESUME_P17_REPLAY**.
Resume using the saved visual CSV, verify the same map SHA, complete disabled
short parity and enabled short smoke, then the one fixed full visual replay.
Do not retune visual, NDT, Anchor or EKF parameters based on these partial
extraction statistics. No decision on formal visual-EKF fusion or originality
is supported yet.

Archive text is normalized to LF with line-end whitespace removed for
`git diff --check`. The raw cache is untouched; parsed CSV field parity and
updated archive hashes were checked. This is formatting, not altered results.
