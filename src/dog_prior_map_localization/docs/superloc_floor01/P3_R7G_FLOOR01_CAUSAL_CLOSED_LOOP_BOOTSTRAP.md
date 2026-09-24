# PAPER-P3-R7G: Floor01 causal closed-loop deskew bootstrap

## Scope and boundary

This stage is a 30-second engineering smoke only. It does not evaluate ATE/RPE,
localization accuracy, failure time, or cross-sequence performance. No NDT,
EKF, predictor, limiter, deskew runtime source, map, raw bag, GT, or visual
module was modified.

The frozen implementation is the paper workspace NDT binary built from the
existing baseline lineage. The external adapter uses the already validated
official VLP16 decoder and Floor01 calibration. Its translation source is the
two most recent completed NDT LiDAR poses, transformed to the IMU origin with
the fixed laser/IMU extrinsic. It does not use GT, external odometry,
accelerometer position integration, FAST-LIO, or SuperOdom.

## Operational initialization

Only the official `multi-floor01.yaml` matrix was considered, with the two
finite directions required by the protocol:

* H1: released matrix used directly;
* H2: inverse of the released matrix.

The fixed map-normalization procedure produced an H1 map whose bounding box is
local to the normalized origin (`x≈[-63.17,198.07]`, `y≈[-32.62,155.22]`,
`z≈[-5.05,29.46]`). H2 produces a displaced map (`y≈[-376.04,-110.16]`). A
first-scan NDT smoke with H1 was finite and converged (`fitness≈0.331637`, one
iteration, finite near-origin pose). H2 yielded a non-useful identity result
with `fitness≈3.56e4` and zero iterations. Therefore H1 was selected only as

`OPERATIONAL_INIT_SELECTED_BY_MAP_CONSISTENCY`.

This does **not** confirm the official semantic direction of the released
matrix. No GT pose value was read or supplied to runtime.

## Closed-loop dataflow

```text
raw Velodyne packets + IMU
  -> official VLP16 packet decoder
  -> causal full-SE(3) adapter
  -> frozen NDT
  -> completed NDT odometry history
  -> next scan's deskew
```

Scan 0 and scan 1 use `zero_translation_bootstrap` while still applying IMU
gyro rotation deskew. Scan 2 and all later scans use
`two_prior_odom_poses_cv`. For scan `k≥2`, only completed pose stamps from
`k-2` and `k-1` are eligible; both are strictly earlier than the current scan
stamp. The current scan's NDT pose is appended only after NDT has processed that
scan and is therefore available only to future scans.

## Smoke results

Run A and Run B each produced 297 common NDT outputs over 29.8528779 seconds.
Run A had 297 accepted adapter scans. Run B had one initial scan rejected for
insufficient IMU coverage before the same 297 accepted common scans; no scan in
the common sequence was extrapolated.

For both runs:

* bootstrap scans: 2;
* CV scans: 295;
* deskew success for the common sequence: 297/297;
* NDT finite/converged outputs: 297/297;
* translation and rotation step limits: 0 activations;
* strict prior-stamp causality violations: 0;
* current-scan or future-pose use: 0.

The maximum observed CV velocity was approximately 1.77935 m/s, below the
adapter's 12 m/s operational guard. The maximum translation over one scan was
approximately 0.17943 m. Detailed frame records, causality ledger, and
statistics are stored outside Git under:

`/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_closed_loop_smoke/`.

## A/B repeatability

The 297 common scan stamps, serialized deskewed-cloud SHA256 values, NDT source
cloud hashes, translation modes, NDT initial-guess fields, fitness values, and
final NDT poses match between Run A and Run B. The maximum translation-pose
difference is 0 m and the maximum quaternion-derived rotation difference is
`2.96e-6 deg` (floating-point representation only). No branch divergence was
observed.

## Decision

`CAUSAL_CLOSED_LOOP_BOOTSTRAP_VALIDATED`

`FLOOR01_CLOSED_LOOP_READY_FOR_FULL_RUN = YES`

This is an engineering readiness result only. The full 417-second Floor01 run,
GT performance, initialization dependence, and failure mechanism remain
unevaluated. P4 is not allowed by this stage.
