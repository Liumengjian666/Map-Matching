# P28-R2 — mature visual measurement replacement

Start: `237ae40fa993eba7331ff15cbdf3f14fc583832c`, existing research branch.
No protected workspace writes, GT online use, parameter search or upstream GPL
source copying. The map is frozen XYZ-only: OpenVINS-style feature/MSCKF
mathematics are independently adapted; this is NOT official OpenVINS/SKF reproduction.

## Frozen inputs and measurement construction

Same SSD map, V2 IMU/LiDAR bag, initialization, IMU noise numerical parameters,
NDT/DCReg/Multi-NDT strategy and velocity blend 0.6. Raw Corridor01 camera images
and original MEI calibration produce perspective-rectified FAST/KLT pixel tracks.
No PnP, LiDAR depths, visual.csv motion estimates, pseudo-absolute visual poses
or CI enter the new measurement path. Pixel sigma 1 px and per-track 95% chi-square
gate are fixed before GT, following the standard feature/MSCKF model.

Each image header must be <= its associated scan and <=50 ms old. Repeated/stale
images are skipped, not substituted. Tracks close after at most five views or
loss, >=3 views required. At most 40 accepted features per update. A feature's
closed observations are consumed once. Landmark-only triangulation uses the
published bounded distance/parallax/condition safeguards; left-nullspace
projection removes the landmark state. Live pixel residual and FEJ pose
Jacobians are used, followed by orthogonal measurement compression.

## Shared time/covariance adapter (explicitly approved by user)

Both A and B use causal head/ZOH IMU inputs, exact image-time event subdivision,
right/body attitude-error linearization and up to 11 pose clones. This differs
from historical whole-interval midpoint integration; it is not an A/B factor.
The 15D IMU core remains, with temporary 6D pose clones and one 6D held IMU
measurement-noise driver per genuine IMU interval. Partial intervals and their
remainder use the SAME driver and its cross covariance; bias random walks use
independent increments. The held driver is marginalized at the genuine tail,
never reset at the image. Zero-rank clone covariance is legal; no jitter or
PSD clipping. Joint factors, FEJ poses, driver activity/head and raw IMU inputs
are preserved in OOSM snapshots. Core S is only a marginal derived from the
joint factor. NDT damping scales joint core rows by the original DPD factors.

DCReg axes must come from a received report whose full scan support end is
<=image time, at most 200 ms old. Its existing 0.03 criterion is unchanged.
The visual update uses the real H/weak-projector overlap and constrained gain,
with square-root Joseph positive-term update, no fixed CI. Only weak pose
directions are corrected; velocity/bias and noise-driver gain rows remain zero
(a conservative constrained-gain adaptation, not full official SKF information
projection). Right-attitude injection/reset applies to core and clones.
Original fixed-lag scheduler (.15 s), NDT correction and IMU replay remain;
camera transactions and NDT transactions are sorted by measurement sensor time.

## Run budget and gates

First: directed projection/Jacobian, triangulation/nullspace, clone/cross-noise,
square-root and OOSM tests. Then one 19 s B smoke crossing +18.839069 s.
It must process NDT/OOSM, execute nonzero genuine reprojection updates, maintain
finite factors/states, no new >2 m adjacent jump and no exponential covariance
growth. If a specific implementation fault is found, one concentrated repair
and short retest is allowed. Otherwise no repeated smoke/parameter search.

Only if smoke passes: one full A, one full B (40 s each). A: new update OFF;
B: new update ON, weak-direction selection ON. Both old vision paths OFF,
shared time/covariance adapter ON. No full P19/P24 reference rerun.
Expected full 396 NDT, 396 NDT OOSM and 8004 high-rate states, zero nonfinite.
Freeze outputs and hashes BEFORE loading GT. Use frozen Control V2 0–3 s SE(3),
IMU origin, same interpolation; evaluate 3–35, 26–35 and 34–35 s. No separate fit.
This is prefix-aligned relative drift, not absolute map ATE.

Offline feature extraction costs include measured camera read, rectification and
tracking wall time/RSS; real-time camera/frontend scheduling is not validated.
No accuracy or online-integration claim may be made for tests alone or zero
updates. Negative results are retained. Stop after this task.
