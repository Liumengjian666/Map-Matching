# P15 fixed causal accumulation input gate

START_SHA: 01fee6a8d1743cb86b236e872ee5452993e2a9f2.
Queries fixed before encoding: TX16, TX45, TX708, TX1891, TX1994, TX4691.
N1 is the immutable P14 subset, NOT rerun. Exactly one N5 and one N10 run,
same author official P14 configuration, min32 correspondences, no GICP/GT.

Reuse unchanged external author commit 515759c00fe1e2f6d4d4a005e8c3a6b3f71d17ac.
Call original `inlier/eval/submaps.py::build_submap` with consecutive scans
ordered current to oldest; its first-frame anchor is therefore current.
Both reference and query use this same causal suffix definition. Remove raw
zero/no-return and nonfinite points BEFORE transformation, preserving order.
Any cleaned empty member, gap>0.15s or incomplete suffix is rejected explicitly.
No silent author skip, zero pose padding or future member. Keep original reference
IDs even when an input is rejected. Each condition builds its DB once.

## Explicit approximate pose contract

Loop2 logger stores map_T_imu/body at lio_time. Current documented R_imu_lidar=I
is NOT certified as the active historical calibration. This diagnostic uses
body pose as approximate LiDAR pose, omits the unknown historical lever arm,
and selects last logged state <= member scan end, age <=0.25s. No extrinsic
is invented or claimed recovered. Per-member original timestamps and full
matrices are saved. Missing state/window is a rejection, not a stationary guess.

Loop3 uses the existing P11 NDT final_used map_T_lidar for the corresponding
current/past scans. It is a non-GT local measurement history, not independent
truth. Original deskew was disabled; midpoint pose applied to raw timed scan
is an explicit approximation. Do not use later high-rate EKF/OOSM states.
No member point time or pose timestamp exceeds current scan end. This is
causality in sensor-data time, not a claim about online wall-clock scheduling.

The point transform remains the author `inverse(T_current)*T_member`, not
raw concatenation across sensor frames. This gate can test whether available
approximate accumulated inputs support geometry, not prove exact MID-360
calibration or independent correct localization. N1 used old single PCDs;
N5/N10 use raw bag inputs and may exclude boundary references. Record those
counts; no claim of identical reference support across the three conditions.

## Timing and stop

Preparation indexed-read/clean/load/write is offline and separately recorded.
Per-query accumulation is measured from loaded member arrays through original
author transformation/concatenation. Report that plus actual query core chain
with PCD I/O, preprocessing, encoding, MINT, BEAM and top20 Verify. Combining
these SAME QUERY phase times excludes bag extraction and is labeled so; it is
not a measured ROS end-to-end latency. Failures at correspondence gate are
not successful geometry acceleration. Positive MINT AND BEAM is recognition
evidence; zero-score padded IDs are not. Verify success must be finite rigid
query-to-reference T_sensor, not the default failure identity.

After both fixed-six conditions: if no success, stop InLiER direction under
the available input contract; do not tune, expand, accumulate more, build a
descriptor, run5817 frames or add feedback. If success exists, report actual
cost and cases, still without accuracy claims. ACCURACY_GAIN_NOT_EVALUATED.
