# P30-R1 scan-local IMU Deskew feedback isolation

START_SHA: 9ed197900419660845e41eebce603606852ae126.
One research branch and the existing P10 workspace. Protected robot/paper
workspaces and P28/P29 archives/cache are not modified.

Sole physical intervention: `deskew/experimental_scan_local_imu_enable=true`,
default false. Existing relative geometry `(T_ref*T_il)^-1*(T_point*T_il)`
does not change. Freeze map/bag/features/calibration/initialization and all
NDT/DCReg/Multi-NDT/MSCKF/noise/OOSM/velocity_blend parameters from P29 B.
NDT executable remains exactly P29; rebuild only the EKF containing Deskew.
Same compiler flags as P29; standalone isolated devel directory.

New motion path: copy one causal EKF snapshot at/before scan_start; advance
its p,v,R,ba,bg to start with raw head-sample IMU, then integrate once through
actual IMU boundaries until scan_end. Retain P28 effective causal ZOH model:
right rotation exponential, end-rotation acceleration, frozen map gravity
and biases, without its unused legacy gravity-alignment heuristic. No tail
sample values after point time are used, including the final scan fragment.
Point queries use immutable continuous nodes, not subsequent EKF pose samples.
This isolates intra-scan corrected-node resets, NOT prior visual effects on
the legal anchor. It does not guarantee equal A/B Source.

Directed tests: stationary, constant velocity/angular rate, combined motion
vs fine-step reference, common-left SE(3) including gravity/velocity transform,
future-IMU and later-EKF poisoning, query order, partial causal anchor,
out-of-order/duplicate rejection. Existing Deskew contract test verifies the
untouched disabled path. No additional default-off ROS replay is budgeted;
default-off binary-level trajectory parity is NOT claimed.

One 21-second C prefix on MSCKF ON; capture the same bounded eight P29 requests
(only frames7/193 lie within the prefix). No GT is opened. Require expected
P29 prefix sensor timeline, all due fixed-lag transactions, real nonzero MSCKF
updates, finite states/roots, no future IMU, no new >2m adjacent-state jumps.
Compare ordered raw/deskew points, actual captured Source, Initial/Raw/Used,
and same-current-history legacy/local relative motion diagnostics. Do not pair
preprocessed Source point indexes when voxel membership/order changes.

Only if short mathematical/runtime gate passes and motion differences are
explained: one full40s C capture, no A/B reruns. Require396 NDT/396 due NDT
OOSM/8004 high-rate, finite outputs and actual visual updates. Any timing,
nonfinite, unexplained new severe jump or runtime fault stops further replay.
No noise/threshold/strategy search or repair experiments after full C.

Freeze all complete outputs and Sources BEFORE GT. Formal evaluation reuses
Control V2 fixed0–3s full SE(3), same interpolation/IMU origin, windows3–35,
26–35 and34–35s. Compare P29 B vs C, A only historical reference. Include
approximately1s RPE, actual intervals and adjacent-state jumps separately.
No refitting/selecting samples using GT. Correctness/feedback isolation and
accuracy gains are distinct. Point/relative-motion differences alone do not
prove real motion accuracy or a unique root cause.

Record Deskew/NDT/MSCKF mean/P95/max, per-process CPU/RSS, extra diagnostic
cost and build/input hashes. Offline feature extraction cost uses measured P28
provenance, not a free online-camera claim. No 10Hz/100MiB claim.
