# P23 frozen bounded prototype contract

START_SHA: 805515a59df230f0742c91543c42dd3600b16bd3.
One full original Visual capture replay, then at most eight independent
offline frames and 24 complete offline alignments. No tuning, old-grid call,
EKF/P21 fallback, runtime Coupled feedback, GT proposal input or repeated replay.

Capture exact original frame IDs 2,284,293,296,308,347,368,396; resolve full
P22 archived timestamps uniquely against raw bag to obtain uint64 ROS ns,
then match ns+frame exactly in node. Only default-false opt-in IO after
setInputSource; preserve ordered XYZ and cloud dimensions/density, record
FNV and SHA256. Save original matrices directly, not quaternion reconstruction.
Pose file: LE float64 row-major, five 4x4 matrices ordered previous_used,
initial_guess,imu_rotation_prior,raw_ndt,final_used (640 bytes).
Refuse overwrite; partial files on IO failure are not valid captures.
Map/parameters/diagnostic association come from input freeze and actual ROS
parameter snapshot. Original map/bag/visual hashes must match.

Require 396 NDT/converged/OOSM/corrected,8004 high-rate,169 Visual,zero Late/
nonfinite/overflow, source hashes/counts exactly matching real diagnostics.
P19 full physical parity: translation<=1e-9m, rotation<=1e-8deg,
fitness<=1e-12*max(1,abs(score)), timestamps<=1e-12s; velocities/discrete exact.
Legal callback/wall times are timing evidence, not physical parity.
Allow only capture output parameters and disabled new P21 default in ROS
parameter normalization; NDT rebuild permitted, EKF unchanged.
No follow-up replay if parity/capture fails. >=4 valid sources required.

Offline map preparation and Step Limit must equal production. A uses exact
captured initial/previous matrices and original NDT settings. Predeclared
A parity: translation<=1e-5m, orientation<=1e-4deg, fitness<=1e-6*max(1,abs(score));
iterations/converged/limit flags exact. No tolerance changes. Freeze before GT.

Reuse ObservablePclNdt/nativeJet/dynamicScore and existing coupled local math,
chart p=p0+0.8*eta_t,R=Exp(eta_r)*R0,E=-raw_score/N. Nominal raw anchors chart.
Require finite/symmetric Hessian and positive curvature; original weak count
lambda1/lambda0>=2 gives1 else2. Explore ONLY eigenvector0 (log uncovered second
weak dimension). No valid positive model: retain nominal, do not fabricate weak.

alpha=0.999*min(0.20/(0.8*||w_t||), (2*pi/180)/||w_r||), infinite terms ignored.
Use only +alpha,-alpha and nominal. Strict raw score maximization, ties retain
nominal then positive then negative; no GT. B/C use SAME chosen weak coordinate.
For nonzero chosen weak step: original jointJet at zero plus existing strong
solver with zeroed gradient produces Hvu predictor (cap0.10). At actual predicted
endpoint jointJet with full J/K/g*K, one fixed-u residual strong step with du=0.
Final total strong displacement capped0.10; failed coupling falls back to B.
No double counting Hvu at residual stage, no Newton loop. Same weak coordinate.
Score diagnostics never authorize GT-based rollback or another proposal.

A/B/C each <=1 align, same params/Step Limit, no state propagation. Log value,
jet and align counts separately (internal NDT iterations not explicit jet calls).
Report preparation/map-memory separately; incremental method timing includes
all explicit objective/jet/chart/eigen/solve and complete candidate refinement,
not just matrix solve. mean<=30ms/P95<=50ms engineering goal.

All candidate/results hashes frozen before raw GT. Original ControlV2 fixed
0–3s SE(3) and interpolation, IMU origin, prefix-relative drift. Frame2 is
diagnostic outside primary; seven chosen challenge frames are not random/general
validation. Raw gain>=0.10m and worsening>=0.20m descriptive thresholds.
C vs B >=2 primary gains and no new>=0.20m risk is preliminary advancement
reference; accuracy and compute gates independent. Stop after bounded task.
