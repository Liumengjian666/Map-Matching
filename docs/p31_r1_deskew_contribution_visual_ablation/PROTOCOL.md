# P31-R1 frozen execution contract

START_SHA: eedcc2228158ee88bbd5a6b2bd2a84f809ae2a0c.
Existing research branch and P10 workspace only. Protected workspaces and all
historical archives/caches stay untouched. No push without user authorization.

Historical A/B are P29 captures, C is P30 full; none is replayed.
D: MSCKF ON, historical-node queries, Deskew-local copied config sets midpoint
input and continuous gravity-alignment heuristic false. Never mutate the
global IMU config, gravity vector, filter states, or stored history.
E: unchanged P30 scan-local path, MSCKF measurement update OFF only; mature
camera-time adapter/clones, nominal IMU, square-root and OOSM remain enabled.
Default-off switches preserve LEGACY_HISTORY; conflicting switches reject.

Run budget fixed before evaluation: one D21s, then only if numerical/time/runtime
gate passes, one D40s and one E40s. No A/B/C full rerun, no extra candidate
strategy, noise/gate/parameter search or GT parameter choice. One targeted
D-short repair allowed only on real contract failure, not accuracy outcomes.
Reuse frozen map/bag/features/calibration and NDT/DCReg/Multi/MSCKF/noise,
velocity_blend=0.6, Step Limit and fixed-lag150ms. Same -O2 -g compiler flags;
NDT binary unchanged, rebuild EKF in a separate /tmp devel tree only.

Minimal mode tests: local flags/global immutability, historical exact-node
correction retained by D, causal partial head input, C equality to P30 config,
default behavior and mutual exclusion. Existing geometry/time/extrinsic math
unchanged. No repeated P30 mathematical suite or covariance forensic audit.

Gate full396 NDT/396 due NDT OOSM/8004 high-rate, short208/207/4198; actual
visual updates D>0/E=0, finite states/roots, no Late/future-IMU/lost due
transaction/new>2m adjacent jump. Compare actual ROS params against frozen B,
allow only prescribed Deskew/visual flags and output destinations.

Freeze complete raw outputs BEFORE GT. Use shared frozen Control V2 0–3s full
SE(3), unchanged interpolation, IMU reference point; no new fit. Report all
five groups in3–35,26–35,34–35s, translation/rotation RMSE/P95/Max, ~1s RPE and
state jumps separately. B→D, D→C and E→C are conditional closed-loop contrasts,
not additive independent causal contribution fractions. No new posterior
acceptance threshold; do not claim tail drift or deployment solved.

Measure actual Deskew/NDT/MSCKF timings and per-process CPU/RSS. Visual OFF
update timing is unmeasured (count0), not free0ms. P28 offline frontend cost
retains its provenance; no full online camera10Hz/100MiB claim.
