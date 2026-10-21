# Actual integer timestamp observations; no final pilot admission

Actual filename stamps: 15,301 strictly increasing unique scans, from
1583836591182590976 to1583838121156355584 ns. The actual reference contains
15,301 finite rows with the same exact integer timestamp set. This establishes
format-level zero-difference pairing, not physical synchronization or a statement
that the stamp is scan-end. The reference maximum quaternion norm error observed
in the format audit is6.941737649146518e-7; no pose error was calculated.

The user cap is50ms; no nearest-time substitution or interpolation was necessary
for the observed file-level timestamp pairs, and none was used for an experiment.
Formal selected21-frame matching remains NOT_RUN after the coordinate stop.

The data author states raw PCDs are not motion corrected. The current pinned
driver's ros.cpp assigns per-point `t` from column timestamp minus supplied
scan_ts, and lidar_scan.h uses chrono nanoseconds. The actual three header samples
store t as uint32. This does not identify the historical exporter/caller or prove
that supplied scan_ts equals the saved filename stamp. Do not claim authenticated
absolute point-time `filename+t`, and do not interpret t as scan-end-relative.

The camera/Ouster drift and camera correction on the official calibration page
do not authorize shifting these LiDAR-rate registered poses. No time-offset search,
gyro bias fit, synthetic per-column timing, IMU integration, or deskew was run.
SOCC's generated normalized column times are not a substitute for actual t.

Two full reference-format audits and one first-three-lines inspection occurred;
reference reads are NOT0. One full audit preceded receipt of the expanded user
contract. No reference pose was used for prediction, tuning or candidate choice;
no benchmark error was evaluated. Do not claim the file was entirely withheld.
