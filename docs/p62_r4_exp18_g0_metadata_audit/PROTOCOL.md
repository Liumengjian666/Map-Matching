# P62-R4 — read-only G0 metadata audit

Baseline `1b53e7a4164c1e2871c077a4a6f1362e8c228a23`; R3 local and queried
remote match, R3 worktree clean. New sparse worktree, no R3/stable workspace changes.
Five asset identities have passed `assets.py before` before any substantive decoding.

First preflight: approximately 2026-10-19 12:15 UTC. Global hard stop 14:14 UTC;
parsing alarm at 14:13 reserves 60 seconds for failure receipts. Startup free
34,101,174,272 bytes >=30 GiB. Runtime guard preserves 20 GiB plus 1 MiB receipts.
No large pointcloud export or whole-scan map arrays permitted.

Full sequential bag deserialization with all topics, exact serialized-message hashes,
integer header and record stamps, all pointcloud schemas and encoded point-time ranges.
Unit interpretation must be independently audited: provisional absolute-second
arithmetic is explicitly labeled hypothetical, never a formal coverage PASS.
IMU bracketing uses the full point interval, not cropped headers. Do not sort away
duplicate/nonmonotonic input timestamps. Quantiles: NumPy linear interpolation.

E57 low-level `libe57.ImageFile(path,'r')` only. Do not use `pye57.E57` constructor
because its exception path may remove its input. Decode all point fields in fixed
100,000-point buffers, track count, nonfinite values and stored invalid flags.
Apply E57 stored pose to compute root-frame bounds; never equate root frame to GT world.
Use existing R2 pye57 0.4.5 isolated target; no new installation, no production environment change.

Both official GT files may be read for schema/time/reference eligibility; count 2
unique semantic GT inputs, not zero. No ATE/RPE, registration or trajectory estimation.
Official platform synchronization claims and actual field consistency are separate evidence.

No rosbag play, ROS localization nodes, NDT, EKF, GT fit, bias fit, clock search,
production C++ edits, or push. Primary and coexisting blockers must be preserved.
Final input SHA snapshots must match before local archive commit.
