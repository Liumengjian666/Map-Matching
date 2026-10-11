# Causal input adapter, not a new descriptor

Author: `build_submap(load_scan, scan_files, poses, window)` defines its
reference as `window[0]`, then transforms member k with
`T_reference_member = inverse(T_world_reference)*T_world_member`.
P15 passes exactly N consecutive scans, newest/current first. Thus both DB
and queries express all points in the current scan's approximate sensor frame
without future members. Descriptor, MINT, BEAM and Verify are unchanged author
C++ algorithms. No scalar parameter or geometric minimum is weakened.

Raw sensor no-return `(0,0,0)` must be removed before transforming: otherwise
inter-frame translation moves an invalid zero into a finite fake surface.
Nonfinite points are also removed. After assembly P14's original first-point
0.5m voxel and official encoder settings remain the same. This is input-validity
compatibility, not a descriptor change. Empty members are explicitly rejected
because author's accumulator otherwise silently omits them.

These data do not certify exact scan poses. Reference transformation uses
mapping body pose under an explicit unverified body/LiDAR alignment assumption,
with unknown lever arm omitted. Query transformation uses recorded non-GT NDT
measurements, whose drift and midpoint/time mismatch can deform accumulated
structure. No pointwise motion deskew or true-extrinsic claim is added.
Actual timestamp/pose ledgers make these approximations visible. A failed gate
does not prove that properly calibrated/deskewed InLiER can never work.

Positive retrieval evidence requires BOTH MINT>0 and BEAM>0 for the same ID.
Author returns padded zero-score IDs: those may still be run through Verify
to preserve the original pipeline, but are not counted as retrieval evidence.
`n_correspondences >=32` is the frozen author RANSAC entry threshold.
Failure identity `T_sensor` is not an estimate. Success must be finite rigid;
its direction is `p_reference = T_sensor*p_query`. This is not map_T_lidar
or an independently correct place label. No position precision is evaluated.

N1 is historical P14 six-query subset, with all389 single references. N5/N10
have the same causal assembly but may omit boundary references lacking pose
history. Consequently N1 vs accumulated results diagnose available input
support, not an isolated mathematical change with identical database support.
Time from loaded-member author assembly and time from accumulated PCD through
complete C++ query are separately measured, not added into a fabricated actual
ROS end-to-end latency. Offline indexed extraction/setup RSS is separate.
