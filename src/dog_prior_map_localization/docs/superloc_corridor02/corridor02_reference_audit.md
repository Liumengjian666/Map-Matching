# Corridor02 reference semantics audit

## Evidence

The official SuperLoc page publishes Corridor02's trajectory in TUM field
order: `timestamp x y z q_x q_y q_z q_w`. It does not state whether the pose
is the IMU, LiDAR, camera, or base origin.

The official SuperOdom repository documents LiDAR/IMU extrinsic direction, but
contains no Corridor02 GT-generation or export code. The local initial-pose
file contains `extrinsicRotation_world_darpa` and
`extrinsicTranslation_world_darpa`, but does not establish whether these are
`T_world_sensor` or its inverse.

The ICCV LiDAR-inertial challenge page explicitly describes *that challenge's*
SubT-MRS trajectories as IMU poses, but Corridor02 is listed by the official
SuperLoc release as source `SuperLoc` rather than SubT-MRS. That separate
statement is not sufficient lineage evidence for Corridor02.

## Hard gate

GT_POSE_ORIGIN = UNRESOLVED

evidence_level = UNRESOLVED

map_world_transform = UNRESOLVED

relative evaluator valid = NO (authoritative oracle blocked)

No GT-relative oracle, absolute error, persistent-failure classification, or
cross-sequence replication claim may be produced until the origin is closed by
an official Corridor02 source/config/lineage artifact.
