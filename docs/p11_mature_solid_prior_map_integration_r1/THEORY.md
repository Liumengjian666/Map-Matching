# Mature SOLiD reuse: scope and semantics

This integration is not a new descriptor, NDT solver, covariance model, or
Dual-U decision algorithm. It vendors the author's actual C++ implementation.

## Original implementation

`SOLiDModule::pt2rah` assigns range, azimuth and elevation bins.
`makeSolid` builds range–height counts R and angle–height counts A, and height
counts n. The original weight is

    w = (n - min(n)) / (max(n) - min(n))
    d_range = R * w
    d_angle = A * w

`loop_detection` computes cosine similarity of the first 40 (range) elements.
We retain this exact implementation, including the map descriptor structure.
Ranking is descending cosine with ascending reference ID for ties. Angle
descriptor/yaw estimation is not used as an accurate query pose.

## Compatibility, not innovation

MID-360's vertical FOV is 59 degrees, unlike the upstream default [-24.8,+2]
degrees. The author's README explicitly instructs users of another LiDAR to
change header parameters. We freeze [-7,+52] degrees and 40 m max range
before the independent query experiment. Bin counts, min3 m range and .4 m
voxel remain upstream values. Point/descriptor/normalizer guards reject
invalid input instead of inventing descriptor values or changing the equation.

The original profile remains separately measured. On initial real mapping
scans, most points saturate its last elevation bin; on empty input its original
normalizer yields NaN. These are compatibility/boundary facts, not proof that
the adapted descriptor recognizes places accurately.

## Database and coordinate boundary

References are real scans from an independent Loop2 offline mapping session,
paired to the exact active map's FAST-LIVO2 mapping-state pose file. The older
archived pose file is not used: despite largely matching timestamps, it differs
by up to 9.34 m from the active map's pose lineage. No localization GT creates
the database. Query recording Loop3 is distinct from Loop2 mapping.

The existing extractor pairs nearest raw scan header within .12 s, copies the
mapping-state pose and replaces its stored timestamp with that scan header.
Actual residuals and original state timestamps are archived. It does not
interpolate the pose or compensate the raw reference scan. FAST-LIVO2 source
stores `_state.pos_end`/`rot_end`, not a proven exact LiDAR-origin pose.
The API therefore names the matrix `map_T_reference`, not `map_T_lidar`.

These reference map locations are valid coarse mapping-index locations, not
ready-to-use NDT initial transforms. Historical physical LiDAR/IMU calibration
and precise scan-time reference transport remain limits if a future task uses
them for registration. This Shadow never does so.

## Main-chain isolation and experiment limitations

The optional independent ROS subscriber consumes `/dog_livo/filtered_points`:
the exact ordered preprocessed source also given to NDT. It publishes no pose,
observation or TF and shares no filter state. All original NDT/EKF/math files
are unchanged. The current frozen robot baseline has deskew disabled; this
task preserves it and does not mislabel the source as verified deskewed.

Structural isolation does not prove that async ROS timing is unchanged.
Full A/B outputs test source hashes, NDT poses, OOSM releases and EKF states.
Fixed-lag late arrivals prevent deterministic main-chain certification, even
if the retrieval node never changes an observation. Tolerances are not relaxed.

Returned references are Place Recognition Candidates. Separated reference
locations with high cosine can motivate investigation but do not prove distinct
NDT basins, ambiguity, correctness, probability, covariance, or accuracy gain.
No independent Loop3 map-frame place labels have been established, so recall,
precision and localization accuracy are NOT_EVALUATED. GT is never loaded.
