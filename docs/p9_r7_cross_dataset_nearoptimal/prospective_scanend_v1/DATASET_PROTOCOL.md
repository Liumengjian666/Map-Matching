# P9_CORRIDOR01_RAW_SCANEND_V1

Prospective protocol; **not historical v1 equivalence**. Frozen before extraction.

## Raw conversion

One complete pass over the official raw bag, SHA256
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.
Only `/velodyne_packets` and `/imu/data` are consumed. Bag inventory:
2777 raw scans and 55957 raw IMU messages. Old adapter counts are not the target.

Compile `rawdata.cc` and `calibration.cc` directly from clean Velodyne commit
`29abd0e1361cb7f5eda451d2b51c35eeca45e0d5`, with its headers taking precedence.
Use pinned VLP16db calibration, no TF, no adapter, no deskew, no map registration.
`HAVE_NEW_YAMLCPP` matches the pinned project's build for installed yaml-cpp 0.6.2.
Decode the full angular/range packet contents; actual P9 range/voxel filtering remains
downstream and unchanged. Reject nonfinite/zero returns with explicit counts.
The parser's own azimuth-overflow omissions are counted as `not_emitted`.

The point parser runs with **packet-local** origin, so its float firing time contains
only the <=1.307 ms firing offset, not the whole scan duration. Its value rounds to
the pinned 24-firing x 16-laser schedule: `f*55296 + d*2304` ns. Add the exact
uint64 packet header stamp. Check packet stamp against the embedded sensor
microsecond clock modulo one hour, within 1000 ns (one clock tick, not label tuned).
Accept only the observed VLP16 single-strongest-return mode. Other modes fail closed.

Scan start is the first packet's sensor timestamp, equal to the scan header.
Scan end is the final packet timestamp plus 1306368 ns, the final scheduled firing,
whether or not its range return is usable. No assumed 100 ms scan duration.
Overlaps are logged; points remain in parser order. P7's existing `prepareScanWindow`
must later handle overlap relative to the committed state. All raw scans retain IDs.

P7 packed layout is little-endian float32 XYZ + uint32 offset_ns (16 bytes).
`filter_scans.csv` stamps are scan end, not bag-record times. Offsets index contiguous
binary bytes. No points are deskewed at extraction. Raw IMU numeric values and sensor
stamps remain in Epson coordinates without rotation; full serialized IMU payloads
are retained externally to preserve covariance/orientation provenance. They are not
used as an initialization orientation prior.

## Initialization admission BEFORE any nominal NDT

Read actual sensor-only init and normalized map provenance. The init file describes
a pose referenced to the first scan but obtained from 50 scans over five seconds.
It is not by itself a velocity, gravity, bias, or current-time IMU state. Backdating
this estimate to deskew earlier scans is not authorized.

The existing P7 implementation initializes with the first 200 causal IMU samples,
sample acceleration standard deviation <=0.50 m/s² and gyro standard deviation
<=0.05 rad/s per axis. It sets gyro bias to sample mean and velocity to zero, then
aligns specific force to map +Z. Passing variance alone does not prove rest.
Before using it, validate rest, map gravity convention and pose epoch. Failure is
an initialization blocker, not permission to fabricate zero velocity/bias.

No executable `params.txt` is emitted until initialization is accepted. If blocked,
archive this as NOT_CREATED rather than produce an apparently usable fake initial
state. No smoke/baseline run may bypass this gate.

## Intended baseline contract (conditional; not a claim of execution)

`T_map_lidar = T_map_imu * T_imu_lidar`; use the documented laser-to-IMU calibration
once, including its translation. Raw IMU stays unrotated. Rounded calibration
rotation must have a documented SO(3) treatment before baseline use.
Use existing P7 `ScanEndProcessor`, `prepareScanWindow`, IKFoM propagation and
`CurrentFrameNdtRegistration`; no new deskew algorithm. Map is the normalized
Corridor01 map, not Floor01. Never copy its target-point-count guard.

NDT stays 0.8 / 0.08 / 1e-5 / 80; map/target voxel 0.15 m, source voxel 0.25 m,
range 0.5–80 m, maximum source points 1400, minimum effective 50.
Source/T0/score/U_obs must share the same scan-end LiDAR frame and objective.
The frozen P9 chart is map-product, with its original curvature definition.

If admitted: smoke <=30 alignments, then <=2 full identical sequential replays.
Freeze comparison bounds now: source count/hash exact; position <=1e-5 m;
rotation <=1e-4 deg; score difference <=1e-6*max(1,abs(score)); eigenvalue
infinity-norm difference <=1e-6*max(1,max(abs(eigenvalues))); W2 projector
Frobenius difference <=1e-5 outside a relative eigenvalue-gap <=1e-6 degeneracy
region. Degenerate W2 is explicitly non-unique, never sign-compared columnwise.
Boundary frames must be retained in the processing ledger with reasons.

## Non-execution boundary

GT, oracle263, B12, visual extraction, candidate selection and R6 statistics are
out of scope. Preparation cost is not DUAL-U online incremental cost.
Persistent output is exclusive-create `Corridor01/results/p9_corridor01_raw_scanend_v1/`.
An extraction failure preserves partial artifacts and stops; no automatic retry.
