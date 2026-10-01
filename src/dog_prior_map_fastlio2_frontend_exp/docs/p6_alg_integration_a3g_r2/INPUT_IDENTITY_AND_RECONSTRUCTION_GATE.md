# Input identity and strict deskew reconstruction gate

## Identity verification

The entry state was clean at the required revision:

- branch: `research/p6-i6d-full-algorithm`
- HEAD: `283be8cf8298e24200099c4a1e51eb3ed1c2d0f7`
- frozen run ledger: `EXTERNAL_RUN_FILES.json`, SHA256 `4992c4f56fa49e2b8837f57733b9edc2bde51efb997e6c475a716be35a3cdf05`
- ledger validation: all 22 external entries matched both expected SHA256 and byte size.

The relevant frozen input hashes also matched the A3G identity:

| Input | SHA256 |
|---|---|
| SuperLoc Corridor01 `raw_timed_points.bin` | `4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff` |
| `raw_timed_catalog.csv` | `fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f` |
| `filter_scans.csv` | `41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d` |
| `RAW_TIMED_INPUT_MANIFEST.txt` | `fb20125c63aa4c7110851b8bb08ec33e8938d94be18e28e9737d207001377575` |
| normalized prior map | `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f` |
| official calibration parameters | `7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d` |
| Corridor01 IMU CSV | `7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa` |

The three directly inspected selected-run evidence files retained their frozen hashes:

- `trajectory.csv.r1_preopt_capsule.csv`: `47850cf860886ac7fb0ddb2ecdf52de201b54541e730bd7c76da8b2a0e93618b`
- `trajectory.csv.deskew_evidence.csv`: `81493d06f8e8b6ce4e2834f92c05e30e7620cc11c60f1d6f6e4c449bd06fc350`
- `trajectory.csv.a3g_health.csv`: `c35427464625a2880335d2f32b8cd9be93533944dd2182c1213832d4cf2803db`

Before the deskew stop, the 19 explicitly allowlisted raw records were read by catalog offset using the frozen 40-byte record layout. For every selected transaction, binary record count, actual point timestamp minimum/maximum, catalog scan interval, and the logged deskew evidence count/extrema agreed exactly; details are in `RAW_SCAN_RECORD_GATE.csv`. This validates raw-record identity and timing only. It does not validate deskewed coordinates or authorize map-overlap calculations.

## Why the geometry gate did not pass

Production obtains the scan-start anchor from `adapter.activeStateAt(raw.scan_start_ns, &anchor)` and calls `deskewScanWithWindowState(anchor, ...)` in `scripts/p6_a2c_fixed_lag_producer.hpp`. `WindowState` is 15D: rotation, position, velocity, gyro bias, and accelerometer bias (`include/dog_prior_map_fastlio2_frontend_exp/window_factors.hpp`). The deskew implementation integrates the causal IMU trajectory from that complete anchor and uses it to transform every timed point (`src/window_scan_processor.cpp`).

The frozen A3G result artifacts do not preserve the inputs needed to reproduce those exact transforms:

- `trajectory.csv` contains terminal position and quaternion only.
- `trajectory.csv.deskew_evidence.csv` contains scan bounds, point count and time extrema, anchor/end position and quaternion, displacement aggregates, and provenance. It contains neither point coordinates after deskew nor the per-knot IMU pose trajectory.
- `trajectory.csv.r1_preopt_capsule.csv` records predicted/terminal pose and factor diagnostics, not the scan-start velocity or bias components.
- No state snapshot, point-level deskew cloud, or equivalent saved deskew transform sequence is present in the frozen run directory. The other indexed run files are diagnostic summaries/traces, not those missing operands.

The raw timed scan and IMU samples alone do not determine the optimized scan-start velocity and biases. Inferring them from adjacent pose rows, re-running window optimization, or substituting a synthetic constant-velocity/bias model would violate the strict reconstruction requirement. A count and displacement summary is not enough to reconstruct or validate the point coordinates.

The static source path was also checked: the producer calls `geometricObservations(source, nominal.pose.cast<float>(), 0.8)` only after NDT convergence (`scripts/p6_a2c_fixed_lag_producer.hpp`); the implementation radius-searches target voxel cells at the supplied resolution and emits observations for finite leaves/distances (`scripts/p6_i1_branched_recovery.cpp`). `analyzeGeometricObservability()` then rejects invalid weights/fields or unusable voxel covariance/information and labels zero valid observations as `NO_VALID_GEOMETRIC_CORRESPONDENCES` (`src/reliability_metrics.cpp`). These source semantics were not invoked on reconstructed geometry in this stage, so no layer counts are reported.

Therefore no point cloud was deskewed offline, no map point/voxel search was run, no geometric observation was rebuilt, and no production-support equivalence was claimed. This is `STRICT_RECONSTRUCTION_INPUTS_MISSING`, not `OFFLINE_GEOMETRY_REPRODUCTION_MISMATCH`.

An adversarial review checked whether endpoint poses plus the frozen IMU could recover the missing anchor state. They cannot do so under the task contract: the logged end pose does not uniquely determine the anchor velocity and accelerometer bias (and the exact optimized state is not otherwise saved); inferring these from adjacent poses would be an estimated state, not the production state. The contract also does not give a numeric tolerance for the non-bitwise meaning of “strict” reproduction, but that ambiguity is not decisive here because no pointwise candidate output can be formed or compared.

## Scope and exclusions

- Selected transaction allowlist was not widened.
- No rosbag, full runner, P3 runner, or NDT alignment/optimizer was run.
- No global alignment, pose search, GT, ATE, or RPE was used.
- No map-overlap or support conclusions were drawn.
- Production code was not changed.
