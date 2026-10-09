# P10 Corridor01 trusted causal replay R1

FINAL_RESULT = `TRUSTED_P2B_CAUSAL_CONTROL_BLOCKED`

NEXT = `RESTORE_P2B_HISTORICAL_RUNTIME_LINEAGE`

## Decision

No new Control replay or NDT comparison was run. The trusted P2B input and
runtime contract cannot be reconstructed from the currently available
artifacts without substituting an unpinned adapter implementation or guessing
the P2B runtime configuration. The earlier R7-R2 lineage result remains in
force. This is an engineering/provenance stop, not a Corridor01 algorithm
failure.

`P2B` already has a historical deterministic receipt: its derived v1 bag was
reported as SHA256
`60048c6369ea1065cc4dee2484ff38360620fa6b7321abc05b71e874f2b06345`, with
2776 scan-start PointCloud2 messages and 55946 IMU messages. The R1 receipts
report 2776/2776 NDT source/initialization/result rows identical between Run C
and Run D. Those are historical findings, not results reproduced in this
task. The derived bag, Run C `result.bag`, and Run C `ndt_determinism.csv` are
absent at their recorded external paths.

The accepted `recovery_r2/REPORT.md` already records
`HISTORICAL_V1_LINEAGE_UNRECOVERABLE_IN_AVAILABLE_SOURCES`: the P2B manifest
pins a wrapper but not the shared implementation included by that wrapper;
only 7/10 directly listed adapter files still match, and no historical
transitive source snapshot was recovered. This turn confirmed the boundary:
the wrapper still hashes to the recorded value, but the current included
`superloc_sensor_adapter.cpp` is the later full-SE3 implementation with a
separate rotation-only diagnostic output. Its source hash is not present in
the historical P2B manifest. The launch, CMake, and package hashes also differ
from the historic manifest. A rotation-only output in today's code is not
proof that it is byte- or behavior-equivalent to the missing P2B source.

The frozen P2B paper commit `8e0c1661a2ddac1d669d752d3b0a7ad771cb7409` is
present in current Git history. Its NDT node uses a previous-pose/last-increment
initial guess and can apply a local IMU gyro rotation prior; the paired EKF
propagates IMU state and applies delayed NDT observations through its OOSM
path. However, the exact P2B R1 launch/runtime override is not archived with
the frozen commit. The committed default NDT YAML at that revision selects a
mid-scan reference, while the P2B adapter contract says its cloud is deskewed
to scan start. A later Corridor01 YAML was introduced with the subsequent
full-SE3 v2 work; it cannot be retroactively treated as the missing P2B R1
runtime receipt. This leaves a material timestamp/reference ambiguity, so
the historical chain cannot be safely rebuilt from source alone.

The available P2C `replay35.bag` is not a substitute. `rosbag info` reports
35.0 seconds and 346 cloud/6997 IMU messages, and the recording includes the
later adapter's `points_deskewed`, `points_rot_only`, motion diagnostics, and
NDT odometry topics. Its adapter log explicitly records that the first scan
was rejected for missing IMU coverage. It is an already processed P2C slice,
not the missing P2B scan-start v1 input, and it does not provide the required
5-second startup plus 35-second evaluation window.

## Prior P10 discrepancy

The P10 failure-onset paired diagnostic's Control prefix fit residual
(`5.8036 m` translation RMSE and about `175 deg` rotation) is not a valid
reproduction of P2B. Its own protocol identifies the run as standalone
scan-to-map diagnostic, not full IKFoM feedback. The map was normalized at the
first scan, but matching began at TX52 about five seconds later using the
stale identity first-scan prior rather than a causally propagated TX52 pose.
It also used the P9 scan-end source contract instead of P2B's scan-start
rotation-only cloud. These are sufficient reasons to reject that Control as
the trusted P2B reference. The archived evidence does not isolate the roughly
180-degree residual into one unique cause; a separate frame-convention defect
is not proven and must not be asserted.

## Historical metrics retained, not rerun

- Historical P2B PREFIX_10S first ten seconds: translation mean `0.1424 m`,
  P95 `0.2929 m`, max `0.3490 m`.
- Historical persistent drift crossings: `0.5 m` at `+11.2375 s`, `1.0 m` at
  `+13.1537 s`, and `2.0 m` at `+16.4819 s` after the evaluation origin.
- This task: `CONTROL_REPLAY = NOT_RUN`; `NDT_CALLS = 0`; `GT_LOADED = NO`;
  Nominal/Weak-only/Coupled comparison = `NOT_RUN`.

The evaluation origin remains the historical first NDT sensor stamp plus
five seconds (`1517157224.188979`). No new GT alignment or fit was calculated.

## Delivery scope

This is a documentation-only blocker receipt. No source, configuration,
historical archive, raw input, stable machine-dog workspace, or prior result
was modified. `CODE_SHA = NOT_APPLICABLE`. The containing commit is the Git
end SHA; no self-referential commit hash is embedded in this report.
