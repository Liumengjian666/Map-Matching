# R7-R3: prospective raw input prepared; causal initialization blocked

FINAL_RESULT = `CORRIDOR01_INITIALIZATION_CONTRACT_BLOCKED`

NEXT = `RESTORE_CAUSAL_INITIALIZATION`

## Git and scope

Repository: Liumengjian666/Map-Matching.
Branch: `research/p9-r4-heldout-visual-evidence`.
Start: `816f7e8d75ec7af4ecce208e9520156359efa250`.
End: containing commit (`git log -1 --format=%H -- this-directory`).
Workspace: `/tmp/dog_loc_paper_r4_ws.Fq21k2`.
No push performed. Stable workspace and all earlier experiment receipts unchanged.

## Raw protocol result

Protocol: `P9_CORRIDOR01_RAW_SCANEND_V1`, **prospective, not historical v1**.
Fresh complete raw bag SHA256 PASS:
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.
Velodyne pin `29abd0e1361cb7f5eda451d2b51c35eeca45e0d5` and VLP16db hash PASS.
Normalized map hash PASS:
`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`.

One full extraction completed; no rerun. Persistent output:
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p9_corridor01_raw_scanend_v1/`.
Actual write probe passed; old v1/v2 were not overwritten.

| Output / audit | Result |
|---|---:|
| Raw scan messages preserved | 2777/2777 |
| Raw IMU messages preserved | 55957/55957 |
| Valid raw timed XYZ points | 79,932,911 |
| Packed point bytes | 1,278,926,576 |
| Zero-return samples omitted, counted | 1,103,601 |
| Nonfinite emitted points | 0 |
| Scheduled slots not emitted by pinned parser | 7,456 |
| Packet sensor-header vs embedded-clock max error | 119 ns |
| Scan duration range | 100.726439–100.839450 ms |
| Inter-scan overlaps | 4 scans; maximum 40,366 ns |
| In-order output with local point-time reversals | 272 scans, 276 reversals |
| Initial IMU-boundary uncovered scan | transaction 1 |
| Max causal IMU tail gap at scan end | 5.683412 ms |

No timestamp sorting or silent scan removal. The pinned parser has a documented
azimuth-wrap skip path; its not-emitted slots are counted, not invented. Actual
packet timing produces small overlaps/reversals, retained for P7's existing
time-indexed handling. No claim of real deskew success is made.

All generated points passed finite/time-bound/binary-offset checks. All 55957 IMU
numeric fields and timestamps round-tripped against the preserved serialized
raw-message payloads. This is an output/provenance audit, not a second independent
raw packet decoder comparison. The genuine P7 reader independently consumed
2777 scans, 79,932,911 points and 55957 IMU records successfully.

## Initialization: decisive stop

Frozen original P7 parameters: first 200 samples; max per-axis sample standard
deviation 0.50 m/s² acceleration and 0.05 rad/s angular velocity.

| First 200 raw samples | X | Y | Z |
|---|---:|---:|---:|
| Acceleration std, m/s² | 2.212268455 | 0.921842325 | 0.468201597 |
| Gyro std, rad/s | 0.150764662 | 0.241932641 | 0.404157685 |
| Gyro mean, rad/s | -0.010306436 | 0.106549935 | -0.473918351 |

Time range: 1517157219159216000–1517157220154144000 ns.
First measured gyro norm: 0.456212974 rad/s.
Real unmodified P7 `initializeStatic()` returned:
`static_imu_variance_exceeds_gate`.

The input probe used identity pose fixtures only to exercise this variance gate;
the rejection occurs before pose use. It never established a real initial pose,
propagated real scans or aligned NDT.

Historical `corridor01_init.yaml` is hash-verified, but its 50-scan/5-second
first-scan pose is not a current-time moving IMU state. It has no initial velocity
or bias, and cannot simply be backdated into earlier deskew. No trustworthy
minimal causal moving-state adapter was established in this task. This is not
a proof that such an initializer cannot be built.

`params.txt` intentionally NOT_CREATED until that state is accepted. Generating
executable parameters with guessed zero velocity/bias would hide the blocker.

## Baseline, scientific boundaries and costs

Raw timing representation: PASS. Real scan-end propagation/deskew: NOT_RUN.
Smoke NDT = 0; Run A = NOT_RUN/0 calls; Run B = NOT_RUN/0 calls.
Source/T0/U_obs/W2 parity and baseline determinism: NOT_RUN, not PASS.
No actual Corridor01 target count or U_obs values are fabricated.

NEW_ORACLE263_CALLS = 0; NEW_B12_CALLS = 0; VISUAL_EXTRACTION = 0;
GT_LOADED = NO. No R6 statistics. Historical P2B NOMINAL_TRACKING_RISK remains;
there is no new P9 trajectory or GT-error finding.

Raw extraction wall time: 38.758459 s (excludes preflight hashing).
Child peak RSS: 58168 KiB (preflight children + extraction process high-water mark).
Alignment mean/P95, preprocessing and deskew costs: NOT_RUN.
All work is CROSS_DATASET_PREPARATION_COST, not online DUAL-U increment.

## Verification and review

Release converter + actual P7 input core build PASS. Existing P9 Release build
PASS; P9 tests 41/41 PASS. Input/parser/P7 IO/propagation/deskew/frame tests 6/6
PASS. Additional synthetic-bag boundary cases 5/5 PASS (normal packet, missing
IMU, missing LiDAR, unsupported dual return, repeated-output refusal).
Actual baseline repeatability tests NOT_RUN due the gate, distinct from synthetic
tests. Test logs and hashes are archived.

Fresh-context independent read-only review fixed empty-topic success and pinned
header-precedence gaps and added real parser synthetic coverage. User chose no
external cross-model review. Build-only ROS overload/YAML macro fixes did not
alter parser math. Archive generation also fixed relative/absolute path handling;
neither correction reran raw extraction or NDT.

Input hash chain, CSV/JSON schema-width/hash audit and scoped `git diff --check`
are final delivery guards. The converter CMake was later extended only for the
no-NDT P7 probe. Its pre-extraction version is preserved byte-exact against the
preflight SHA; extraction binary SHA remains unchanged.

Only next step: restore a credible causal initial state using the preserved raw
input. Do not resume oracle/B12/cross-dataset inference before baseline closure.
