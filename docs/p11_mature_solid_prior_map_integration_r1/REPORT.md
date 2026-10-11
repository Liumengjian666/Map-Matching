# P11 mature SOLiD prior-map integration: measured result

TASK: P11-MATURE-SOLID-PRIOR-MAP-INTEGRATION-R1

FINAL_RESULT: **SOLID_QUERY_FUNCTIONAL_MAINCHAIN_PARITY_NOT_CERTIFIED**

The author's actual descriptor runs on real independent mapping/query data
with sub-millisecond algorithm timings. Full NDT source and nominal numerical
outputs are identical in A/B. EKF/OOSM output parity fails because fixed-lag
arrival deadlines are not reliably met in either real-time ROS replay. This
is a partial engineering result, not a certified unchanged localization chain,
retrieval-accuracy result, or a Dual-U scientific validation.

GT_LOADED=NO. ACCURACY_GAIN_NOT_EVALUATED. Feedback=0.

## Git and protected workspaces

- Repository: Liumengjian666/Map-Matching.
- Branch: `research/p9-r4-heldout-visual-evidence`.
- START_SHA / observed remote HEAD: `0c594b11e68ed0e7f3a2b0b00aa9e6ce7584298e`.
- CODE_SHA: `18c702afa3af1750ffecac27c668dca3eb176075`.
- Execution workspace: `/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`.
- The main paper workspace is dirty and older; it was inspected read-only.
  The stable machine-dog workspace was not changed.
- No push executed. The report-owning final commit is reported separately in
  the chat and external delivery receipt; it is not circularly embedded here.

## Mature source, compatibility and architecture

Upstream: https://github.com/sparolab/SOLiD, actual fetched main
`9a4f2895e5211f69fbbd6d4732577e1a67e9b882`, BSD-3-Clause.
`cpp/src/solid.cpp` is byte-identical to the vendored implementation, SHA256
`765d21b53f4c12e2ebaabf935701c423db83076b9c8476944d836b061c711225`.
Copyright, license and upstream README are preserved.

Compatibility constants only: vertical FOV [-24.8,+2] -> [-7,+52] degrees;
max range80 ->40 m. Min range3 m, bins40/60/32, .4 m voxel and equations are
unchanged. Adapter guards finite XYZ, sufficient compatible points, valid
bins, zero height-normalizer, finite nonnegative descriptor counts and norms.
DB loading validates profile, unique IDs, rigid poses and descriptors.

The original unguarded profile has a NaN descriptor on empty input. On the
first three real raw reference scans, .938889/.865421/.974239 of its retained
points occupy the last height bin. `upstream_original_profile.csv` preserves
this measurement. These raw-profile probes and the adapted NDT-preprocessed
database are **not** a paired ranking/accuracy comparison: preprocessing and
sensor profile differ. They identify boundary/FOV incompatibility, not a
measured retrieval gain from changing FOV.

New independent `SolidPlaceRecognitionAdapter` calls upstream `makeSolid`
and `loop_detection`; deterministic descending cosine / ascending ID tiebreak.
The optional sibling `solid_shadow_node_cpp` subscribes to the NDT node's exact
`/dog_livo/filtered_points` source and publishes no pose, measurement or TF.
Default launch enable=false. No second NDT map is created. NDT/EKF/math files
and all prior R1-R6 results remain unchanged. This is compatibility/integration
work, not an original descriptor algorithm.

## Independent reference database and input

Corridor01 has no verified independent mapping keyframe sequence in available
assets, so it is not used to manufacture a descriptor database. The permitted
machine-dog alternative is used:

- References: genuine independent Loop2 mapping raw bag,
  `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`.
  SHA256 `d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae`.
- Active map: Loop2 `loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd`,
  SHA256 `016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca`.
- Active mapping-state pose file: `loop2/loop2mapping/pcd/lidar_poses.txt`,
  SHA256 `42d0c3060f77f83ea438916c0b305a9f7320c3ff7161148a7a475f4791022ed9`.
- Independent online queries: Loop3 `loop3_2026-08-11-16-09-07.bag`,
  SHA256 `e7663609a90fd9e59552fefd7f2c4f9c339e8464610f86bfa1e8f2cff2e79df2`.
  5817 LiDAR scans and116330 IMU messages. Only raw LiDAR/IMU topics played;
  no recorded localization outputs/GT supply a prior.

Existing keyframe extractor: 1 m spacing, nearest raw scan within .12 s.
389 real reference scans yield389 valid descriptors. Maximum observed pose
association offset .0502164364 s, with both timestamps retained in the ledger.
Older archived poses differ by up to9.3428 m from the active map's pose lineage
and are excluded; same-looking timestamps are not sufficient provenance.

Frame: `camera_init` mapping frame. FAST-LIVO2 actually writes state/IMU-origin
poses in `lidar_poses.txt`, not a proven LiDAR-origin pose. API uses
`map_T_reference`, deliberately not `map_T_lidar`. The map-generation physical
extrinsic and exact raw-scan/state time transport are not independently closed.
Locations are coarse map-index references, **not legal exact NDT initial poses**.
No localization evaluation GT constructed the DB.

Online only descriptor/pose index loads: text DB400864 bytes, descriptor payload
311200 bytes; descriptors+pose/time/ID numerical payload367216 bytes
(~.350 MiB; excludes C++ container/allocator overhead). Raw reference clouds
remain offline. DB SHA256
`065cd4b6568f331f717421c31060492493ce329d1b5414176428e10d4febd1b4`.

## Frozen full A/B runs

One CONTROL A and one CONTROL+MATURE_SOLID_SHADOW B, each complete5817 scans.
Same map/config/init/filter/voxel, .8 m NDT resolution, .08 step, .001 epsilon,
max40; same fixed lag .15 sensor seconds, queue64, replay rate1.0.
No parameter changes or selective repeats after observing results.

The actual machine-dog baseline uses deskew=false and midpoint observations;
these are preserved. Input is the exact baseline source, **not claimed to be a
verified deskewed source**. The separate SOLiD process does not block publication
by design, but asynchronous scheduling still needs empirical certification.

| Measured quantity | CONTROL A | SOLiD Shadow B |
|---|---:|---:|
| Scans / nominal full NDT calls | 5817 | 5817 |
| Optimizer reports converged | 5817 | 5817 |
| Step-limited frames | 198 | 198 |
| Extra full NDT / jet / feedback | 0 / 0 / 0 | 0 / 0 / 0 |
| OOSM APPLIED | 5603 | 5565 |
| Fixed-lag late rejected | 202 | 240 |
| Initial invalid observations | 10 | 10 |
| NO_HISTORY / final deadline unreachable | 1 / 1 | 1 / 1 |
| Recorded high-rate outputs | 116130 | 116130 |
| Wall time including launch/drain | 593.644583 s | 598.412918 s |

Optimizer convergence is **not** a localization correctness measurement.
All5817 source counts/hashes and initial/raw/step-limited NDT numerical pose,
fitness, iterations and convergence fields have exact A/B parity. All5817
SOLiD inputs match the corresponding NDT sources. Hash comparisons normalize
hexadecimal leading-zero display only, not cloud contents or pose tolerances.

### Earliest real main-chain failure

Observation timestamp1786435755.8664975, deadline1786435756.0164976:

- A arrives at state1786435756.0180912: `LATE_NDT_FOR_FIXED_LAG`, rejected.
- B arrives at state1786435755.9324477: queued and APPLIED at
  state1786435756.0180912 with31 IMU replay samples.

The first sensor-timestamp-matched high-rate state difference is therefore
1786435756.0180912. Of116130 matching high-rate timestamps,114603 state rows
differ thereafter. Corrected streams have5603 versus5565 timestamps;5427
timestamps match,5353 matched states differ. Combined CSV interleaving is
separately audited, not substituted for same-timestamp state comparison.

The first mismatch is a timing-dependent acceptance difference, not a SOLiD
pose feedback (none exists). Both runs have many late arrivals. One A/B run
does **not** identify how much of the timing difference is caused by SOLiD
versus uncontrolled system load. Main-chain parity certification FAILS.
No EKF/scheduler change, slower replay, extra trial, or tolerance relaxation
was made to manufacture a pass.

## Real retrieval output

- 5817 queries;5816 valid descriptors and Top-5 lists (99.9828%).
- One rejected scan: source sequence795, stamp1786435826816218376 ns,
  470 source points ->136 FOV/range points ->48 after .4 m voxel; explicit
  `TOO_FEW_POINTS_AFTER_UPSTREAM_VOXEL`. No invalid vector enters similarity.
- 379 distinct Top-1 reference IDs. Every valid query scores389 references.
- Top-1 cosine mean .964991715, P95 .993123338, min .774588218,
  max .999679165. Top1–Top2 gap mean .012212279, minimum1.69e-7.
- 5605/5816 lists contain a reference >5 m from Top-1. Max Top1-to-other
  reference separation: mean60.0386 m, P95 103.4057 m, max114.3737 m.
- Reference normalized exact duplicate pairs0. Two reference pairs have
  cosine>.99 with mapping-state separation>5 m.

Example first query: Top-1 ID0 cosine .975092; Top-2 ID364 .935862;
Top-3 ID1 .935742; Top-4 ID365 .928508; Top-5 ID163 .926642.
Its reference spread is88.5687 m. Full Top-K records are archived.

This establishes that the code emits distinct mapped reference candidates.
It does **not** establish retrieval precision/recall, correct place discovery,
true nonlocal ambiguity or distinct optimized NDT basins. Independent Loop3
map-frame query place labels are unavailable. Large reference separation and
close cosines may indicate confusion; registration/position evidence is
required before interpreting them. No Dual-U decision module is implemented.

## Measured resource cost

All5817 per-scan timing samples and process resource samples are preserved.

| Stage (ms) | mean | P95 | max |
|---|---:|---:|---:|
| SOLiD descriptor | .371972 | .698959 | 8.014091 |
|389-reference ranking | .194941 | .278724 | .525556 |
| SOLiD callback compute, excluding CSV flush | .665871 | 1.053086 | 8.211325 |
| A nominal PCL alignment | 9.227587 | 14.482972 | 92.738603 |
| B nominal PCL alignment | 9.415089 | 14.681920 | 89.321149 |
| A full NDT callback-to-used-pose | 12.768405 | 18.710025 | 96.576509 |
| B full NDT callback-to-used-pose | 13.101636 | 18.822818 | 93.610286 |

Callback-to-used-pose includes raw conversion/preprocessing/alignment but is
not a combined end-to-end EKF latency. SOLiD runs asynchronously; adding the
two callback means is not a measured per-frame critical-path duration.

SOLiD process CPU13.44 s over5817 scans =2.31047 ms CPU/scan, including ROS,
conversion, hashing and CSV overhead. A NDT/EKF CPU100.07/65.38 s; B NDT/EKF
104.02/68.56 s. Total observed process-CPU difference20.57 s is **not** a
strictly attributed module increment because schedules and applied updates
differ. Retrieval-stage timers alone understate process CPU.

Per-process peak RSS: A NDT92.883 MiB, EKF14.680 MiB; B NDT92.359 MiB,
EKF14.691 MiB, SOLiD42.195 MiB. Independent SOLiD process memory includes
ROS/PCL/library overhead, not just the .350 MiB numerical index. Sums of
separate process peaks are not a simultaneous system peak. No machine-dog
embedded low-memory/real-time success claim is made.

Offline DB builder:13.99 s wall, peak28344 KiB RSS. This is preparation cost,
not online search time. Entire raw/bag/cache data stay in the persistent
external experiment directory, not Git.

## Verification and conclusion

Release build PASS; five CTest contracts PASS; direct frame conversion,
IMU deskew/interval, OOSM planner tests PASS; isolated installed shared-library
loading PASS. Independent review findings were fixed and regression-tested.
CSV/JSON/hash and code-isolation receipts are archived. No external model CLI,
GT loading, feedback, multi-start NDT or research parameter tuning occurred.
Archive CSV/log copies use normalized LF line endings; the persistent raw run
files retain original bytes and separate hashes. No numeric values change.

Mature SOLiD is computationally executable on adapted MID-360 data. Sensor
compatibility for finite descriptors passes; retrieval reliability does not
yet have a valid accuracy test. NDT content parity passes; OOSM/EKF parity
fails. These different facts are deliberately not collapsed into one PASS.

NEXT (one only): **CERTIFY_DETERMINISTIC_SOLID_SHADOW_REPLAY**.

Restore a controlled replay comparison before admitting this integration as
unchanged main-chain behavior. Do not start candidate-based Dual-U decisions
or localization feedback from these unvalidated place candidates.
