# R7-R5 environment recovery and one-shot robust bootstrap V2

`ENVIRONMENT = PASS`

`FINAL_RESULT = BOOTSTRAP_LOCAL_ODOMETRY_NOT_CERTIFIED`

`NEXT = REASSESS_DATASET_BOOTSTRAP_FEASIBILITY`

The one authorized frozen V2 scheme was actually executed. TX2--TX4 failed the
forward-overlap gate consecutively, triggering the predeclared stop. No state
fitting or formal NDT was started. This is a startup engineering result, not a
DUAL-U scientific failure and not a mathematical unobservability result.

## Git and environment

Repository: `Liumengjian666/Map-Matching`.
Branch: `research/p9-r4-heldout-visual-evidence`.
Start SHA: `368b78f8cd2fa124f03103c3807b702197eac7ee`.
Execution worktree: `/tmp/dog_loc_paper_r4_ws.Fq21k2`.
The archive commit is the final deliverable SHA; it cannot embed its own hash.
No push was executed. The start remote SHA was supplied by the research
controller; no new network verification is claimed.

HIKVISION is `/dev/sda2`, fuseblk/exfat, mounted `ro`. Its path exists and it has
ample capacity, so the obstacle is not missing data or a full disk. No chmod,
sudo, remount, symlink bypass or stable-workspace modification was attempted.
The frozen alternative is:

`/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p9_corridor01_bootstrap_v2`

It is persistent `/dev/nvme0n1p2` ext4, mounted rw, inside the explicitly
authorized writable workspace root. A real small-file write/readback/delete
test passed. Observed available capacity was 65,310,380,032 bytes, above the
frozen 10 GiB reserve. `environment_receipt.json` records the exact authorization,
mounts and probe hash. This cache is not included in Git. The only removed item
was the temporary storage-test probe, after successful readback; no user data
was removed.

## Inputs and preflight

The existing prospective raw protocol was reused without extraction or writes:
2777 scans, 55957 IMU samples and 79,932,911 timed points. All ten actual manifest
input files passed SHA256 verification. Input manifest SHA:
`591bfe3fd619966f40e4e2af6b9151937732483f0123c70eb34aa1031742991a`.
The map, calibration and sensor-only anchor hashes were also reverified.
Official raw bag SHA
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`
is inherited from the accepted provenance receipt, not a fresh whole-bag hash
in this turn. Historical v1 equivalence is not claimed.

The first-ten-second ledger has 99 scans, from 1517157219088119030 ns to the
latest legal scan end 1517157229072630478 ns (9.984511448 s). All 99 scans passed
Python timing/field checks and actual C++ P7 input/causal rotation/deskew
preflight, without GICP or NDT. No consumed IMU timestamp exceeded scan end.
This is not certification of the formal initialized IKFoM propagation chain.

## Actual quality results

All metrics below are from real independent forward and reverse GICP, not
synthetic fixtures. Overlap threshold is 55%, bilateral trimmed RMSE threshold
0.25 m, closure thresholds 0.10 m and 1 degree.

| TX | Forward overlap | Reverse overlap | Trimmed RMSE fwd/rev (m) | Closure translation (m) | Closure rotation (deg) | Decision |
|---|---:|---:|---:|---:|---:|---|
| 2 | 38.2217% | 96.2064% | 0.075157 / 0.071092 | 0.009350 | 0.170429 | OVERLAP FAIL |
| 3 | 35.9954% | 91.5280% | 0.084502 / 0.078281 | 0.011760 | 0.305491 | OVERLAP FAIL |
| 4 | 33.5014% | 90.6832% | 0.088435 / 0.093784 | 0.008858 | 0.428984 | OVERLAP FAIL; STOP |

Both solvers converged and their transforms were finite rigid transforms for
all three pairs. Trimmed RMSE passed 3/3, independent closure passed 3/3, IMU
rotation consistency passed 3/3 (0.331474, 0.268596, 0.814909 degrees), and physical
step guards passed 3/3. The only rejection factor was forward overlap.
Untruncated forward fitness was 61.838949, 64.661172 and 66.508626 m² against the
prefit support; these are recorded, not substituted for the new quality gate.

There were 98 planned opportunities, **only 3 attempted**, 0 accepted.
The complete 99-frame ledger contains 1 coordinate anchor, 2 causal prediction
records, 1 rejected-stop record and 95 `NOT_RUN` records. This is not 98 measured
failures. The 90-observation gate was not met and maximum consecutive failures
was three, above the frozen maximum two. Estimation and validation intervals
have no real observations because execution stopped before them.

## Crucial startup limitation

TX1 has no leading IMU coverage. The frozen causal policy omitted
20,394/28,866 raw points (70.6506%) from its bootstrap cloud, leaving 8472 raw
points before filtering and 661 voxel points. The original raw input is intact.
TX2--TX4 had 1732, 1728 and 1785 current voxel points. Since all new clouds were
rejected, they each matched only the **same single truncated TX1 anchor**.
An established three-frame submap and accepted-history CV predictor were never
reached. Thus the outcome certifies failure of this frozen startup protocol;
it does not establish Corridor01 whole-sequence infeasibility. Anchor truncation
is a concrete limiting condition, but no new ablation proves it is the unique
cause. No parameter change or second V2 run was attempted.

## Downstream and cost

Moving-state fit, rank/condition/split stability, 8--10 s state prediction
validation, full covariance, real IKFoM initialization, formal scan-end smoke,
Run A/B and source/T0/U_obs/W2 parity are all **NOT_RUN**, not FAIL or invented
zeros. A full 2777-frame ledger preserves the 99 bootstrap scans and 2678
post-bootstrap inputs. None was represented as an initialized formal NDT
transaction.

Real runner wall time: 0.246438 s; measured wrapper/child wall: 0.263200 s.
Total six GICP solver calls: 0.111179 s; rotational deskew: 0.021827 s;
robust quality evaluation: 0.015173 s. Peak RSS: 22,168 KiB. Per-phase
mean/median/P95 and explicit NOT_RUN downstream rows are in
`runtime_breakdown.csv`. Only three real pairs were attempted; these costs are
not an estimate of a completed 99-frame bootstrap or machine-dog online cost.
They are baseline preparation costs, not DUAL-U incremental overhead.

`ORACLE263_CALLS=0`, `B12_CALLS=0`, `VISUAL_EXTRACTION=0`, `GT_LOADED=NO`.
NDT smoke, Run A and Run B calls are each zero because their gates were not
reached. Raw extraction was not repeated.

## Verification and review

Release build PASS; P9 tests 41/41; V2 CTest targets 5/5; inherited P7/deskew/
initialization tests 6/6. Post-run CSV audit regression tests are separate.
Tests cover causal IMU/time boundaries, future-sample tampering, full lever arm,
SE(3) direction, independent reverse matching, fixed support populations,
invalid solver results, rejected-history preservation, third-failure stop and
one-shot prevention after success or child failure. Real input preflight 99/99
passed. A CTest invocation in the wrong directory produced “No tests found”;
it is not counted as validation. Actual build-directory logs are archived.

The user chose current-task single-model review only; no external CLI ran.
Independent pre-run findings concerning causal timing, lever arm, immutable
point populations, one-shot execution and invalid-result rejection were
incorporated before freeze and tested. One implementation reviewer later hit
a service usage limit; that incomplete review is not claimed as comprehensive.
The final read-only result review found no critical data/count/time/guard defect
and required the truncated-anchor interpretation boundary, included above.

The pre-commit whitespace check found trailing whitespace in CMake-generated
build snapshots. Only archived textual snapshots were normalized (line-end
whitespace and final blank lines); `build_snapshot_normalization.json` binds
the unchanged original build files and normalized copies with separate hashes.
This was an archive-format correction, not a binary/source or experiment
change. Four post-run audit regression tests pass.

`archive_v2.py` and `test_archive.py` are post-run archive utilities; they were
not part of the frozen scientific binary. All scientific source/binary/input
hashes are rechecked unchanged. The audit checks complete hash inventory,
rectangular CSV, finite numeric values, valid JSON, preserved ledger and
explicit NOT_RUN scope. No old R4/R5/R6/R7 archive or production algorithm was
modified. The Git scope contains only the new V2 source and archive; no raw,
build, cache, user preference or unrelated untracked files are staged.

## Decision handoff

The sole formal result is `BOOTSTRAP_LOCAL_ODOMETRY_NOT_CERTIFIED`.
The sole NEXT is `REASSESS_DATASET_BOOTSTRAP_FEASIBILITY`.
The research controller should inspect the real code, quality tables and
causal boundary receipts before choosing a different dataset or authorizing a
new startup protocol. This execution does not initiate V3/V4/V5, moving-state
threshold changes or an alternative dataset.
