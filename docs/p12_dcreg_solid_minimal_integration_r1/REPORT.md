# P12 — author DCReg + existing SOLiD minimal offline integration

FINAL_RESULT: **MINIMAL_INTEGRATION_PARTIAL**.

The two author algorithms actually execute in one connected data path. Exact
reference-state-to-LiDAR poses are still not established; all72 retrieval
initializations are explicitly coarse hypotheses. Matching convergence is
limited and retrieval correctness has no independent label. These facts do
not justify an exact/global relocalization or localization-accuracy claim.

## Git and untouched systems

Repository `Liumengjian666/Map-Matching`; branch
`research/p9-r4-heldout-visual-evidence`. START_SHA
`578291c999ea375ff72538da7b6a34fb54dc85bb` was verified local and remote at start.
CODE_SHA `84625aec06778f630d82ca37fd3a265216a804b9` includes the offline probe,
frozen protocol and corrected smoke launcher. Smoke binary was compiled under
`c8ed91f29a5d9c0be51abe9fe143c1bfe9b0fed8`; the sole later code change removes
an unintended convergence assertion from the launcher. The same binary,
source, map, DB and author settings were used for all96 real registrations.
The later archive-formatting change alters Git CSV line endings and the
archive copy writer only; it does not alter the registration binary/experiment.
Git samples/extraction CSVs use LF; the external frozen originals retain CRLF
and their original hashes. Parsed rows are identical. The samples hash in
freeze.json refers to the external immutable source; artifact_hashes.json
records the normalized archive bytes separately.

Only new files under `scripts/p12_dcreg_solid/` and this archive were added.
NDT/EKF/OOSM, limitNdtStep, SOLiD, R6 and all historical archives are unchanged.
Stable robot workspace and dirty main paper workspace were not modified.
No production integration, ROS replay or feedback took place.
PUSH_EXECUTED=NO. Actual END_SHA is the archive commit reported at handoff;
do not infer remote delivery from a local commit.

## Original implementations and direct connection

DCReg pin: `8ce8451b15491a4bbe17cf85ab02a8bed6696861`, from the
[author repository](https://github.com/JokerJohn/DCReg). The external checkout
is clean; its original `dcreg.hpp` and `utils.hpp` are compiled directly, with
no solver, Schur, PCG, correspondence or manifold rewrite. No license statement
was found. Author source and binaries are not vendored or committed.
SOLiD remains P11's adapted author implementation at
`9a4f2895e5211f69fbbd6d4732577e1a67e9b882`, without a descriptor change.

The actual caller performs:

```cpp
query = solid.query(*source, 3);
initial = p12::coarsePosition(query.top[rank-1].map_T_reference, ndt_initial);
test_case.initial_pose = dcreg::MatrixToPose(initial);
test_case.has_ground_truth_pose = false;
result = dcreg::RunRegistration(*source, *target, test_case, options);
```

Source is not pretransformed; the author applies `R*p+t` and returns an already
complete source-to-map transform. There is no multiplication by initial pose
after registration. Options: original DCReg, SO3, Serial, verbose=false,
**unmodified SolverParameters defaults**, max30 iterations. The third-party
math remains wholly external.

Local control uses the recorded current NDT **initial guess**, not its final
answer. `ndt_baseline.csv` separately preserves raw PCL pose, step-limited used
pose, initial guess, source hash, NDT native fitness and scan start/mid/end.
No NDT pose is treated as GT and no cross-algorithm native score comparison
selects a winner.
DCReg poses are original author terminal outputs, not ROS step-limited
measurements; they are never published or passed to EKF. Existing NDT raw and
used poses remain separately labeled. No main-chain trajectory parity rerun
is claimed from this offline experiment.

## Actual input and coordinate status

Loop2 map: unchanged active `camera_init` map. Target preparation is the same
two .15m voxel steps, resulting in459154 points, FNV `ea1e4e69cf9992c6`.
Loop2 reference DB:389 independent offline mapping keyframes, not Loop3 GT
or evaluation scans. Loop3 is the independent query recording.

The24 immutable samples cover low-reference-spread regions, straight-motion
corridor proxies and widely separated Top3 references (eight each). The proxy
is not a independently verified corridor label. Exact indices and selection
rule are preserved in `samples.csv`, `freeze.json` and `PROTOCOL.md`.
Only those24 raw scans were extracted, not a5817-frame or full bag replay.
P11 did not record filtered point payloads. Reconstructed finite/range/voxel/
cap outputs match the actual prior NDT **24/24 source counts and content hashes**.
Original robot baseline deskew=false and midpoint prediction association remain
unchanged; raw header scan-start nanoseconds and associated timing are retained.

Map SHA256 `016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca`;
DB SHA256 `065cd4b6568f331f717421c31060492493ce329d1b5414176428e10d4febd1b4`.
Both were recomputed. The19.24GB original bag's historical P11 hash is retained;
this round does not falsely claim to have rehashed that complete bag.

FAST-LIVO2 logs the mapping IMU-origin state, not a proven exact LiDAR-origin
pose. The caller therefore deliberately takes only the reference translation
and current query's NDT-initial rotation. All72 instances are marked
`COARSE_POSITION_HYPOTHESIS`, including the unclosed lever-arm/time approximation.
Exact reference-to-LiDAR conversion count: **0**. No guessed calibration or GT
fills this gap. No principle source/target inversion or double transform was
found, but this does not certify the omitted exact frame/time transport.

## Fixed experiment results

24 scans,24 valid descriptors/Top3 lists,24 local registrations,72 actual
SOLiD-seeded registrations: **96 DCReg calls total**. All72 current Top3 IDs
and similarities agree with the previous P11 query on the same payload.
New NDT calls=0; EKF updates/feedback=0; GT_LOADED=NO.

| Result | NDT-initial DCReg | SOLiD Top3 -> DCReg |
|---|---:|---:|
| Calls |24|72|
| Author reports converged |12/24 (50%)|12/72 (16.67%)|
| Valid SE(3) outputs |24/24|72/72|
| Nonfinite output poses |0|0|
| Reached max30 without convergence |12|60|
| Any-iteration author degeneracy mask nonzero |9|25|
| Mean iterations |21.33|27.83|
| Native last-pre-update RMSE mean (m) |.11394|.26780|
| Native correspondence fraction mean |.92548|.73394|

10/24 query samples have at least one converged candidate. Thirty of the72
reference hypotheses are >5m from their query's recorded NDT initial position;
**0/30** of those registrations converge under the frozen defaults. This is
not proof that the distant places are incorrect: the coarse orientation and
frame/time approximation are unresolved, and there are no independent labels.

All96 poses are finite/rigid. Maximum correction from initial:1.56417m,
16.07061deg. No correction crosses the predeclared >3m/>45deg diagnostic
guard. That does not establish correctness: a wrong initial place can remain
near itself while producing a legal pose.

### Three actual cases

TX16: local DCReg does not converge in30; SOLiD IDs0/1/163 have cosines
.976688/.946562/.940087. ID0 converges in6, residual.111229m, position
(+.004997,+.004843,+.005849)m. ID1 converges in22 to
(-.552405,+.044700,+1.214333)m, residual.306089m. ID163 stays near a distant
reference (-31.036,-82.228,-11.313)m, does not converge. No label identifies
which candidate is correct; convergence alone cannot choose ID0 versus ID1.

TX1413, straight-motion proxy: local converges in12, residual.044390m,
position(-3.731995,-10.848292,+3.986037)m. Top3 IDs53/45/258, cosines
.982839/.982468/.977684. ID53 converges in16, residual.106423m,
position(-3.587387,-13.329641,+4.109026)m; others hit30. This is actual connected
candidate validation, not an independently certified alternative basin.

TX4105: local and all three candidates hit30. Top3 IDs265/203/252, cosines
.983585/.914466/.887802. ID265 has correspondence fraction.997967 and
residual.063872m, yet no iteration convergence; ID203 fraction.328252 and
residual.305212m. This preserves a clear negative case rather than calling
finite/high-fitness output successful localization.

## Measured cost (current experiment, not historical sums)

| Stage, ms |mean|P95|max|
|---|---:|---:|---:|
| Input PCD/source preprocessing |.836|.960|.968|
| SOLiD descriptor |.148|.227|.232|
| SOLiD retrieval |.088|.103|.132|
| NDT-initial DCReg, complete API call |109.58|145.57|146.41|
| One SOLiD-seeded DCReg API call |143.20|182.77|188.00|
| Actual query -> three DCReg + logging |430.02|518.74|548.51|
| Complete probe scan, including local control/preprocessing/hash/logging |546.08|650.66|669.21|

The combined timer surrounds this run's actual connected path. It is not the
sum of P11 descriptor timing and a separate DCReg experiment. Per-call wall
time includes author target copy/KD-tree construction; author internal timer
starts after that work. Mean call-minus-iteration setup overhead:57.94ms.
No target caching/performance optimization changed author code this round.
Map/DB loading occur once per process and are not per-query cost.

Smoke wall1.17s + remainder12.07s =13.24s, including two process starts/map and
DB loads, excluding build/extraction. Peak RSS64664KiB =63.148MiB.
This is an offline probe resource measurement, not a combined ROS/EKF runtime
or embedded real-time success claim. The430ms three-candidate cost is not yet
a low-cost online validation module.

## Verification, real interruptions and final assessment

Release build PASS. CTest1/1 PASS covers source->map direction, pose roundtrip,
coarse hypothesis provenance, NaN guard and actual author three-plane
registration. Real24 source hash/count parities PASS; actual96 rigid outputs
and72 live query-to-registration links PASS. Structured output consistency and
artifact audit PASS. Only own interface/extraction/calling/summary code is
committed. Original author files' hashes and clean status are recorded.

Build required the existing SOLiD header include path and correct API signature.
An inherited camera-SDK LD_LIBRARY_PATH selected an old libusb; the probe
explicitly loads system libraries. These are build/runtime compatibility
fixes, not mathematical changes. Original failed build logs are retained.

After the first two real scans, the launcher stopped on its unintended
"a local call must converge" assertion. This was not an author quality gate or
the frozen experimental acceptance rule. It was removed from the launcher to
separate interface validity from scientific convergence, and the remaining22
preselected scans were run with the **identical binary and solver parameters**.
The initial eight registrations were neither deleted nor rerun. Full negative
outcomes and the two-code-SHA launcher provenance are retained.

Necessary independent read-only review identified source-integrity and score
meaning boundaries, and invalid-pose delta logging was corrected to unavailable
rather than zero before real registration. No external model CLI was used.
The incremental/debugging/review workflow affected interface guards and result
semantics only; it did not modify either mature algorithm.

Answers to acceptance questions:

- DCReg runs on actual map/scans and converges in some cases: **yes**.
- Actual SOLiD candidates become real DCReg inputs: **yes,72 calls**.
- No identified source/target direction error; exact pose calibration: **unclosed**.
- Nonfinite/illegal pose or large-excursion diagnostic: **none**; many nonconverged.
- Candidate geometry validation cost: **430.02ms mean for Top3**.
- Further research foundation: **connected prototype established, exact reference
  pose and candidate correctness not certified; no online decision/fusion claim**.

ACCURACY_GAIN_NOT_EVALUATED. No NDT result is substituted for GT; no success
rate above is a localization correctness rate. Minimal coarse integration
works, but complete exact candidate-registration acceptance is not claimed.

NEXT (one only): **ESTABLISH_REFERENCE_TO_LIDAR_POSE_CONTRACT**.
Close the actual reference-state/extrinsic/time mapping before judging precise
candidate recovery or beginning any Dual-U/feedback module. No automatic next
experiment, descriptor redesign or parameter search is started.
