# A3G-R4 direct debugging result

Engineering objective **NOT ACHIEVED**. Tracking/recovery infrastructure and
regression gates are implemented, but the final safety-corrected prefix still
loses LiDAR admission. Full Corridor01 was not launched. No GT or visual input
was used; no localization-accuracy conclusion is made.

## Implementation

- Explicit optional tracking configuration; the original V3 CLI without this
  argument remains disabled, and legacy FULL/V1/V2 behavior is preserved.
- GOOD/DEGRADED/RECOVERING/LOST health; two consecutive failures trigger at most
  two extra seeds: last committed registration, then bounded constant motion
  from the last two committed registrations. No Window reset or prior injection.
- Nominal and recovery candidates share support, reliability, adaptive-noise,
  preview and selected-NIS admission. Only the first admissible candidate is
  submitted once; rejected previews consume no observation ID.
- PCL zero-iteration and iteration-exhausted terminals are not independent
  registration successes. The same classification applies to nonlocal probes.
  Recovery probes are paired with their own nominal seed; current Window P15
  and current Window innovation/NIS remain authoritative.
- A guarded launcher hashes all seven frozen inputs, uses the official
  calibration, closes expensive legacy shadows and stops after 20 consecutive
  completed noncommits. This stop is a test guard, not a new measurement gate.

No estimator files under `src/` or `include/` changed from START_SHA. U_obs,
U_nonlocal definitions, NIS formula/thresholds, factor math, optimizer settings,
QR covariance/marginalization and deskew math were not changed. Wrapper validity
classification and recovery/probe wiring intentionally changed when enabled.

## Actual debugging trials

Every trial started from the frozen handoff with independent output storage.
This R4 stage authorizes iterative debugging; four real prefix trials were run.
All four input SHA gates matched the frozen A3F/A3G identities. The C++ raw-timed
manifest validation remained enabled and passed; map and calibration were not
regenerated or changed.

| Trial | Completed terminals | Commits | Last commit | Longest noncommit streak | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| prefix220_v1 | 158 | 115 | 189 | 20 | Guard stopped at tx209 |
| prefix220_v2 | 169 | 127 | 220 | 6 | Process 0; not a stability/accuracy claim |
| prefix400_v2 | 314 | 194 | 365 | 6 | Covariance rank failure at tx366 |
| prefix220_v3 (final code) | 125 | 94 | 156 | 20 | Guard stopped at tx176 |

The v2 longer run still developed large internal motion (maximum terminal
translation increment 10.14824 m). At tx366 it had **9 active LiDAR factors**,
yet covariance failed: stack 614x600, rank 495, threshold 928.39483569,
max pivot 6.80963947e15. The rank gate was neither repaired nor bypassed.
Continued commits alone were demonstrably insufficient for a soak PASS.

Inspection found that alternative nominal registration and its probes used
different seed centers; ineffective probe terminals could also pass the raw PCL
convergence flag. v3 corrects only this wrapper inconsistency. Therefore the v2
completed prefix is not evidence that the final implementation is stable.

## Final v3 evidence and blocker

- Last commit: tx156, stamp 1517157234821345402, constant-motion seed, 2975
  correspondences, rank 3, NIS 3.4567845518 / threshold 11.345.
- tx157--176: 20 completed terminals without a commit. The external guard
  terminated the process group (exit -15); this was not an estimator crash.
- Recovery candidates: 48 extra nominal NDT calls; 36 with effective registration
  and valid support; 35 rejected by selected NIS, 1 committed.
- tx166 last-observation recovery: 15 iterations, fitness 0.6300550851,
  2742 correspondences, rank 3, NIS 60.41871214 > 11.345.
- tx174 last-observation recovery: 31 iterations, fitness 0.06783386965,
  3009 correspondences, rank 3, NIS 62.02884909 > 11.345.
- tx175 constant-motion recovery: 56 iterations, fitness 0.05804044707,
  3154 correspondences, rank 3, NIS 532.42370236 > 11.345.

These are internal support/admission observations, not evidence of ground-truth
pose correctness. Existing Window prediction and recovered registration remain
statistically incompatible. Increasing the NIS threshold, resetting Window
state, injecting a prior, changing reliable-subspace math or modifying QR rank
would exceed this stage's frozen-core boundary. No such action was taken.

Final run health: 125/125 P15 available, 212/212 QR removals successful, optimizer
238 ACCEPTED_UPDATE plus 13 CONVERGED_WITHOUT_STEP, finite/SO3 checks passed,
maximum completed window 40 nodes / 1.958914906 s, post-handoff IKFoM=0,
visual=0, legacy fallback=0. Raw PCL `hasConverged` count is 125; only 105
nominal/selected registrations satisfy the new effective-registration rule.
Do not confuse those two counters.

## Verification and disposition

Release build PASS; full CTest 44/44 PASS; Debug targeted 24/24 PASS;
`git diff --check` PASS. No previous regression tolerance was relaxed.
Healthy-path OFF/ON fixture: identical trajectory/deskew/non-timing health,
same factor/observation lifecycle and no extra NDT/probe calls. Displaced-state
fixture: valid recovery does not bypass NIS or mutate state through preview.

Incremental commits and first-failure preservation were used throughout.
External manual cross-model review is left to the user/decision AI, as requested.
Builds were moved to persistent external directories and serialized after an
earlier environment restart; no real trial was silently retried.

External results and file hashes: [EXTERNAL_RUN_FILES.json](EXTERNAL_RUN_FILES.json).
Compact trial/resource records: [RUN_SUMMARY.json](RUN_SUMMARY.json).
References: [REFERENCES.md](REFERENCES.md).
Ready-to-send decision handoff: [DECISION_AI_HANDOFF.md](DECISION_AI_HANDOFF.md).

CORE_ENGINEERING_SOAK=NOT_RUN

READY_FOR_NEXT_INNOVATION_STAGE=NO

READY_FOR_FORMAL_EXPERIMENT=NO

STOP: remaining admission/prediction closure requires decision-AI direction;
no full run, no covariance rank repair, no GT, no visual, no core-math change.
