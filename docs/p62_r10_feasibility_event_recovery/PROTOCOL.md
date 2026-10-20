# R10 pre-run execution contract

START_SHA=340a42190c9101f98cd483849537ced23a2a20a0. New research branch/worktree only. One new R10 native100-frame experiment; no R9 rerun, R9 output review, scale diagnostic or other dataset. Read/reuse specified R9 source and fixed R7 raw input manifest as dependencies, not previous result selection.

## Frozen inputs and optimizer

Original raw10641–10740, fixed100, same frozen map and historical causal initials. Source preprocessing and Coverage V2 unchanged. Actual PCL NDT .8m/.08/.001/40/.55, two Target.15m filters, Source.25m/range.5–80/cap1400. One CONTROL_FULL and one CONTROL_SPLIT per frame; PROPOSED reuses SPLIT. No GT, EKF/ROS playback, feedback, new trajectory estimate or production edit.

## Four distinct layers

1. Detect: preserve legacy R9 trigger (!converged OR iterations>=40 OR active median>.25 where defined). Additionally detect every nominal formal-admission failure: unsafe/nonfinite pose or numerical score, convergence, translation>.8m, rotation>5deg, insufficient active geometry overlap<.5, missing/invalid validation or coverage. Reasons may coexist; detection alone does not authorize optimization.

The old-trigger comparison field reproduces R9's actual validation-availability-gated predicate, with NA where unavailable. New nonconvergence/iteration-limit reason detection remains independent of validation availability; no missing median is fabricated.
2. Permit: recovery requires detected event, independently safe finite native SE(3) and scores, valid common prediction, training>=300/B>=4/finite active losses and overlap, remaining Budget20. Nonfinite/invalid nominal poses never enter G or seed generation. Missing evidence cannot become zero evidence. Bounds/convergence/support failure itself does not block a safe recovery attempt. Record detection and permission separately.
3. Generate/select: unchanged G ratio<=.01, weakest direction, at most SMALL+ and SMALL- (.30m/3deg), each once; physical dedup .10m AND1deg; unchanged .8m/5deg/overlap/convergence admission and common-weight robust D>.02. FULL is outside the pool. No LARGE or fixed forced diagnostics in R10. All extras share Budget20, chronological, no refill. Invalid candidate stops further seeds for that frame, but is preserved.
4. Final observation: independently recheck all eligibility and strict robust rule before replacing T0. If no supported eligible alternative but T0 itself is legal, ACCEPT_NOMINAL. Otherwise NO_ACCEPTABLE_MEASUREMENT, final index=-1 and accepted pose=NA. An eligible but unproven alternative is not automatically accepted. Prediction fallback is only a Shadow record of the unchanged causal initial, never a claimed EKF action. A rejected pose cannot re-enter through fallback or a comparator.

An out-of-bounds T0 may seed G if it is finite and otherwise safe; this does not expand final bounds. A detected event can end with no weak direction, no budget, collapsed candidates, unsupported evidence, or no admissible output. Record all of these distinctly. Nonfinite native output is a rejected measurement; because every frame uses an independent frozen causal initial and no feedback, subsequent frames may continue safely. Unhandled native/NN exceptions stop and preserve logs, never trigger automatic reruns.

## Reporting and guards

Compute the old trigger on the new run's nominal outputs, separately from newly added events; do not rerun/re-audit the R9 experiment to obtain its count. Explicitly tabulate local60–64 irrespective of results. Compare three selectors on the same deduplicated pool, but only Coverage Robust governs PROPOSED final alternatives. Native/mean comparisons never feed back.

Freeze input/source/binary/protocol/preregistration before one execution. Source parity and masks are checked before the first align in the R10 process. Preserve all raw candidates, invalid values as NA, seeds, G spectra, block losses, pairwise evidence, call journal, event/permission/recovery/final-observation states and timing. Protect old source by Git diff to START_SHA; do not run old audit programs or hash/review the R9 output archive. Input and current runtime source hashes, stable production files/HEAD/status are checked before/after. Internal disk reserve>=20GiB. Exclusive execution token; no second attempt after a real run.

Engineering state-machine correctness is not localization accuracy, basin certification, innovation or sustained realtime success. If no supported legal distinct alternative is observed, stop and recommend a separately approved candidate-generation improvement, not a threshold change or immediate new run.
