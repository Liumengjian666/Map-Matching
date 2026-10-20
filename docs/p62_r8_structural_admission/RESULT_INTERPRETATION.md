# Pre-run interpretation rules

Frozen before the first R8 NDT align, after structural admission. This is an exploratory revision following R7's failed input qualification, not a first blind experiment.

## Separate scientific fields

- Natural events count only the frozen trigger on the 100 formal scans. Forced diagnostics never enter that count or any online recovery success rate.
- A distinct alternative is a finite, validation-evaluable member of the frozen deduplicated candidate pool other than T0. Count eligible alternatives separately: convergence, prediction bounds and overlap qualification still apply. Duplicate optimizer outputs are retained in the records but not counted as different branches.
- Main outcome precedence: no admitted window -> `NO_100_FRAME_STRUCTURALLY_ADMISSIBLE_WINDOW`; otherwise any distinct alternative -> `REAL_SHADOW_DISTINCT_CANDIDATES_OBSERVED`, explicitly identifying NATURAL versus DIAGNOSTIC_FORCED origin; otherwise a natural event -> `REAL_SHADOW_TRIGGERED_NO_DISTINCT_CANDIDATES`; otherwise -> `REAL_SHADOW_NO_NATURAL_TRIGGER`. A diagnostic-only distinct result also retains the separate formal-window status `REAL_SHADOW_NO_NATURAL_TRIGGER`.
- Holdout discrimination is `HOLDOUT_DISCRIMINATION_NOT_EVALUABLE` if no distinct alternative exists. With distinct alternatives, it is `HOLDOUT_DISCRIMINATION_DEMONSTRATED` only if the frozen robust rule actually accepts an eligible alternative with D(j,k)>0.02 against every other deduplicated rival. Otherwise it is `HOLDOUT_DISCRIMINATION_NOT_DEMONSTRATED`. Loss/ranking differences below this gate remain descriptive evidence, not acceptance.
- A singleton has no pairwise opponent: its margin is NA. Robust abstention versus a singleton Native/Mean nominal choice is not evidence of nontrivial discrimination.
- Native score, held-out mean and robust paired selectors must expose identical pool IDs for each phase/frame. CONTROL_FULL is not a recovery candidate. The cached nominal in diagnostics incurs no new nominal align.

## Boundaries

All costs include the separate CONTROL_FULL arm; report it separately from PROPOSED's shared SPLIT nominal and recovery. Peak memory is native-process ru_maxrss, not a sum of process RSS values. Map build is separate from per-frame work; preprocessing cached before the loop is separately timed. Validation/selection I/O overhead is included where the native timers place it, not silently relabeled pure algorithm time.

Geometry support, candidate separation and selection differences do not establish absolute pose correctness, independent basin minima, accuracy improvement, generalization, 10 Hz capability or novelty. Raw input is not newly deskewed. Fixed historical causal initials are never replaced by current shadow results. The map comes from the same recorded dataset and is not an independent cross-session truth source.

The full admission failures and motion-selection censoring remain primary method-applicability results even if the surviving window runs successfully. The motion-bin relationship is descriptive and nonmonotonic; it does not prove angular speed causes sparse sectors.
