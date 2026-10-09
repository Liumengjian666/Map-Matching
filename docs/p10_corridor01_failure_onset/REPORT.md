# P10 Corridor01 failure-onset coupled benchmark

`FINAL_RESULT = PAIRED_DIAGNOSTIC_NO_COUPLED_ACCURACY_GAIN_CAUSAL_FEEDBACK_BLOCKED`

`NEXT = RESTORE_TRUSTED_P2B_CAUSAL_REPLAY_CHAIN`

## Outcome

The fixed 0–35 s common-predictor paired diagnostic completed once on TX52–
TX397 (346 complete scans). Exact prepared-source count/hash parity and
historical nominal status/pose/score replay parity passed on 346/346 rows. The
run used one shared map instance and exactly 346 complete PCL align calls
(one nominal align per scan; zero extra full NDT aligns). Candidate generation
and recommendation did not read GT. No causal branch or IKFoM feedback was
run.

Strict ordinary NDT succeeded on 332/346 rows (95.9538%). The first strict
non-success was TX155 at +10.531506 s. All 14 non-successes were
`ITERATION_LIMIT_EXHAUSTED`; the longest consecutive run was 2 frames, so no
five-second persistent strict failure occurred in this fixed segment. This is
not the historical P2B failure-onset measurement.

The R6 event triggered on 90/346 rows. On 63 triggered rows the reconstructed
Anchor was valid; 27 triggered rows had no valid Anchor. There were 56
Weak-only and 57 Coupled recommendations. The Coupled conditional strong step
was selected on 21 rows. The 14 ineffective nominal rows did not yield a
refinement. Per-method extra computation stayed within the R6 budget: Weak-only
63 jets / 65 values; Coupled 124 jets / 114 values; untriggered rows had zero
extra jet/value calls.

## Paired matching and post-hoc relative drift

All numbers below are **PREFIX-ALIGNED RELATIVE DRIFT of per-frame
scan-to-map outputs**, not closed-loop localization trajectories. The one
proper SE(3), unit-scale transform was fitted to the Control nominal positions
in the first 10 s (98 samples), then reused unchanged. The Control's own
first-10 s position fit residual was RMSE 5.8036 m / P95 14.3323 m / max
18.9523 m. This is far from the historical P2B prefix tracking result and
signals a materially different/poor common predictor record. Therefore neither
the very large full-window errors nor their threshold crossing times are a
valid reproduction of P2B onset.

Selected output policy is frozen and non-GT: use the proposed candidate only
when R6 marks it `recommended`; otherwise retain nominal. Candidate proposals
that the online contract rejects are separately retained, but are not counted
as selected poses.

An initial post-hoc draft that measured every raw proposal even when the
non-GT policy rejected it is preserved under `superseded_posthoc/` for audit;
the tables and authoritative `posthoc_evaluation.json` use the selected-output
policy above.

| Paired method | translation RMSE / P95 / max (m) | rotation RMSE / P95 / max (deg) | selected alternatives |
|---|---:|---:|---:|
| Nominal | 36.5956 / 64.6598 / 75.6774 | 177.9497 / 179.8945 / 179.9975 | — |
| Weak-only | 36.6023 / 64.6598 / 75.6774 | 177.9561 / 179.8917 / 180.0000 | 56/346 |
| Coupled | 36.6025 / 64.6598 / 75.6774 | 177.9562 / 179.8917 / 179.9893 | 57/346 |

Relative to nominal, Weak-only translation RMSE was worse by 0.00663 m
(0.0181%); Coupled was worse by 0.00686 m (0.0188%). Coupled versus
Weak-only changed the selected per-frame translation error on 21 frames: 10
lower, 11 higher, and 325 unchanged; mean Coupled-minus-Weak error was
`+0.000410 m`. The Coupled strong correction therefore produced measurable
candidate differences but no useful accuracy advantage in this paired run.

Among selected alternatives only, the post-hoc relative translation error
improved on 22/56 Weak-only frames and worsened on 34; Coupled improved on
23/57 and worsened on 34. This descriptive evaluation did not feed back into
candidate choice. The objective score was higher for Coupled than Weak-only on
21 paired candidate rows, with mean difference `+0.219964` in the stored raw
score carrier; that follows the existing strong-step score-admission rule and
does not establish better localization.

The nominal relative error exceeded 0.5 m at the first sample (+0.143537 s),
but the large Control prefix-fit residual makes that crossing unsuitable as an
onset estimate. The archived P2B historical persistent crossings remain
unchanged: 0.5 m at +11.2375 s, 1 m at +13.1537 s, and 2 m at +16.4819 s.

## Runtime and resource use

Each arm's complete per-frame figure charges the shared scan read, frozen
rotational deskew and one nominal full alignment, plus that arm's refinement.
The paired total includes both refiners run sequentially and must not be read
as a single-arm runtime.

| Complete wall cost | mean (ms) | P95 (ms) | max (ms) |
|---|---:|---:|---:|
| Nominal | 59.012 | 243.586 | 280.806 |
| Weak-only | 59.916 | 243.595 | 280.815 |
| Coupled | 60.687 | 244.354 | 280.811 |
| Full paired work (both refiners) | 61.621 | 245.761 | 284.089 |

Peak process RSS was 53,352 KiB (52.10 MiB). The segment run wall time was
21.526 s. The P95 and maximum exceed the nominal 150 ms target; this run is a
research paired diagnostic, not a real-time guarantee.

## Experiment B and scientific conclusion

Experiment B is `NOT_RUN`. The historical P2B derived bag, frozen Run C
`result.bag`, and `ndt_determinism.csv` were confirmed absent at their original
data paths. The old P10 nominal predictor is explicitly a gyro/CV scan-to-map
diagnostic, not the trusted P2B inertial chain. Reusing it for three new
feedback branches would violate the task's causal-contract rule. No new
bootstrap, long-horizon gyro/CV tracking, or synthetic state initialization
was attempted.

This run does **not** establish a Coupled advantage or delay/avoidance of
historical drift. It establishes a reproducible paired candidate comparison
on the available P9 source and map, and it exposes that this common predictor
is already inconsistent with the historical P2B prefix-evaluation behavior.
The next required step is restoration of the trusted causal P2B replay chain;
only then can the fixed failure-onset window be used for a causal Weak-only /
Coupled localization comparison.

## Validation

- Existing Release P10/P9 CTest suite: 10/10 PASS.
- Fixed-window, alignment, and persistent-failure Python tests: 4/4 PASS.
- Paired runner self-test: PASS.
- Input/source/nominal parity: 346/346 PASS.
- No-GT evidence freeze preceded GT evaluation; `GT_LOADED_BY_EXECUTION=false`.
- `git diff --check`: PASS before commit.
