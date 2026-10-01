# tx90 complete duration enforcement

Both the single P3-100 and the P3-200 prefix have identical tx90 event/attempt
rows at stamp `1517157228164951397`. This confirms this repaired prefix, not
bitwise equality to the old alias-corrupted estimator trajectory.

Optimizer: ACCEPTED_UPDATE, 8 iterations; first surrogate cost
29.640314594899319, final candidate 21.667662999656720. Rank5, weak dimension1,
selected NIS8.5159969994942681 <=15.086, committed. Basis callbacks in inner
candidate acceptance remain0. No damping change.

| Field | Attempt1 | Attempt2 |
|---|---:|---:|
| Nodes before -> after | 41 -> 40 | 40 -> 39 |
| Span before (s) | 2.017077923 | 2.017057491 |
| Span after (s) | 2.017057491 | 1.916218042 |
| Duration trigger / node trigger | 1 / 0 | 1 / 0 |
| Removed stamp | 1517157226147873474 | 1517157226147893906 |
| Raw Schur max asymmetry | 7.2759576141834259e-12 | 2.9103830456733704e-11 |
| Consumed H / Hmm max asymmetry | 0 / 0 | 0 / 0 |
| Stored prior max asymmetry | 0 | 0 |
| Stored prior lambda_min | 0 | 0 |
| Solve-only jitter | 0 | 0 |
| Production validator | PASS | PASS |
| Removal result | SUCCESS | SUCCESS |

Attempt1 alone does not satisfy duration limit; the unchanged enforcement
while-loop proceeds to attempt2 and completes with39 nodes/span1.916218042s.
The finite/PSD/symmetric prior is stored without any rank completion or clamp.

Actual records: `RUN_P3_100/TX90_marginalization.csv`,
`RUN_P3_100/TX90_optimizer.csv`, `RUN_P3_100/events.csv` and the corresponding
P3-200 files. Old capsule regression uses the committed A3C-R1 NPZ by SHA:
`2f597628160e75f41bc4c13d9de015727c17b1755535ad806263f272ebcf0280`.

That old raw Schur is tested separately: old Eigen in-place evaluation gives
max defect1.1175870895385742e-8 and FAIL; repaired **actual production evaluator**
matches independent symmetric oracle exactly, defect0, eigen minimum0, and
unchanged validator PASS. This isolates expression evaluation from later
closed-loop differences.
