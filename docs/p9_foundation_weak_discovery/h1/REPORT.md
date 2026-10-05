# H1: weak-subspace concentration closure

WEAK_SUBSPACE_CONCENTRATION_SUPPORTED

This result supports a geometric association only in the frozen nine-frame oracle cohort; it does not establish search efficiency or a global NDT law.

## Historical parity

22 basins / 9 frames. Fraction mean 0.804192132450, median 0.923759723797, >=0.8: 13. Max per-basin error 1.83167365053e-07; gate PASS.

Original representative matrices are used. Both iteration-limit basins 616/P03 and 3796/P06 are retained. No reclustering or new optimization was performed.

## Frame-macro results

| tx | major IDs | n | W1 | W2 | W3 | S2 | difference | W2 pair rank |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 368 | P02 | 1 | 0.094886046 | 0.130826994 | 0.151257298 | 0.001418718 | 0.129408275 | 6 |
| 616 | P02;P03;P10;P12;P13;P17 | 6 | 0.735458614 | 0.882682371 | 0.890006578 | 0.047410160 | 0.835272211 | 1 |
| 2226 | P05;P09 | 2 | 0.042080109 | 0.907246540 | 0.913762106 | 0.049023003 | 0.858223537 | 1 |
| 2350 | P01;P03;P05 | 3 | 0.494000640 | 0.885888532 | 0.887636708 | 0.010237119 | 0.875651413 | 1 |
| 2722 | P04;P05 | 2 | 0.178983501 | 0.190743811 | 0.371585735 | 0.000396380 | 0.190347431 | 8 |
| 2846 | P03 | 1 | 0.545039305 | 0.545115526 | 0.644222465 | 0.276988895 | 0.268126631 | 5 |
| 3341 | P01;P02;P03;P08;P10 | 5 | 0.246605005 | 0.543077749 | 0.583961843 | 0.354201177 | 0.188876572 | 3 |
| 3796 | P06 | 1 | 0.001871867 | 0.999311459 | 0.999864586 | 0.000016602 | 0.999294857 | 1 |
| 3962 | P03 | 1 | 0.133676122 | 0.567915540 | 0.764555351 | 0.005542336 | 0.562373204 | 3 |

T_weak2=0.628089835905, CI95=[0.4260365915467964, 0.8140064295409548].
T_strong2=0.082803821321, CI95=[0.012164018950215813, 0.1739745775258611].
Difference=0.545286014584, CI95=[0.3354612544483959, 0.7558898261558473].
Weak>strong: 9/9; exact sign-flip p=0.001953125 (512 patterns).

## Random-subspace null and dimension ablation

| k | T_weak | k/6 | excess | null mean | null median | null95 | null99 | null max | p |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 0.274733468 | 0.166666667 | 0.108066801 | 0.166838869 | 0.161695036 | 0.271489796 | 0.317735863 | 0.477784649 | 0.045695430 |
| 2 | 0.628089836 | 0.333333333 | 0.294756503 | 0.332847138 | 0.331139788 | 0.455054524 | 0.512269465 | 0.635429917 | 0.000199980 |
| 3 | 0.689650297 | 0.500000000 | 0.189650297 | 0.500066116 | 0.499507441 | 0.629863422 | 0.680879796 | 0.801199480 | 0.006599340 |

Each null has 10,000 replicates and independent subspaces per frame; PCG64 seeds and software versions are recorded in results.json. Frame bootstrap also has 10,000 replicates; all frame draws are stored in bootstrap.csv.

WEAK2 overall eigenpair rank: 1/15; frame top1 4/9, top3 6/9.

Top5 eigenpairs: q1_q2=0.628089836; q2_q4=0.580902250; q1_q4=0.502279350; q2_q5=0.432887800; q2_q3=0.414916829.

LOFO weak>strong: 9/9; difference range [0.488534909332, 0.597270731980].

## Acceptance and limitations

A-G conditions: {'A_historical_parity': True, 'B_haar_p': True, 'C_weak_gt_strong': True, 'D_exact_p': True, 'E_ci_lower': True, 'F_frames': True, 'G_pair_rank': True}; H1=PASS.

WEAK2 is not the highest-projection pair in every frame: TX368 rank6, TX2722 rank8, TX2846 rank5. The whole cohort is historically selected and failure-enriched; temporal independence and generalization to unrelated NDT scenes are not established. Read THEORY.md for the conditional-null and resampling boundaries.

NEW_NDT_CALLS=0; GT_USED=NO; H2_RUN=NO. No production core or old archive files were modified.

## Reproduction

```bash
PYTHONDONTWRITEBYTECODE=1 python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/run_h1_concentration.py --self-test
PYTHONDONTWRITEBYTECODE=1 python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/run_h1_concentration.py
PYTHONDONTWRITEBYTECODE=1 python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/run_h1_concentration.py --audit
```

Source/input/CSV/theory/report hashes are recorded in results.json. The C++ helper has only Eigen dependencies and exposes no registration interface.

The runner builds its helper from the recorded source in a fresh /tmp directory; arbitrary prebuilt libraries are not accepted. Failed runs replace the current JSON/report with failure state; old CSVs are preserved but explicitly invalidated. The weighting self-test invokes the actual statistics() path on a synthetic duplicated-basin fixture.

NEXT=H2_MATCHED_BUDGET_BASIN_DISCOVERY_EFFICIENCY
