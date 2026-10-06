# R1C3A numerical contract closure

FINAL_RESULT = **SUPPORT_TRANSITION_IS_MATERIAL**

This is a reclassification of archived roots and already-executed support transitions. No alpha was advanced; no proposal search, full NDT alignment, GT, or EKF was used.

## Independent roots

| root | stationary | predictor | anchor ≤0.2m/2° | exact support | numeric support | eligible |
|---|---|---|---|---|---|---|
| 2226/P05 | 1 | 1 | 1 | 1 | 0 | 1 |
| 2350/P01 | 1 | 0 | 1 | 1 | 0 | 0 |
| 3341/P02 | 1 | 1 | 1 | 0 | 0 | 0 |
| 2722/P05 | 1 | 1 | 1 | 1 | 0 | 1 |
| 616/P10 | 1 | 0 | 1 | 1 | 0 | 0 |
| 3796/P06 | 0 | 0 | 1 | 1 | 0 | 0 |

2350/P01 and P05, and 3341/P02 and P03, are each counted once.
Root displacement and energy context (the historical .02m/.2deg gate is diagnostic only). Start provenance is checked from the archived first corrector check against T0 support and the frozen R1C3 root call site:

| root | ||v_root|| | translation m | rotation deg | old gate | nominal association | u=v=0 verified | start support=T0 | oracle info | dynamic energy drop |
|---|---|---|---|---|---|---|---|---|---|
| 2226/P05 | 0.0020795326755445663 | 0.0012425959575921297 | 0.079237527439433228 | 1 | 1 | YES | 1 | NO | 0.0018501099706864643 |
| 2350/P01 | 0.004569502251243784 | 0.0018724440596997738 | 0.22485936347366312 | 0 | 1 | YES | 1 | NO | 0.003924136827841096 |
| 3341/P02 | 0.010564085919125582 | 0.0068086208775639534 | 0.35857279788745616 | 0 | 1 | YES | 1 | NO | 0.0025031061274032318 |
| 2722/P05 | 0.0020778214413980697 | 0.00073965912451967597 | 0.10660713291791377 | 1 | 1 | YES | 1 | NO | 0.00034644138850259409 |
| 616/P10 | 0 | 0 | 0 | 1 | 1 | YES | 1 | NO | 0 |
| 3796/P06 | 0.0089465411405762491 | 0.0029551859479397535 | 0.46686432598127964 | 0 | 1 | YES | 1 | NO | -0.00076581975361245114 |


## Predictor tests

| node | stable h | delta alpha | EPS_DV_NUM | max FLOAT/DOUBLE step gap in stable h | max adjacent FLOAT spread in stable h | FLOAT raw FD | DOUBLE raw FD | valid |
|---|---|---|---|---|---|---|---|---|
| 616/P10/ALPHA_0P68125 | 0.0040000000000000001…0.001 | 0.0050000000000000001 | 3.5466910402299991e-05 | 1.2208740984802524e-05 | 7.462367027309366e-06 | NOT_RECORDED | NOT_RECORDED | 1 |
| 616/P10/NEW_SUPPORT_ALPHA_0P68625 | 0.0040000000000000001…0.000125 | 0.0050000000000000001 | 0.0010610363923854798 | 0.000717610952521207 | 0.0006429498034425317 | NOT_RECORDED | NOT_RECORDED | 1 |
| 616/P10/OLD_SUPPORT_ALPHA_0P68625 | 0.0040000000000000001…0.00050000000000000001 | 0.0050000000000000001 | 7.4538143953070218e-05 | 5.433166936150694e-05 | 4.2242565425690846e-05 | NOT_RECORDED | NOT_RECORDED | 1 |
| 616/P10/ROOT | … | 0.050000000000000003 | 2.4154465896221168e-05 | NA | NA | PASS | PASS | 0 |
| 2226/P05/ROOT | 0.0040000000000000001…0.001 | 0.050000000000000003 | 0.00014461730395191058 | 5.441007812954457e-05 | 5.38226675048726e-05 | FAIL | PASS | 1 |
| 2350/P01/ROOT | … | 0.050000000000000003 | 3.7629351281623447e-05 | NA | NA | FAIL | PASS | 0 |
| 2722/P05/ROOT | 0.002…0.00050000000000000001 | 0.050000000000000003 | 2.5127375851564605e-05 | 8.568160173647234e-06 | 6.491798889917067e-06 | FAIL | PASS | 1 |
| 3341/P02/ROOT | 0.0040000000000000001…0.00050000000000000001 | 0.050000000000000003 | 5.047940775606216e-05 | 3.3316720671393924e-05 | 3.8766675363227224e-05 | PASS | PASS | 1 |
| 3341/P02/ROOT_DYNAMIC_SUPPORT | 0.0040000000000000001…0.00050000000000000001 | 0.050000000000000003 | 5.047940775606216e-05 | 3.2498361979830136e-05 | 3.8114659164475466e-05 | PASS | PASS | 1 |
| 3796/P06/ROOT | … | 0.050000000000000003 | 5.7996549444694161e-05 | NA | NA | FAIL | PASS | 0 |

The complete per-h values, Hvv spectra, conditioning, predictor vectors, and step disagreement are in `predictor_multih.csv`. Historical independent directional FD rows are retained separately in `raw_derivative_diagnostic.csv`.

## Existing support transitions

| group | pairs | numerically equivalent | both stationary | both dynamically self-consistent | Δv range | energy-gap range |
|---|---|---|---|---|---|---|
| 3341/P02=P03 ROOT | 8 | 0 | 8 | 0 | 0.00077381…0.00217883 | 0.000245571…0.00376191 |
| 616 .68625 OLD_SUPPORT | 8 | 0 | 8 | 0 | 0.000716837…0.0030281 | 0.00132009…0.00456391 |
| 616 .68625 NEW_SUPPORT | 8 | 0 | 8 | 0 | 0.00102929…0.0030281 | 0.000212671…0.00456391 |

Exact support equality is not inferred from hash equality alone; pair decisions use the frozen R1C2 numeric envelopes. Across these archived pairs the fixed-support stationary solutions are materially separated by the stated numeric envelopes. However, the optimized endpoints are not dynamically support-self-consistent in these records, so this does not certify two distinct dynamic local minima. Changed-point fractions remain descriptive.

Detailed calculations are in `support_pair_stationary.csv` and `support_equivalence.csv`.

PRIMARY_ROOT_CONTRACT_CLOSED = False
616_BOUNDARY_SUPPORTS_ALL_NUMERICALLY_EQUIVALENT = False
NEW_FULL_NDT_CALLS = 0

NEXT = STOP_SMOOTH_SUPPORT_FIXED_CONTINUATION_AND_USE_DISCRETE_SUPPORT_TRANSITION_EVIDENCE
