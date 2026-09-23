# PAPER-P3 mode transition report

`p3_mode_transition_summary.csv` records, for every selected frame, the
tracking error, objective at the offline reference, best sampled wrong-mode
objective (candidate at least 1 m from reference), score gap, longitudinal
separation, weakest translation information eigenvalue, and tangent alignment.

The transition is not a sudden single-frame failure.  At N2 (+8 s), a wrong
sampled mode already reaches 2254.17 versus 2019.89 at the reference.  At T0
(+10 s) the corresponding values are 2139.28 versus 1730.92; at the 0.5 m
crossing T1 they are 1414.52 versus 1138.53.  By T3 (+16.4819 s), the best
wrong mode is 1604.94 versus 844.944 at the reference and is separated by
8.6 m.  At F1 (+25.1553 s), it is 1755.34 versus 100.098 and separated by
9.4 m.  The wrong mode therefore becomes competitive before the persistent
2-metre crossing and dominates after it.

The multi-start endpoint table gives the basin evidence independently of the
1-D scan.  Because the selected seeds are deliberately very far apart, the
large number of endpoint clusters should be interpreted as attraction-basin
diagnostics, not as a final count of physical modes.

## Scientific decision

The most defensible P3 classification is **C: geometric degeneracy plus map
ambiguity**.  The Hessian shows a corridor-aligned weak direction in much of
the early/transition sequence, while the landscape and multi-start tests show
separated high-score alternatives before and after failure.  Later reference
Hessians become weak/indefinite, so a single local covariance cannot represent
the full uncertainty.

This supports investigating an intra-mode/inter-mode uncertainty model in P4,
but does not by itself prove a paper innovation or authorize online changes.
The scalar `ndt_fitness` fails because it is nearest-neighbour distance and,
even when replaced by the true NDT score, the wrong map mode can have a higher
objective than the reference.
