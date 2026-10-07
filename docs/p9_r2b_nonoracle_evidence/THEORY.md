# P9-R2B: non-oracle competing-terminal evidence gate

## Scope and frozen construction

Start at R2A commit2712573a8d3be1768ba59f6aa92f184d0949d19f. DUAL-U remains within-basin directional observability plus nonlocal ambiguity. This experiment tests a descriptor of the already produced competing terminals, not pose switching, a probabilistic interpretation, online runtime, or a certificate of multiple stationary basins. No new NDT call is permitted.

The standalone construction process reads only R2A `ndt_runs.csv`, `predictor_parity.csv`, and numerical carrier diagnostic `score_carrier_sensitivity.csv`, verified byte-for-byte against the start commit. It does not import prior evaluation modules, load labels/canonical terminals/GT, or accept their paths. Only COND_WEAK2, prefix budgets4/8/12 and secondary16 are considered. All converged returns are retained, including PCL-converged iteration-limit returns, which are marked separately.

T0 is the archived nominal matrix, not a score-selected replacement. S0 is its recorded R2A raw PCL score. EPS_SCORE is fixed by the user at `2*1.373802138e-4 = 2.747604276e-4`, derived from the earlier carrier audit, not fitted to labels. Raw score S is maximized; exact PCL energy E=-S is minimized. Numerical provenance covers the measured carrier audit rather than claiming a rigorous bound for all possible poses.

## Geometry and representatives

Include T0 as member rank0. Geodesic rotation/Euclidean translation use the same normalized pose convention as R2A's descriptive clustering. Complete-link agglomeration uses pair distance `max(dt/0.2m, dr/2deg)`. Merge the pair of clusters with the lowest maximum inter-member distance while that distance<=1; exact ties use lexicographically ordered member-rank tuples. Each cluster must satisfy both geometry bounds for every member pair. The group containing rank0 is `NOMINAL`; other IDs are local descriptors, never oracle IDs. Representatives maximize S, with lower probe rank winning exact ties. The nominal group remains nominal even if a candidate is its best-scoring representative.

The prescribed competitive set is all non-nominal groups whose representative has `S_k >= S0+EPS_SCORE`. No labels, GT, IDs or probability enter this rule. Complete-link group separation does not by itself imply that every representative lies outside the nominal center's admission ball. Representative-center distance and any such boundary cases are therefore recorded, not silently filtered or used to tune a new rule.

## Fixed chart and tensor

Displacements use the exact P9 Matrix4f-carrier map product chart, with the spatial quaternion logarithm and float translation subtraction:

`xi_k = [(t_k-t0)/0.8m; Log_spatial(R_k R0^T)]`.

The math-only C++ extraction has no PCL dependency or optimizer path. It preserves the existing matrix chart rather than using SE3 body log or historical seed coordinates.

`A_comp(B) = sum_k xi_k xi_k^T`, with an empty sum equal to the6x6 zero matrix.

`U_comp = sqrt(lambda_max(A_comp))` is the only classification score. No division by B or cluster count, probability weighting, softmax, or covariance interpretation is allowed. Secondary predeclared descriptors are trace, competitive cluster count, maximum/median displacement norm and best competitive score advantage divided by frozen source count. Empty-set descriptors are0. Tensors/eigensystems and all these values are frozen in `nonoracle_evidence.csv` before evaluation opens labels.

## Evaluation contract (fixed before loading labels)

The9 major/23 no-major frame labels are read from the committed H1 frame table only after the evidence file, clusters and competitive terminals are hashed. All32 frames are retained. These are frozen oracle-cohort proxy labels; NO_MAJOR is not proof of absence of optimizer ambiguity.

Frame ROC-AUC uses average ranks for exact score ties. The one-sided frame-label Monte Carlo test uses AUC as its statistic,10000 shared permutations, NumPy PCG64 seed20261011, preserving9 positive labels. `p=(1+count(null_auc>=observed_auc))/10001`. Nominal p-values for the three ordered primary budgets are reported; they are not a multiplicity-adjusted universal claim.

LOFO chooses a threshold using only31 training frames, maximizing balanced accuracy among midpoints of training scores and +/-infinity. Ties choose the highest threshold (conservative positives). Prediction is U>=threshold. Report pooled held-out balanced accuracy, ordinary accuracy, sensitivity and specificity separately; the gate uses balanced accuracy.

To operationalize the user requirement that no single frame drives the result, deletion of any one frame must preserve ROC-AUC>=.80 and nested LOFO balanced accuracy>=.80. Each nested held-out classification is trained on the remaining30 frames. This stability rule reuses the primary numerical thresholds and is frozen before observing evidence/labels; no per-frame exception is allowed. Report all omission ranges even when the primary gate fails.

Pick the smallest B in4/8/12 with AUC>=.80, p<.05, LOFO balanced accuracy>=.80 and the omission stability rule. B16 is secondary only. If only B16 passes, report NONORACLE_EVIDENCE_TOO_EXPENSIVE; if none passes, ORACLE_DISCOVERY_WITHOUT_USABLE_ONLINE_EVIDENCE. No other secondary feature can rescue the gate.

Only a primary pass permits loading U_obs for principal-angle measurement. No directional measurement changes evidence. Archived R2A GT annotations may be read only after classification/gates are frozen, solely to identify the two departing no-major risk examples; no new trajectory, GT fit or GT-derived threshold is used. Evidence never chooses or replaces a localization pose.

## Verification

Freeze construction code, input/theory/library hashes and the exact evidence SHA256. Evaluation preserves this snapshot and independently audits membership coverage, complete-link geometry, maximizing representative, competitive threshold, exact chart tensor, empty set, PSD, scalar/CSV/JSON values, frame denominators and training-only thresholds. Release build, P9 tests, hash audit and diff checks precede explicit-path commits. The stable workspace and production localization core are untouched; no push is executed.
