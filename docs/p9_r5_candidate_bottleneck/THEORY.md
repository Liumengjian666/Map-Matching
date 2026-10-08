# R5 post-hoc candidate bottleneck attribution

This is POSTHOC FAILURE ATTRIBUTION of R4 at commit
06ff017831d2654a62c72f854926b9d3b459ce40. R4 remains FAIL with 17/40 strict
candidate coverage. No confirmatory gate is retested and no threshold is changed.
DUAL-U and the previously accepted scientific conclusions are unchanged.

Only committed R4 tables are inputs. No NDT, baseline replay, images or GT are
loaded. Both the 96-frame cohort and all recorded terminals stay fixed.

## Two contracts

The original R2B `evidence_for_frame` is called directly, including T0 as rank0,
PCL-converged terminals only, deterministic complete-link .2m AND2deg, maximum
raw score representative with lowest-rank tie, nominal-cluster exclusion, and
S >= S0 + EPS_SCORE (2.747604276e-4). Strict center separation is translation>.2m
OR rotation>2deg. Iteration-limit terminals with converged=1 remain eligible.
FULL263 maps archived seed_index0..262 to rank1..263, preserving original order;
B12 preserves its original rank1..12. Oracle cluster IDs never enter this call.
Applying the contract to FULL263 gives the ceiling within this recorded pool,
not a mathematical upper bound over every possible initialization. B12 starts
are projected/conditioned proposals, not a nested subset of FULL263 starts.

The historical oracle first requires supported clusters (seed_count>=5 and
basin_fraction>=.02). It admits centers separated from T0 and with
(best_supported_score - cluster_best_score)/N_source <=
.05 * max(1, abs(best_supported_score/N_source)). These conditions are retained
as archived labels; no oracle reclustering or relabeling is performed.

For score comparison, positive/near/negative means deltaS>EPS_SCORE,
abs(deltaS)<=EPS_SCORE, deltaS<-EPS_SCORE. This frozen numerical-carrier envelope
is not a label-tuned tolerance. Exact signed comparisons and the online inclusive
deltaS>=EPS_SCORE result are also recorded, including any exact boundary case.

## Funnel and attribution units

Raw-terminal and representative counts are reported separately. N0 indicates no
non-nominal cluster; N1 indicates at least one non-nominal cluster with an
inside-center representative; N2 indicates at least one strict-separated raw
terminal failing score; N3 indicates at least one strict-separated raw terminal
passing score. These flags may coexist. N3 after representative selection is
reported separately and must reproduce R4 strict eligibility.

The mutually exclusive per-frame hierarchy is: ELIGIBLE if a strict competitive
representative exists; otherwise REPRESENTATIVE_SUPPRESSION if a strict,
score-passing raw terminal exists; otherwise SCORE_REJECTION if a strict raw
terminal exists; otherwise INSIDE_CENTER_ONLY if a non-nominal cluster exists;
otherwise NO_NONNOMINAL_CLUSTER. This hierarchy avoids mistaking a raw N3 event
suppressed by representative choice for an online eligible candidate.

ID-disagreement analysis uses the frozen R4 single-ID admission rows, including
their overlap limitation. Every recovered terminal in the specified17 frames is
joined to its non-oracle cluster and representative. Center and score rejection
reasons are multi-label; counts of raw terminals, clusters, IDs and frames are
never interchanged. The same diagnostics are run for all56 NO_MAJOR controls;
that label remains an oracle proxy, not absence of all optimizer ambiguity.

R5 classifications are descriptive, not significance tests. Search-pool reach,
eligibility losses, and oracle/online definition differences are reported as
separate mechanisms with overlap. A MIXED conclusion is used if distinct
mechanisms affect frames; no unmeasured global reachability or causal dominance
is inferred. A unique next research question will be based on measured counts.

Offline analysis wall time is separate from all historical optimization costs.
NEW_NDT_CALLS=0; BASELINE_REPLAY_CALLS=0; VISUAL_EXTRACTION=0; GT_LOADED=NO.
