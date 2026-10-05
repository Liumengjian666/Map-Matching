# P9-R1B fixed-u strong attraction and support controls

This is a bounded offline diagnostic, not an online discovery algorithm. No GT,
weak grid, transported W, posterior probabilities, or EKF is used. Exact PCL
optimization energy is closed; posterior NLL is not closed.

## Inputs and coordinates

Start commit: a1ff5fce3c5c5a8ab8257108b65ca6fbcf136aac. The seven GROUP A
targets and their CLOSED_CANONICAL_TERMINAL, u_b, v_b are read unchanged from
R1A canonical_oracle.csv. Prepared source hashes, target count, U_obs provenance
and pose reconstruction are checked by the existing FrameContext. Map SHA256:
2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570.

eta=[delta_t_map/0.8m,delta_theta_map]; t=t0+0.8 eta_t;
R=Exp(eta_theta)R0; eta=W u_b+S v. W/S and u_b never change. No SE(3) log
is substituted for the existing map-product coordinates.

## Parameters frozen before experiments

1. Main beta seeds: beta=0,.1,...,1, v_init=beta v_b, existing Newton20.
2. Local cross: v_b +/- d e_j, d=.02,.05,.10, all four strong axes.
3. Bisection: each adjacent main-beta interval with different endpoint groups;
   recursively split both differing halves (not assume one monotone boundary),
   stop at width<=.01 or eight depth levels. Seeds always beta v_b.
4. Support/value-only path: beta=0,.01,...,1. Dynamic support, frozen T0 support,
   frozen closed-canonical support are evaluated at exactly the same poses.
5. Causal objective control: beta=0,.25,.5,.75,1 for dynamic, frozen T0 and
   frozen canonical, all using the *same* R1A pattern100 solver/settings.
   Dynamic pattern is necessary to avoid confounding objective with solver.
6. Endpoint groups: deterministic greedy complete-link admission, .2m AND
   2deg, matching R1A basin proximity. The closed canonical is a fixed anchor
   member of CANONICAL; all admitted endpoints must be near every group member.
   Main seeds are processed in increasing beta. Subsequent bisection/local-cross
   endpoints are classified against frozen main groups, without changing them;
   unmatched endpoints receive distinct deterministic supplemental complete-link
   group IDs (NEW_01, NEW_02, ...), never a shared unknown label. Supplemental
   groups may admit subsequent endpoints; main groups are not changed.
   NOMINAL/OTHER labels describe proximity only, not correctness or stationarity.
   Canonical-radius capture is also reported separately: complete-link grouping
   and distance to a single canonical anchor are not identical predicates.
7. Full refine: one actual endpoint representative per distinct main Newton
   endpoint group. Choose lowest dynamic-energy main endpoint, tie by beta;
   never synthesize/average a pose. Canonical-neighborhood escape requires a
   pre-refine pose within .2m AND2deg and a post-refine pose outside either bound.
   Full-refine parameters stay .8m/.08/1e-5/80/outlier .55.

## Measurement definitions and inference limits

All energies are -exact PCL score/N with the unchanged prepared source count.
Frozen objectives keep per-source-point Gaussian leaves, not nearest-neighbor
correspondences. They are diagnosis only. Every accepted strong step is logged.
Frozen callbacks report their fixed support; endpoints also have separately
evaluated *dynamic* support, so fixed callback support cannot masquerade as a
dynamic support diagnostic.

Endpoint clustering is not a certificate of multiple stationary attractors.
Newton20 and pattern100 are finite budgets. Their termination, accepted moves,
canonical proximity and local capture rates are reported separately. The beta
boundary is only a section along 0->v_b and can reflect a group threshold,
support behavior or budgeted solver; it is not a 4D volume or exact separatrix.

Path diagnostics use largest adjacent support change and energy-slope change
(absolute second finite difference on the .01 lattice). All locations are
reported. For descriptive association only, a boundary interval expanded by
.02 beta is compared with the top five path support-switch and energy-cusp
locations; YES means both kinds intersect, PARTIAL one kind, NO neither. This
is not a causal proof or a tuned algorithm threshold. Frozen slopes/curvature
are reported on the same path, without adjusting pose.

Each discovered main group representative is evaluated under all three
objectives. A separated feasible endpoint with dynamic energy lower than the
canonical-group representative by >1e-6 demonstrates that a global lower
envelope would not retain that representative at this u. It does not certify
the global minimum, prove the representative is a strict dynamic local
minimum, or define a posterior. The canonical CLOSED pose energy is also
reported independently to avoid selection artifacts.

Frozen controls can strengthen causal attribution only when compared with
the same dynamic pattern seeds/budget. Budget-limited differences alone cannot
prove intrinsic multiattractor structure or support as its only cause. Results
must distinguish robust neighborhood capture from unresolved endpoints.

The formal experiment does not modify the paper's profile formula in advance.
