# R13 frozen pre-run mathematical contract

T_M_L maps LiDAR/body coordinates into frozen map M. Shared TP is the exact historical causal prediction, T0 the exact frozen R11 nominal native output. Neither is re-estimated. No online feedback or new trajectory is computed.

## Grid and coarse evidence

IDs0..44 iterate dx ascending(-.4,0,.4)m, dy ascending(-.4,0,.4)m, yaw ascending(-10,-5,0,5,10)deg. xi=(rho,phi)=(dx,dy,0,0,0,yaw_rad). Double seed TP Exp(xi), using unchanged R11 full SE3 exponential, so translation is TP.R J(phi)rho, not plain dx/dy. Native seed is the exact float cast consumed by PCL. Both double and float matrices must be valid SE3 and relative TP satisfy <=.60m/12deg; no clamping or amplitude adaptation. This search box is distinct from final .8m/5deg admission.

Original Source preprocessing/split is inherited unchanged. Only even-sector Training is passed to the coarse scorer; all training inputs must be finite and in even sectors. Deterministic subset indices: n=min(256,training_size); for n>1, round(i*(training_size-1)/(n-1)), i=0..n-1; n=1 uses0. Preserve training order and log indices/XYZ/sectors. Odd-sector validation coordinates/losses never enter this scorer.

For every actual native seed, query the same double-.15m frozen Target KD tree at each transformed coarse training point (query coordinates cast by PCL PointXYZ). E=mean(min(d_squared/.25,1)), overlap=mean(d_squared<=.25). All45 seeds are scored, including locally excluded seeds. This is a geometric initializer score, NOT the native PCL NDT objective or an independent correctness reference.

Local exclusion iff relative T0 translation<.25m AND rotation<4deg (strict). Sort eligible initializers by E ascending, overlap descending, ID ascending, no tie tolerance. Pick first. Pick first subsequent initializer with relative first translation>=.30m OR rotation>=4deg. At most2, no fallback third. Use actual float poses for these physical comparisons as well as coarse scoring. Candidate results cannot alter the selection: all5 events'225 scores and selections are logged and flushed before the first align.

## Native fine alignment and evidence

Only events local60..64/raw10700..10704. Same frozen Training/Target and NativeNdt resolution .8m, step .08, epsilon .001, max40, outlier .55. Each selected seed gets one capturedAlign; hard maximum10 total and2/event. Native optimizer pose is flushed before auxiliary fitness. Invalid SE3 stops; nonfinite metrics are preserved/rejected, never fabricated.

Nominal heldout losses are loaded unchanged only after all coarse selections are fixed. New outputs use actual inherited Coverage V2 validate. V3 construct/select/decide are imported without edits. Original T0 evidence retained. Final eligibility remains finite/converged, relative TP<=.8m/5deg, active evidence and overlap>=.50. Dedup <=.10m/1deg, legal representative priority, common-weight D>.02 unchanged. No supported replacement for these illegal-nominal events means NO_ACCEPTABLE_MEASUREMENT, only a Shadow fallback record.

Baselines are read-only R11 outputs with R12's already archived V3 decisions, separately labelled for the two historical generation arms. They do not supply extra candidates to R13 and incur no new align. R13 pool contains only shared T0 plus its own selected-grid outputs. Selector comparison uses identical R13 representative pools/evidence rules; native score/mean ranking does not replace the robust acceptance rule.

## Counts and limits

Raw non-nominal output differences and unique V3 non-nominal groups are reported separately. An eligible representative in a non-nominal group is an admissible distinct hypothesis. Robust preference also requires that distinct representative meet the frozen all-opponent D rule. Same-group representative promotion is not a new hypothesis. None establishes absolute correctness without independent reference.

Record Source preparation, map/grid/tree build, seed-grid/subset/logging phase,45-seed nearest-neighbor scoring, selection, exact coarse NN query count, native align, capture/auxiliary fitness phase, actual heldout validation, V3 decision and complete extra event phases. Process wall includes startup/loading/I/O; hashes lie outside algorithm process. RSS is shared process high-water, not independent-arm memory. No 10Hz claim.

All parameters and sample/float/tie rules above are frozen before the sole real experiment. No second grid/window or remaining-budget use. Missing independent reference does not block candidate generation; no GT fitting or error evaluation will be invented.

Known pre-run counterexample: if every sampled training point is farther than.5m under every grid seed, all E=1/overlap=0. The frozen tie rule can still select2 initializers. This is deterministic availability, not proof the coarse evidence is informative; no new quality threshold is introduced. Report actual score spread/ties and saturation.
