# One candidate opportunity, with evidence limits

## Concrete issue exposed

At raw7570, the shared constant-velocity prediction has 4.8741° reference-relative rotation error. Full/Split native NDT reduce it to 1.8335°/1.9587°, yet their 5.6277°/5.8297° corrections violate the fixed 5° admission rule. Both converge and have Overlap@0.5m above 0.874. The unchanged rejection causes the common predictor to hold Full17 for frame19. This is a concrete admission/fallback limitation under the conditional Base benchmark reference; an NDT wrong basin has not been demonstrated.

## One small research hypothesis

**Bounded, one-scan deferred observation confirmation:** retain an out-of-bound but geometrically supported native measurement as a separate, uncommitted shadow hypothesis for exactly one subsequent scan. Compare its causal continuation with the prediction continuation using new scan evidence; expire it after that one scan. The existing immediate 5° gate stays fixed for the formal path during the mechanism test. No GT, extra modality, covariance fiction, or unlimited branch search would enter the confirmation logic.

This would test whether temporal confirmation can distinguish prediction error from bad alignment before permanently losing a potentially useful observation. It is an admission policy hypothesis, not another subspace solver or adaptive covariance method. It was **not implemented or tested in R16**, and its candidate budget, evidence partition and acceptance contract would require research-lead approval.

## Targeted recent-paper difference check

- [SuperLoc (ICRA 2025)](https://arxiv.org/abs/2412.02901) already predicts alignment risk and combines alternative odometry by directional observability. Risk gating and sensor-prior use cannot be claimed as new. The proposed narrow experiment would use a fixed-map NDT measurement and a bounded next-scan confirmation instead of claiming new observability analysis or multimodal fusion.
- [X-ICP](https://arxiv.org/abs/2211.16335) already classifies localizability and constrains ICP updates; [GenZ-ICP (RA-L 2025)](https://arxiv.org/abs/2411.06766) adapts the registration objective. R16's possible issue is after optimization, at measurement admission, so renaming their directional constraints would not address the observed failure.
- [LODESTAR (RA-L 2026)](https://arxiv.org/abs/2511.09142) already manages weak directions and retained information in an adaptive Schmidt-Kalman formulation. Temporal information retention itself is not novel; a precise bounded NDT-specific policy would need further nearest-method checking before any originality claim.

The four-paper check does not establish that deferred registration confirmation is absent from the wider literature. **NOVELTY_NOT_ESTABLISHED.** The evidence supports investigating admission harm; it does not establish that this idea improves localization or that the formal G flags identify erroneous updates.

## Unique NEXT

Before implementing the hypothesis, preregister one published long-corridor benchmark with a traceable reference and reproduce an actual native NDT failure. This supplies the missing positive failure case and a meaningful contrast to the current converged, reference-closer NDT example. No further window or threshold search is authorized in R16.
