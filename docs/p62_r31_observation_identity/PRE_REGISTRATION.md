# R31 fixed execution plan, before new reference evaluation

START edb4e913bdbaec2a493252cd97b7c842f9e41109 fetched from the approved remote branch. Production change: LEGACY(default) / STRICT_FAIL_CLOSED opt-in observation identity. Existing candidate quality is required; altered terminal poses are rejected before prior-map history, pose/path/TF publication. No quality, motion threshold, input, extrinsic, optimizer or EKF/OOSM change.

Unit gate: eight required cases pass and the real raw62 LEGACY matrix is reproduced; STRICT refuses it, without NDT or EKF. The complete actual-node behavior is checked by per-scan state-before/after and publish counters, full candidate/sent pose identities and original EKF receipts. PCL's float rotation is compared through the identical normalized quaternion message representation; this is serialization, not post-hoc alignment or a new admission tolerance.

Fixed runs, in order, each once:
1. Koide EASY C0 / B(G-only) STRICT, anchor raw4, evaluation raw5–2027 (2023).
2. Koide EASY C0 / C(C+G) STRICT, same2023 input frames, independent processes/state.
3. Newer College W1/C3 / C+G STRICT, anchor2600, evaluation2601–2660 (60), only if the frozen R23 input/initialization/map identities pass. Conditional Base reference retained, not independent truth.

Only these2 full runs plus1 historical regression. Extra quota uses the inherited <=2/event, <=12/disjoint60 input frames; NC's60 frames have <=12 total. No preliminary NDT trial or extra LEGACY trajectory; R29 B/C and R24 G/W1/C3 are historical controls. R30's target-order repair is already present; R29 Tile order was correct and is tested, so intended Tile arithmetic remains unchanged. No engineering/algorithm benchmark equivalence is claimed for the global-PCD path.

No future/reference evaluation pose is read by supervision. Only frozen anchor initialization and immutable sensor/map metadata are used. All online outputs are sealed before offline reference scoring. Native candidates, certified pose, proposed limit pose, actual sent observation, rejection reason and unmodified navigation history on rejection are retained. EKF application is counted from actual OOSM/corrected receipts, not from a decision to publish.

Evaluation follows R29/R24: direct unaligned map-frame translation/rotation RMSE/P95/Max, legal-reference coverage, missing reference remains unknown, full output including propagation/failure retained; sustained high error/recovery requires3 legal consecutive frames at>.5m or5deg / below both. Full-cycle timing excludes one-time startup and is reported separately; simultaneous RSS sampled50ms. Published versus fused counts remain distinct. No10Hz or new accuracy claim from identity tests.

Engineering PASS requires all strict published observations to equal their certified terminal representation, no navigation-state mutation or publication on identity rejection, legitimate time/OOSM handling and budget. Performance PASS is separate; long observation starvation or unchanged/worse full tracking is a failed localization result even if all engineering invariants pass. No new gate or threshold is added to repair a negative result.
