# Expression repair, not estimator redesign

Schur remains `S = Hrr - Hmr.transpose()*solve(Hmm,Hmr)` and
`b = gr - Hmr.transpose()*solve(Hmm,gm)`. Production elimination still consumes
only the existing prior plus factors incident on the oldest state. Retained-only
factors remain active/relinearizable. Stored S now comes from an independent,
fully evaluated average of raw S and its transpose.

Unchanged validation: maximum absolute symmetry defect 1e-8, prior PSD eigen
tolerance 1e-6. Unchanged solve-only jitter and both absolute LDLT pivot gates.
No prior epsilon-I, eigen clamp, rank completion or production pseudoinverse.

The A3B-R2 outer-relinearized/inner-frozen projection contract is untouched:
the same frozen B and B^T R B are used in H/g and candidate acceptance; a new
outer iteration may update B. No basis callbacks were added to acceptance.

The rank-five regression uses nonzero position/rotation residuals, checks
selected residual against independent B^T raw, selected covariance against
B^T R B, and checks the pose 6x6 information has rank five and a null pose
direction. It does not mistake velocity/bias nullspace for the omitted pose
direction. Symmetry repair cannot add measurement information along that
direction.

The multi-removal regression compares a full-graph two-state Schur oracle
with new prior plus retained-only factors after two removals in one enforcement.
Each trace records exact consumed/Hmm/stored symmetry, sequential lifecycle,
and continuity of first stored prior to second incoming prior. Final revision
advances twice, optimized feedback remains current, and active factor/ID counts
match the retained graph. OFF/ON captures give identical states, priors,
gradients, decisions and active counts. No later-attempt failure was injected;
batch rollback behavior is not redesigned or certified by this test.

The committed real tx90 capsule is reused by SHA, reconstructed only in a
temporary test directory. C++ tests retain the unsafe expression as legacy
forensic evidence, call the actual production evaluated helper, and compare it
with an independent out-of-place oracle under unchanged validator thresholds.

These tests establish local mathematical and engineering contracts, not
localization accuracy, a proof of global observability, or formal performance.
READY_FOR_FORMAL_EXPERIMENT = NO.
