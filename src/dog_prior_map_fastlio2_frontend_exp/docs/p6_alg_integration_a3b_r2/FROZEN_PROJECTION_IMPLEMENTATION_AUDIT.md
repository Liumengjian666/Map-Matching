# Frozen projection implementation audit

- `FrozenLidarProjection` is declared in `include/dog_prior_map_fastlio2_frontend_exp/window_factors.hpp`.
- `freezeLidarProjection()` is implemented in `src/window_lidar_factor.cpp`. Full-rank factors retain their fixed complete basis. Degenerate factors call `basis_relinearizer` only while the outer snapshot is being built.
- The freeze step stores the full basis, reliable rank, keyed observation identity, and symmetrized `B^T R B`; it validates rank, finiteness, orthonormal columns, and SPD projected covariance.
- `linearizeLidarFactorWithFrozenProjection()` computes `B0^T r_raw(state)` and `B0^T J_raw(state)` while returning the stored projected covariance. It has no call to `basis_relinearizer` and does not recompute the projected covariance.
- `FixedLagWindow::buildLidarIterationSnapshot()` rejects duplicate observation IDs and resolves each factor by observation ID plus timestamp.
- `blockLinearizedSystemWithLidarSnapshot()` checks a one-to-one keyed match against all active LiDAR factors. IMU, visual and prior contributions retain their existing paths.
- `objectiveWithLidarSnapshot()` is the candidate acceptance objective and uses the same frozen snapshot as the current `H/g/cost` assembly.
- The ordinary debug/dense-reference evaluators remain separate and may relinearize the basis. They are not used by the production optimizer acceptance path.

The optimizer trace names both current and candidate values as surrogate costs.
The older `current_cost`/`candidate_cost` members are retained as aliases for
compatibility; CSV headers use `surrogate_current_cost` and
`surrogate_candidate_cost`. Diagnostic dynamic-B values are explicitly labeled
`diagnostic_relinearized_basis_*`.
