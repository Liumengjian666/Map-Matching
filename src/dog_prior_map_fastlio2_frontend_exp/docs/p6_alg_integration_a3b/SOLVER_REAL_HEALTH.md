# Solver / Window real-link health

| Check | P0-100 | P3-100 before tx83 failure |
|---|---:|---:|
| Main solver in completed runtime rows | `BLOCK_SPARSE_SIMPLICIAL_LDLT` | `BLOCK_SPARSE_SIMPLICIAL_LDLT` |
| Dense solver fallback count | 0 | 0 |
| Covariance available at completed LiDAR terminals | 49/49 | 31/31 |
| Covariance requests at non-LiDAR events | 0; rows say `NOT_REQUESTED_NON_LIDAR_EVENT` | 0; rows say `NOT_REQUESTED_NON_LIDAR_EVENT` |
| Window maximum | 40 nodes, 1.958915 s | 40 nodes, 1.958915 s |
| Observed node-count decreases | 30 | 12 |
| Visual events/factors | 0 / 0 | 0 / 0 in saved rows |
| Post-handoff IKFoM calls | 0 | 0 in saved rows |
| Logged state finite / quaternion norm | yes / max error `6.67e-16` | yes / max error `4.44e-16` |

The producer checks the marginal covariance call sequence around each event and throws if a dense marginal oracle request or an unexpected non-LiDAR request appears. Both completed paths before the P3 exception passed those guards. A1-R1/ A2D Schur and sparse covariance Release regressions remained in the 24/24 suite.

P3-100 is not a completed run. The failed transaction's status is `all_optimizer_candidates_rejected`; the exception path does not provide a persisted per-event solver trace. No claim of an accepted P3 optimizer update is made for tx83.
