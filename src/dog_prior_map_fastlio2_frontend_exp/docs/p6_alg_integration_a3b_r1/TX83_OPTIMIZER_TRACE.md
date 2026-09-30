# TX83 optimizer iteration trace

Source: `RUN_P3_100/trajectory.csv.r1_optimizer_trace.csv` (four rows, all flushed before throw). The tested production backend, damping schedule, maximum step and strict candidate acceptance were unchanged.

| Iteration | λ before → after | `||g||∞` | `||d||` applied | `gᵀd` | `dᵀHd` | predicted reduction | candidate cost | actual reduction | accepted |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 0 | 1e-6 → 1e-5 | 0.06473488 | 1.16505e-6 | -7.25541e-8 | 2.55810e-10 | 1.44852e-7 | 5.479582454477638 | -1.40669e-8 | no |
| 1 | 1e-5 → 1e-4 | 0.06473488 | 1.36876e-7 | -7.34216e-9 | 4.76986e-11 | 1.46366e-8 | 5.479582442305456 | -1.89476e-9 | no |
| 2 | 1e-4 → 1e-3 | 0.06473488 | 1.77110e-8 | -7.46909e-10 | 6.30113e-12 | 1.48752e-9 | 5.479582440487510 | -7.68150e-11 | no |
| 3 | 1e-3 → 1e-2 | 0.06473488 | 1.96811e-9 | -7.59967e-11 | 3.96303e-13 | 1.51597e-10 | 5.479582440412317 | -1.62181e-12 | no |

Each row's model predicted a positive decrease; each production candidate increased objective and was rejected. The optimizer returned `FAILED_ALL_CANDIDATES`, 4 attempts, with current cost unchanged at `5.479582440410695`. The initial tx83 factor itself was NIS-rejected, so no new LiDAR factor was in the graph.

Trace is opt-in (`P6_A3B_R1_DIAGNOSTICS=1`); a synthetic trace-on/off parity test confirms the trace does not alter final state, status, iteration count, or cost.
