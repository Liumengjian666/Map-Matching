# PAPER-P6-I2-BASIN-STABILITY-PROBE

Result: `MINIMAL_PROBE_NOT_PROMISING`

## Replay and frozen input gates

- Branch/HEAD/remote: `paper` / `9ece6bc50b24b9bc8d619fc2ac84fdb74616480c` (remote paper matched at run start).
- Frozen map SHA-256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570` (`EXACT`); persistent path: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd`.
- Floor01 raw bag SHA-256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`; locked config SHA-256: `4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77`.
- Official GT SHA-256: `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`; calibration SHA-256: `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414` (both used only post-hoc).
- Baseline reproduction: PASS; see `baseline_gate.txt`.
- All three complete trajectories were written before this script opened official GT: baseline, COV3, GEO7.
- P6-I1 S7 objective wins: `71`; therefore GEO7 S0-S6 was required and ran 7 seeds on all 4127 scans.
- COV3 valid covariance frames: `4127/4127`; fallback frames: `0`; NDT calls: `12381`.
- COV3 used no visual input, visual seed, visual residual, or visual arbitration; no DCReg result selected a pose.

## Global post-hoc metrics

| Mode | Translation RMSE (m) | P95 (m) | Max (m) | Final (m) | Rotation RMSE (deg) | P95 (deg) | Max (deg) | Final (deg) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| BASELINE | 21.936133 | 46.757084 | 54.298935 | 47.293159 | 9.906078 | 19.089644 | 22.682864 | 2.272537 |
| COV3_OBJECTIVE | 1.770324 | 5.187582 | 6.711113 | 0.218039 | 5.326515 | 12.725238 | 15.819438 | 1.025194 |
| GEO7_REFERENCE | 0.983331 | 1.654417 | 4.091686 | 0.255718 | 2.789055 | 5.515775 | 12.983695 | 0.841294 |

## Persistent translation-error crossings

| Mode | Threshold (m) | Time (s) |
|---|---:|---:|
| BASELINE | 0.5 | 84.91932845115662 |
| BASELINE | 1.0 | 93.5928385257721 |
| BASELINE | 2.0 | 151.48321318626404 |
| BASELINE | 5.0 | 157.43361401557922 |
| COV3_OBJECTIVE | 0.5 | 83.50740337371826 |
| COV3_OBJECTIVE | 1.0 | 92.8868191242218 |
| COV3_OBJECTIVE | 2.0 | 144.92766046524048 |
| COV3_OBJECTIVE | 5.0 | 166.20793628692627 |
| GEO7_REFERENCE | 0.5 | 64.24420714378357 |
| GEO7_REFERENCE | 1.0 | 65.35361242294312 |
| GEO7_REFERENCE | 2.0 | none |
| GEO7_REFERENCE | 5.0 | none |

## Basin evidence and fixed decision gates

- `BASIN_ESCAPE=1`: 913/4127 (22.1%).
- Across all COV3 scans, `G_B` mean/median/P95 = `0.011347` / `0.002910` / `0.041253`; `Delta_t` mean/P95 = `0.090133` / `0.619145` m; `Delta_R` mean/P95 = `0.965006` / `3.236367` deg.
- Recovery retention `R=(E_base-E_cov3)/(E_base-E_geo7)`: `0.962440`.
- Escape post-hoc: better `474`, worse `439`, equal `0`; better/(better+worse) = `0.519168`.
- Decision verdict: `MINIMAL_PROBE_NOT_PROMISING`.
- Objective/capture evidence is not combined into a new scalar; retained outputs are `G_B`, `Delta_t`, `Delta_R`, and `BASIN_ESCAPE`.
- Distinct practical signals observation: `PARTIAL` — the cross-tab contains non-degenerate-proxy basin escapes and degenerate-proxy non-escapes, but the DCReg log is only a transaction-aligned proxy from a different closed-loop mode; this is descriptive evidence, not NDT observability validation or a novelty proof.

## DCReg proxy cross-tab

The table is transaction-aligned with the P6-I1 FULL_ROUTER DCReg log, whose closed-loop states can differ from COV3. It is **DEGENERACY PROXY ONLY**, not NDT-compatible observability `U_obs`.

| DCReg degenerate proxy | COV3 basin escape | Count | Fraction of all scans |
|---:|---:|---:|---:|
| 0 | 0 | 1276 | 0.3092 |
| 0 | 1 | 491 | 0.1190 |
| 1 | 0 | 1938 | 0.4696 |
| 1 | 1 | 422 | 0.1023 |

## Runtime and memory

| Mode | Stage | Count | Mean (ms) | P95 (ms) | Max (ms) |
|---|---|---:|---:|---:|---:|
| BASELINE | M0_NDT | 4127 | 7.128 | 19.856 | 101.169 |
| COV3_OBJECTIVE | M0_NDT | 4127 | 4.918 | 9.251 | 116.159 |
| COV3_OBJECTIVE | MPLUS_NDT | 4127 | 19.284 | 34.799 | 54.099 |
| COV3_OBJECTIVE | MMINUS_NDT | 4127 | 19.584 | 33.715 | 80.254 |
| COV3_OBJECTIVE | THREE_HYPOTHESIS_NDT_SUM | 4127 | 43.786 | 72.458 | 178.488 |
| COV3_OBJECTIVE | FULL_REPLAY_PIPELINE | 4127 | 55.454 | 86.668 | 191.795 |

COV3 has at most three registrations per scan. The <100 ms candidate gate uses the mean sum of NDT registration runtimes; `runtime_breakdown.csv` also includes full replay pipeline latency. RSS is in `memory_metrics.csv`; one target map is reused within each mode process.

## Representative frames

- Five escape cases were selected mechanically by descending `G_B`, preferring frames not flagged by the available DCReg proxy: 1998, 1997, 1982, 1968, 1385.
- Five stable cases were selected mechanically by descending `G_B` among `BASIN_ESCAPE=0`: 1380, 1382, 1383, 322, 2453.
- No local NDT Hessian/curvature was computed (`NOT RUN`); DCReg is only a proxy and did not screen/route candidates.

## Innovation boundary and limitations

- This is a single Floor01 offline closed-loop replay with frozen scan-end deskew clouds.
- GEO7/multistart is a brute-force reference, not a novelty claim.
- Visual is disabled in COV3; DCReg is a non-equivalent degeneracy proxy; official GT is post-hoc only.
- The only claim at this stage is a `NOVELTY_CANDIDATE`: local observability and capture-basin stability may be distinct registration-reliability dimensions; covariance-guided perturbations provide evidence about nearby attraction basins. Novelty search remains for the total controller.
- No early trigger, adaptive threshold, rho search, extra directions, visual fusion, DCReg adaptation, or ROS runtime change was performed.

Generated artifacts include per-scan trajectory/error, basin, covariance direction, degeneracy-proxy, timing, memory, and representative-case tables plus eight plots.
