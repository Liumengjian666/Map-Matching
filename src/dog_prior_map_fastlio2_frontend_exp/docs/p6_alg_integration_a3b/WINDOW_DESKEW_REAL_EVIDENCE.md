# Window-owned real deskew evidence

The producer wrote one evidence row immediately after each successful `deskewScanWithWindowState()` call. The sidecar records transaction, raw scan start/end, raw point count, min/max point timestamp, active Window anchor pose, predicted scan-end pose, deskew output count, and displacement mean/P95/max. A count mismatch is a hard error before preprocessing/NDT.

## P0 complete run: first, middle, last

| tx | start ns | end ns | raw/output points | point min/max ns | displacement mean / P95 / max (m) |
|---:|---:|---:|---:|---|---:|
| 52 | 1517157224231677055 | 1517157224332516266 | 28836 / 28836 | exactly scan interval | 0.005604 / 0.011782 / 0.117388 |
| 76 | 1517157226652194977 | 1517157226753034426 | 28857 / 28857 | exactly scan interval | 0.163943 / 0.399427 / 2.428192 |
| 100 | 1517157229072650909 | 1517157229173490358 | 28917 / 28917 | exactly scan interval | 0.180894 / 0.358338 / 1.174253 |

Across all 49 P0 rows, `min(point_stamp)==scan_start`, every point stamp stayed in `[scan_start, scan_end]`, and raw/deskew output counts matched. The maximum individual point displacement in the run was `8.607300 m`; this is only evidence of a non-identity coordinate transform, not a quality bound or accuracy result.

P3-100 recorded 32 deskew sidecar rows through transaction 83, including the scan whose following optimizer call failed. That row contains 28,943 points, exact min/start and max/end alignment, and displacement mean/P95/max `0.144848 / 0.309312 / 2.945280 m`. Thirty-one earlier terminal scans completed their event; tx83 did not complete optimization.

Full per-scan evidence is retained in each run's `trajectory.csv.deskew_evidence.csv`. No map pose, GT, or legacy deskew cloud was used to generate these clouds.
