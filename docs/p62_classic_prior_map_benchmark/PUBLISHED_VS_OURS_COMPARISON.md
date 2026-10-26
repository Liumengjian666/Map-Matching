# Published versus ours: no fabricated direct ranking

Published Table II is verified in CLASSIC_BASELINE_LITERATURE.md and its six rows are in PUBLISHED_BASELINE_RESULTS.csv. No external algorithm was installed or rerun.

Our Outdoor01/02 complete-sequence result is **NOT_RUN_NOT_AUTHORIZED_THIS_ROUND**. OUTDOOR01_FULL_RESULTS.csv and OUTDOOR02_FULL_RESULTS.csv are intentionally not generated without actual runs. NR must never be plotted as zero or interpreted as failure rate0. There is no new accuracy, recovery, runtime or advantage measurement this round.

|Dimension|Published classic baselines|Planned C+G|Current compatibility|
|---|---|---|---|
|Sequence/sensor|Outdoor01/02 MID360|Same designated raw assets|Official metadata supported; raw payload checks pending|
|Map/reference|Dataset map / batch reference|Same map / same map-frame reference|Coordinate definition official; actual runtime chain pending|
|Initialization|Exact outdoor baseline poses NR; author automatic global initialization|One declared anchor and R30-accepted causal initial state|Not certified equivalent|
|Reset|Restart corrupted methods|Continuous no reset, failed outputs retained|Different by design|
|Metric|ATE[m], centre/± statistic NR|Direct position/rotation RMSE/P95/Max, ATE/RPE definitions frozen|Not interchangeable|
|Alignment|Exact evo flags NR|No post-hoc SE3/Sim3 fit|Not certified equivalent|
|Coverage|Exact restarted scoring intervals NR|All input after legal anchor, missing references/output coverage disclosed|Not certified equivalent|
|Resources|Author implementation/platform|Our machine, full NDT/EKF and player scope|Conditional only|

**Current grade: B published context, no Grade-A direct benchmark.** Same dataset alone cannot overcome the reset/statistic/initialization differences. If a future published-compatible restart protocol becomes fully specified, it must be a distinct predeclared experiment, not edits to a failed continuous trajectory. Until then do not turn ATE centres into RMSE, calculate improvement percentages, infer an RMSE from the printed pair, or present a SOTA ranking.

The main paper table should place published and our rows in clearly separated protocol blocks. R23–R25 internal ablations remain auxiliary mechanism evidence; R29 Azure negative evidence remains visible. Neither fills the missing Outdoor observations.
