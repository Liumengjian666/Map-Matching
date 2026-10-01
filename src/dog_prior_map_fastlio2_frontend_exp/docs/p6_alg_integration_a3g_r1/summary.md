# A3G-R1 — offline pre-tx366 causal forensics

Dataset: **SuperLoc Corridor01**. Frozen catalog: 2777 scans, 51 pre-handoff,
2726 expected post-handoff terminals. START_SHA:
`88831c3906f05589031af8d06c95bc405d9d24d9`.

All 22 files in the committed A3G external ledger passed SHA256 and byte-size
verification. The ledger itself and official calibration are also hash-pinned.
No production changes, replay, point-cloud processing, image access, GT or NDT
execution occurred. Only new offline scripts and derived CSV/Markdown were added.

The last actual LiDAR commit was **tx182**, not tx201. Continuous completed
measurement rejection starts at **tx183** and lasts through tx365 (183 frames).
All those 183 rejections are map-support failures: 181 have no valid geometric
correspondences and two have insufficient support. They never reach selected NIS.
The failed tx366 is a separate pre-measurement covariance failure, not the 184th
map-support rejection.

First correspondence loss is **tx166–172**, with subsequent recovery. Strictly
continuous `NO_VALID_GEOMETRIC_CORRESPONDENCES` starts at **tx188** (178 frames).
In all 188 no-correspondence records, NDT reports converged but iterations=0 and
objective=0. In the audited window its terminal is effectively the prediction
seed, not independent evidence of successful registration.

Active LiDAR count last exceeds zero at tx201; **tx202–366 = 165 zero-count
terminal records**, including the failed terminal. QR history directly shows
tx202 consuming the last committed tx182 state/factor. Historical information
remains in the square-root prior; zero active raw factors does not mean zero
historical LiDAR information.

First completed translation increments >0.5/1/2/5 m occur at tx160/176/215/285.
Maximum is tx365: 10.221731186 m. Thus growing motion already precedes permanent
rejection; a single threshold crossing cannot identify the first physically
wrong prediction or NDT pose without stronger evidence.

P15 is valid/full rank throughout tx140–220 and remains available through tx365.
Deskew timing/count/provenance invariants pass for all 81 audited frames, but
large displacement maxima and increasing deskew magnitude are disclosed, not
used to certify physical deskew correctness.

PRIMARY_ROOT_CAUSE_CLASS = **A3G-R1-H / ROOT_CAUSE_NOT_ISOLATED**.
The direct sustained admission-blocking mechanism is isolated; its upstream
origin (prediction versus NDT/map-overlap/geometric implementation) is not.

TX366_COVARIANCE_FAILURE = **DOWNSTREAM_OF_LIDAR_MEASUREMENT_STARVATION** in
observed chronology. This is not proof that starvation alone causes the rank
failure, or that the numerical rank rule is correct.

A3G_R1_PRE_TX366_ROOT_CAUSE_ISOLATED = **PASS under the task's forensic-delivery
criteria, which explicitly permit honest class H**. Upstream root-cause isolation
is NOT achieved. A3G full engineering gate remains FAIL; core freeze remains NO;
READY_FOR_FORMAL_EXPERIMENT=NO. See [ROOT_CAUSE.md](ROOT_CAUSE.md) and
[DECISION_AI_HANDOFF.md](DECISION_AI_HANDOFF.md). STOP; no repair authorized.
