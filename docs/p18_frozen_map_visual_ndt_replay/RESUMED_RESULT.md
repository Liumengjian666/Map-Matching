# P18 later recovery and real replay

The earlier root REPORT/results describe the **09:14 missing-device stop**
and remain unchanged. At 10:04 the user reconnected HIKVISION successfully.
The later complete experiment is archived in
[resume_20261012_1004/REPORT.md](resume_20261012_1004/REPORT.md), with its own
actual run CSVs, frozen receipts and result JSON.

Later result: VISUAL_CAUSAL_BENEFIT_OBSERVED for primary RMSE, with explicitly
retained maximum-error regression. Full replay: 396 NDT, 396 OOSM, 169 actual
visual applications. EKF translation RMSE 1.887877 -> 1.248108 m (33.89%
lower), P95 5.011039 -> 2.552407 m, maximum 5.667157 -> 6.247122 m (10.23%
worse). No direct vision exists at 26-35s. This is one development-data
relative-drift comparison, not independent validation or deployed realtime VIO.
