# Bounded independent review

One fresh-context read-only pass covered the finite analysis and optimizer
pairing. It ran no ROS, native align, parameter search or external CLI.

Required finding: effect/time-line translations mixed LiDAR and IMU origins.
Classified actionable. Derived initial/raw poses now use the same frozen IMU
extrinsic as GT errors; LiDAR displacement is retained only in a named auxiliary
column. No capture/native output was modified. A .1m lever arm and90deg rotation
regression proves coincident LiDAR origins need not mean coincident IMU origins.

Recomputed corrected values: frame7 source effect .006110443246m;
frame365 initialization effect on SA 1.033236151431m, observed AA→BB
1.102985024395m, interaction .239025824974m. Tests passed without new align.

Reviewer independently recomputed formal EKF RMSE A1.976924725831,
B3.104707802052m and RPE RMSE approximately .456187936/.893484529m.
The nominal1s RPE actually spans1.004863977–1.004960060s; this is explicitly
reported, not hidden or retuned.

The first real MSCKF release is1517157219.809216, not merely its earlier
measurement time1517157219.654659032. A later within-frame7 image at
1517157219.858260648 releases1517157220.0091519 and cannot explain frame7.
Actual fixed-lag release rows are now associated by their integer observation
IDs; within-scan image times and already-released updates have separate columns.

Cross-model/external CLI: the user requested it. A proposed exact read-only
Codex CLI command and immutable limited input (SHA256
e8cba347a7752077875162871e7f691aa3c4b5f55c42cd3d65dc9bc6ab9ff22f)
were provided for confirmation. The CLI is NOT run without that confirmation.
Final status/evidence, if executed, is added separately; no success is assumed.

Final external status: user confirmed that exact command and input. One Codex
CLI0.159.2 invocation completed with exit0, model gpt-6.1-sol, xhigh, read-only,
ephemeral, no user/project config. Verdict: `No substantive issue.` The model
catalog refresh initially timed out but final review returned normally. This
is an independent external session, not a claim of a different architecture.
Exact authorized command, immutable input and result are in `external_review/`.
