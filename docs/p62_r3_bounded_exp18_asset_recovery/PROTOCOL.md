# P62-R3 — bounded fixed-revision asset recovery

This is file acquisition only. No ROS replay, bag traversal, E57 decode, localization,
NDT/EKF, trajectory generation, GT-based evaluation, external CLI, or dependency install.
No HIKVISION access. No push.

Baseline: `d383bfed52db638f54a9b208c1ec27f663d63eae`.
R2 local and queried remote branch both equal this SHA; R2 worktree is clean.
The new sparse R3 worktree avoids duplicating historical tracked research artifacts.
An initial Git 2.25 checkout/sparse interaction temporarily materialized baseline
copies and reduced free space below the startup gate. No transfer occurred then.
Only the new checkout was sparsified; historical assets/worktrees were not pruned.

Revision: `e62017f907007fdc5ab8c721842e4ae7359d7f49`.
Pinned bag size/SHA: 8,657,654,860 bytes /
`98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280`.
Pinned E57 size/SHA: 5,447,128,064 bytes /
`46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756`.

One active HTTPS request; 64 MiB buffered segments; exact 206/range/length checks;
90 s socket timeout; at most two attempts per segment. A failed or rejected segment
is never appended. Prefix probe is one 1 MiB request; compare byte-for-byte before
copying the complete 452,984,832-byte old linear partial to a new staging file.
The old sparse ranged partial is not a usable prefix. Preserve all five old partials.
Hash all old partials before/after; compare both bag partials with R2 frozen identities.

Start gate: free >= 48,000,000,000 bytes. Every local write retains >=20 GiB,
plus allocation rounding and 1 MiB receipt reserve. No conversion workspace is used.
Task start was first checked at 2026-10-19 09:04 UTC. Conservative hard stop:
2026-10-19 12:00 UTC (20:00 Asia/Shanghai), additionally capped at process-start+180 min.
Stop transfers 60 s earlier for final snapshots. Do not restart this one-shot program.

Only a complete size/SHA/magic-verified file may be hardlinked to its new verified
path, with post-publication SHA and inode checks. Bag must pass before E57 starts.
No old file is overwritten; existing one-shot output prevents rerun.

Test command:
`python3 -m unittest discover -s docs/p62_r3_bounded_exp18_asset_recovery/tests -v`

Authorized execution command (once, after tests/review):
`python3 -u docs/p62_r3_bounded_exp18_asset_recovery/recover.py --run`

The final report must distinguish per-response validity, committed prefix, full-file
identity, and safe publication. Failures, timeouts and retry counts remain archived.
