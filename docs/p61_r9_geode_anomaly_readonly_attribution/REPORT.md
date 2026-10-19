# P61-R9 asset gate stopped before attribution

Execution status: **ASSET_IDENTITY_CONTRACT_BLOCKED_BEFORE_ATTRIBUTION**.
Allowed result label: **MIXED_OR_INCONCLUSIVE**, solely because attribution was
not performed. This is not evidence for a mixture of H1–H5 or an algorithm failure.

## Verified identity

Repository: https://github.com/Liumengjian666/Map-Matching

Branch: `research/p61-r9-geode-anomaly-readonly-attribution`.

Independent worktree: `/home/jian/livox_ws/p61_r9_geode_anomaly_audit_ws`.

START_SHA: `eba814000a06a2b110acf3835247cd6cffe9db6b`.

The R8 remote branch was independently queried and also points to this SHA.
R9 remote branch was absent at initial query. No push was executed.
R8 actual algorithm remains17922f2b63dae1f2af34f353287728d50a312f73 and actual
replay tool1367e1d9587d1f90a47bf6f44405239a7b65e4f6; neither was executed in R9.
No R9 analysis runtime CODE commit exists; this commit contains a blocked receipt.

## Unique blocker

The task's expected original-bag SHA string is **61**, not64, hex characters:

`162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80ddf59d0b22091`

Actual fresh read-only SHA256 of
`/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag` is64 characters:

`162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091`

This actual value exactly equals the value in R8's frozen
`docs/p61_r8_geode_full_return_control/input_manifest.json`, under that original
bag path. The difference is consistent with an omitted `d80` in the task digest,
but the authoritative task digest has **not** been silently repaired.
This is not evidence that the bag changed relative to R8.

The R5 map was freshly hashed first and matches the required full digest:
`4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4`.

The initial hash check stopped at the bag assertion. The remaining manifest,
configuration and31 raw-output checks were not reached; no claim that all asset
gates passed is made. No post-analysis hash pass exists because no analysis ran.

## Scope and non-results

No NDT align, EKF run, ROS node, rosbag replay, GT access, optimizer or pose search
was executed. No HIKVISION access. No C++ or frozen asset/config was changed.
Before inspecting the hash command's failure, an already-issued source-metadata
read returned the inherited limiter/time-conversion snippets and PCD header.
It was not a numerical geometry/trajectory attribution. No fixed-window metrics,
map-density estimates, SE(3) comparisons or hypotheses were evaluated.

Consequently all anomaly, transition, serious, recovery and secondary-control
stages remain **NOT_EVALUATED**. Required CSVs contain headers only, not fabricated
zero-valued measurements. H1–H5 all remain unassessed. This includes H5: a malformed
checksum contract is not a demonstrated sensor-time or coordinate violation.

The intended scope stays exactly as requested:
5159–5229,5140–5157,5155–5165,5169–5175,5220–5240,5035–5045.
No additional interval or experiment is requested.

## Unique NEXT

Research control/user should confirm the full64-character bag SHA from the frozen
R8 manifest as the corrected R9 source identity. Only then repeat the asset gate
and continue the already-approved bounded read-only attribution in this worktree.
Do not replay localization, alter the map, tune a parameter, or enable a new module.
