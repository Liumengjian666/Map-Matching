# A3G frozen full-Corridor engineering soak

START_SHA: `7bcbd5fb2c5c1822e5e0f161f5d59abc356632fd`.
Initial HEAD matched and worktree was clean. The authoritative dataset identity
is **SuperLoc Corridor01**, as pinned by the A3A/A3F manifests and file hashes;
the instruction's M3DGR label does not select different data.

The sole real process uses all 2777 catalog scans, initialization
`1517157224188979000`, 51 pre-handoff scans, and 2726 expected post-handoff
terminals. V3, ADAPTIVE_SELECTED_NIS, visual/provenance NONE, official calibration,
and both square-root QR backends are unchanged. No GT or images are read.

Only observability wiring was added. A separate lightweight marginalization
health flag records existing QR results/enforcement identities and copies a
failure capsule on failure. It does not run the legacy Schur shadow. Covariance
health records reuse the already-computed production result without requesting
the legacy covariance shadow. Optimizer trace retains the last call in memory;
success traces are not serialized. No FD/damping forensic sweep runs in soak
mode. Preopt evidence and state/prior health are flushed each event. All
estimator branches, equations, constants and parameter values remain frozen.

Synthetic diagnostics OFF/ON parity compares states, stamps, A/b, H/g caches,
optimizer/marginalization status, active factors/IDs, callback count and revision
exactly. It also tests an intentionally isolated synthetic node sequence with
a later-attempt rank failure, without changing the production failure behavior.

The launcher checks all seven frozen file hashes and matches the A3F identity
JSON, then launches one process in an exclusive new result directory. Health,
covariance, QR attempts and trajectory identities are monitored. Explicit
fail-closed records get at most 10 seconds to finish capsule persistence; a
hung process is terminated. No retry is implemented.

Before launch, parallel compilation was interrupted after a compiler process
was killed under memory pressure; serial Release and targeted Debug builds
succeeded. A launcher metadata preflight failed before directory creation or
Popen due to Git paths relative to the package; repository-root resolution was
fixed. These are build/preflight attempts, **not real estimator replays**.

Full outputs: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3g_full_corridor`.
Resource data are ENGINEERING RESOURCE OBSERVATION, not paper benchmarks.
