# P62-R14B — Newer College prior-map benchmark admission

## Outcome

**COORDINATE_CONTRACT_BLOCKED**. OnlyNewer College2020 /01_short_experiment was
inspected. The original Wheel-float01 route was suspended, not resumed. The full
R14B contract was received during this turn and is digested in CONTRACT_DIGEST.md.
The user explicitly requires stopping before NDT if the frame chain is unclosed.

## Git and implementation

Repository:https://github.com/Liumengjian666/Map-Matching

Branch:`research/p62-r14b-newer-college-prior-map`.
Workspace:`/home/jian/livox_ws/p62_r14b_newer_college_ws`.
Start:`a611f385025788481a747b3649ea4a90f3316540`.
Executed input-inspection source archived at
`168b9446e0cc81093f6af5054bc4ff5580d24960`; its later guard hardening is separate,
not retroactively claimed to have governed the earlier read. Actual file hashes
are in results.json. Final commit is obtained from `git rev-parse HEAD`, avoiding
a self-referential commit field. No push was executed.

No research/production NDT algorithm was changed, compiled or run. Added scripts
only inspect inputs, capture fixed official sources, test parser guards and seal
the blocked result. Existing R6/R9/R10/R11/R12/R13 headers were read to identify
reuse interfaces, not run as historical experiments. No old preflight/access
program, Git preservation operation, or asset-drift closure was rerun.

## Real assets and formats

Local PCDFiles:15,301; bytes29,142,896,640. Reference rows:15,301. Exact integer
filename/reference pairs:15,301.20 side assets and3 chronological PCD samples
have recorded hashes and before/after checks.19 expected side-asset identities
matched the retained local download manifest; one historical converted PCD lacks
such an expected entry. This is not publisher-hash authentication or whole-PCD
integrity certification. No new large sensor payload was downloaded.

Leica5cm PLY SHA:
`e8bf6c7f2ddacde4f6f65ba7759b00bdca5c6bf5e706344ce8c3592529f04ab0`.
Reference CSV SHA:
`b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e`.
Full paths/sizes/calibration identities: DATASET_IDENTITY.md and ASSET_MANIFEST.json.

SOCC-ICP pinned source commit:
`557a98e39a163f0b369a1612db7fc69f8b5aae60`. Its actual loader accepts this PCD/CSV
layout, but applies an approximate extrinsic then first-pose normalization. It
does not certify absolute Leica localization coordinates. The reference is itself
Ouster-to-Leica ICP-registered Benchmark Reference, not independent physical mocap.

## Sole primary stop gate

The current official GT page calls stereo poses RS_C1; the platform page calls
RS_C1 optical and distinguishes robotics Base. Official robot README and original
paper support Base, but no release/file-specific writer or authoritative
clarification resolves which quaternion axes this actual CSV serializes.
Both possible calibrated chains are documented in MAP_AND_FRAME_CONTRACT.md.
No geometry-based axis choice, test-pose alignment, or inferred identity extrinsic
was used. This is missing provenance, not proof that either interpretation or
Newer College data is wrong. Do not bypass this gate with SOCC's approximate transform.

## Time, map, prediction, experiment

Integer epoch pairing is observed. It does not prove absolute point-time origin,
scan-end semantics or physical synchronization; the actual clouds are uncorrected.
No guessed camera time offset was applied. TIME_ALIGNMENT_CONTRACT.md bounds
these observations without calling the unfinished pilot time gatePASS.

No map/ROI was created. No21-frame pilot set or reference anchor was selected.
No causal predictor, native NDT or recovery was executed. Test counts/metrics
are null/NOT_RUN, not successful empty outputs. Header-only result CSVs preserve
all unexecuted stages. There are no alignment/error/event numbers to report.

## Checks and preservation

Initial inspection failure is retained under inspection_failed_01. A real sample
matches its official ZIP entry but has3846 zero trailing bytes; no source edited.
Seven synthetic guard tests pass; no PCL compatibility claim. Stable production
HEAD remains`41999ea700c66c4cadf0eca9e0c5d73caa2783fd`; scoped before/after contents,
index, refs, reflog and status match. Final protection recheck is in
PRODUCTION_FINAL.json. Scope excludes ignored files and unrecorded metadata.

Git housekeeping encountered a missing local author setting; the successful
research commit used the verified existing author's identity per command, without
changing global/stable config. Raw CSV CRLF and official source whitespace were
retained; raw-file diff whitespace warnings are not hidden by rewriting evidence.

The first6 synthetic tests created auto-cleaned fixtures in `/tmp`, exceeding
the self-imposed archive-only test-write scope. This was not a production/data
mutation; final7 tests use the new archive. Full disclosure and review findings
are in TESTS_AND_REVIEW.md. No additional substantive issue remained in the
second bounded static code-review pass.

## Decision and NEXT

H1BLOCKED,H2not established,H3–H7not run. NoveltyNOT_ESTABLISHED. No claim about
5deg correctness, NDT precision, successful recovery,20-frame efficacy or10Hz.

**Only NEXT:** provide authoritative release/file-specific reference-frame evidence
resolving opticalRS_C1 versus roboticsBase, then review the unchanged R14B admission
contract. NoNDT, alternate dataset, or automatic next task until it closes.
