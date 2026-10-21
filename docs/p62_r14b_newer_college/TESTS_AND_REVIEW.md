# Verification and bounded adversarial review

## Actual checks

`python3 -B -m unittest discover -s docs/p62_r14b_newer_college -p test_inspection.py -v`

First version:6 synthetic tests PASS. After code guard changes:7 tests PASS in
0.002s. Cases: canonical packed binary; zero and nonzero suffix visibility;
truncated bytes not passed; compressed format rejected; non-uint32 t rejected;
fixed8MiB sample read cap. These are Python input-inspector tests, not C++/PCL/NDT
integration or sensor/coordinate certification. No C++ algorithm build was attempted.

Two input-inspection attempts occurred, neither registration. Attempt1 failed at
first sample's noncanonical file length after writing production-before and scan
inventory. Preserved in inspection_failed_01. Attempt2 conservatively records
noncanonical length/null decode metrics, completes input identities and protection
snapshots. It did not silently interpret extra bytes as point data or fix the PCD.

Only3 sampled scans have fresh SHA checks, plus20 side assets. Their identities
were rechecked after inspection and in final sealing.15,301 paths/sizes and
reference timestamps were inventoried; all-scan hashes/decode were NOT performed.

## Fresh-context coordinate review

Separate local read-only reviewer received the proposed frame/time chain and
official-source files, not previous experiment outcomes. Findings were reconciled:

1. Mandatory admission issue: current GT page explicitly names RS_C1, current
   platform defines it optical, README says robotics Base. Original paper supports
   Base; it was additionally considered, but does not identify the actual CSV
   writer/version or resolve current conflicting wording. Action: do not declare
   full chain closed; preserve both interpretations, stop before NDT.
2. Current driver's supplied LUT can include LiDAR-to-sensor transformation;
   historical PCD caller/exporter not established. Valid provenance limitation,
   not proof of wrong PCD frame; do not use current driver alone to certify it.
3. Current driver t and ROS publish stamp are caller arguments; filename+t is
   not authenticated by these functions alone. Narrow time claims. This would
   block a deskew/absolute-point-time pipeline; no such pipeline was executed.
4. Registered-reference dependence and camera drift are limitations, not proof
   of wrong data. No camera offset or outcome-based alignment was applied.

The review stops at missing provenance, not a conclusion about real localization.
No repeated geometric audit of historical R9–R13 was used to choose an axis.

## Fresh-context inspection code review

Separate static reviewer used only new scripts and ASSET_MANIFEST; no network,
script execution, production writes or registration. Three actionable findings:

- Earlier6 synthetic tests used system TemporaryDirectory without dir, creating
  small transient fixtures under /tmp, automatically removed. This exceeded our
  self-imposed archive-only testing scope, not a production or dataset mutation.
  Fixed dir to the new archive; final7 tests ran there. The history is disclosed.
- Inspector silently filtered absent side assets. Required side/calibration files
  now explicitly reject absence. All actual inspected files already existed.
- Sample payload read lacked a byte cap. Added fixed8MiB guard and test. Actual
  observed samples were1,904,640 bytes; no actual resource overrun was observed.

Preservation scope is explicitly tracked/nonignored source content and recorded
Git metadata, not ignored build files, inode attributes or every .git file.
No blanket full-filesystem unchanged claim is made.

Executed inspection source is immutable in commit
168b9446e0cc81093f6af5054bc4ff5580d24960. Later hardened inspector source was only
synthetically tested, not presented as the source used for the earlier real read.
results.json records both SHA256 identities. Do not rerun it over the existing
archive: exclusive output creation intentionally refuses overwrite.

Cross-model review was offered asynchronously. No external CLI was authorized
or executed for R14B. The single old R14 CLI permission was not reused.

## Reproduction without localization

- Validate stored hashes: from this archive, `sha256sum -c checksums.sha256`.
- Inspect fresh snapshot files and explicit source/asset identities directly.
- Synthetic guard tests use the unittest command above.
- Do not execute fetch_sources.py or inspect_inputs.py against their existing
  exclusive output directory. It preserves the one bounded inspection attempt
  and its failure/recovery provenance; reproduction needs a separate reviewed
  output archive rather than deletion/overwriting of these receipts.
- Header-only experiment CSVs mean NOT_RUN, never a successful empty pilot.
