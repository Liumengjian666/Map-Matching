# Independent read-only adversarial review receipt

The user selected current independent review only; no external CLI/cross-model
review was executed. The reviewer received artifact paths and contract, without
the author's claim. It did not run the replay or access GT/raw datasets/outcomes.

## Replay/audit guard reviews (two iterations)

Required issues were corrected before final preflight and the sole replay:

- Include staged changes in the detached-worktree patch boundary and reject
  unexpected staged/untracked source.
- Reconstruct canonical six-input lineage from Git-pinned provenance rather
  than trust editable preparation path/hash pairs.
- Bind the executable argv to the exact archived command and isolated cache.
- Pin original blocker/hash manifests and source code to the start commit.
- Bind source-auditor identity and prepared values to historical references.
- Bootstrap shared Python helper integrity independently of imported require.
- Freeze the complete, fixed guard dependency set.
- Keep strict ordered audit gates; evaluate manifest parity only after160/160.
- Bind start/completion receipts and reject existing outputs even if a marker
  is accidentally missing, preventing a second scientific replay.

Final replay/audit guard review reported zero remaining Required issues.

## Archive validator review

Additional non-algorithm archive defects were corrected:

- Validate ID order/uniqueness, exact values, integer hashes, field/schema
  coverage and canonical transaction-to-raw-path linkage, not PASS text alone.
- Require complete192 raw /5 replay output hash maps and legal result/next
  flags, keeping R4_SCIENTIFIC_RESULT=NOT_RUN.
- Bind fourth-gate and TX2932 JSON values to verified CSV values.
- Convert delivery duration labels to explicit seconds without altering the
  frozen audit source or measured values. Original audit receipt/cost CSV remain.

Six in-memory mutation fixtures reject duplicate control rows, one-bit64-bit
hash changes, prepared hash mismatch, an extra manifest column, duplicate field
rows and swapped raw-provenance fields. No physical observation artifacts are
modified by these tests. No replay retry, NDT tuning or scientific rule change
was used to address review findings.

Known measurement limitation: export I/O is not separately timed by the frozen
runner; report only its conservative upper bound. P7 synthetic test alignments
are recorded separately from4127 provenance data calls.
