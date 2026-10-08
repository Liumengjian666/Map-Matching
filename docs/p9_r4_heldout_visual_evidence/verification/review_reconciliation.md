# Resume review and reconciliation

Two fresh-context single-model read-only reviewers examined the recovered-source
input guard and the blind evidence/evaluator stage boundary before alignment.
Findings fixed before their reviewed logic stood:

- Continuously hash all192 closure exports, not only160 held-out raw sources.
- Skip archived oracle receipt content in blind-builder verification.
- Own oracle worker processes and terminate siblings promptly on a worker failure.
- Require count-only ID/hash authorization before B12, and a separate label-free
  candidate-PASS authorization before visual extraction.
- Never overwrite the earlier blind LiDAR freeze while producing later evidence.
- Bind downstream evaluation to cohort/candidate freeze hashes and output hashes.

A final independent reviewer recomputed the actual candidate diagnostics and
checked frozen proposals, complete-link clusters, competitive-score and strict
center contracts, and source parity. Recomputed:96 frames=40 MAJOR+56 NO_MAJOR;
84 major IDs,39 recovered; macro0.51625; micro39/84; strict coverage17/40.
Eight multiple-ID admission events use the unchanged historical single-assignment
rule.50 converged iteration-limit candidate rows are retained.

The new archival code review found two additional actionable error paths:

1. Unconditionally clearing downstream CSVs could conceal an already executed
   visual/GT/statistical stage. The archiver now checks every skipped CSV's exact
   header-only content before any write, rejects downstream execution receipts or
   unexpected files, and preserves originals on failure. Actual computed visual
   coverage is also refused, not converted to a NOT_RUN status.
2. Hash checking only listed keys could miss new files. Audit now requires the
   complete actual file set to equal the artifact manifest key set, excluding only
   the root manifest itself, before checking each byte hash.

Regression fixtures test nonempty CSV preservation, unexpected primary receipts,
computed coverage preservation, and an unlisted file. These are synthetic file
fixtures, not new scientific measurements. Relative/absolute invocation regression
is documented separately. Both actual scoped audits and the40-test P9 suite must
pass before commit.

No cross-model CLI was invoked. Additional external review was offered, without
assuming authorization from previous tasks. Review fixes affect contract checks
and archival safety, not frozen research parameters or scientific rows.
