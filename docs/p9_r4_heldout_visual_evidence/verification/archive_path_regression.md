# Archive invocation path regression

The first archive invocation from the repository root failed while constructing
the source SHA map, after writing derived report outputs but before saving the new
hash manifest. Scientific CSVs and immutable freeze receipts were not changed.

Command form:
`python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/archive_r4_recovered.py archive`

Error:
`ValueError: 'src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/archive_r4_recovered.py' does not start with '/tmp/dog_loc_paper_r4_ws.Fq21k2'`

Root cause: a relative `__file__` was passed to `relative_to` against an absolute
repository root. The fix resolves source paths before forming their repository
identity. The new self-test compares relative-invocation and absolute-invocation
SHA keys. Both invocation forms and the complete archive audit must pass before
commit. No scientific NDT, image extraction, source replay, or GT work was rerun.

During archival review fixes, a later audit correctly rejected an archiver source
SHA mismatch because its source file changed while that audit was still running.
This was a derived archival receipt, not a scientific input mismatch. The final
archive/audit is rerun only after code edits finish; existing scientific data and
freeze receipts stay unchanged. No alignment was repeated for these archive fixes.
