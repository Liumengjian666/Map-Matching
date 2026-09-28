# P6-I5C sample-selection protocol (frozen before new NDT calls)

## Scope

The selector chooses a small, targeted diagnostic set from P6-I5A's finite
DENSE boundary endpoints. It does not read GT, run NDT, or classify a
mechanism. The score `q` below is only a sampling rank; it is not a basin
margin or a basin-hopping classifier.

## Eligibility

A row is eligible only if all conditions hold:

1. `boundary_phenotype.csv.margin_source == DENSE`;
2. the DENSE boundary is finite (`alpha_diff` present and terminal geometry
   available), and the matching I4 DENSE summary says it is not censored;
3. the boundary phenotype and I4 DENSE summary agree on frame, ray type,
   ray id, sign, and quantized inside/outside bracket endpoints;
4. the corresponding I5B support row is `STRICT_LOCAL_SUPPORT`.

Contaminated frames are excluded from every group. No eligibility threshold is
relaxed to fill a group.

## Ranking and groups

For each eligible frame:

`q = max(terminal_jump_translation_m / 0.20,
         terminal_jump_rotation_deg / 2.0)`.

- **A — high jump:** up to four distinct frames, descending `q`.
- **B — low jump control:** up to two remaining distinct frames, ascending
  `q`.
- **C — first-exit mismatch:** up to one remaining eligible frame with the
  largest count of unique strict-support signed first-exit mismatch rays.

All ties are resolved by ascending integer `transaction_id`. For C, only the
two `EXIT_PREDICTED_*_ACTUAL_*` mismatch case types in I5B's
`exception_frames.csv` count; primary acceptance disagreements, rank-only
outliers, non-strict rows, and duplicate ray keys do not count. Groups do not
overlap. If fewer qualifying frames exist, the group remains short.

## Endpoint lineage

Inside and outside endpoints are joined to the frozen I4
`ray_probe_results.csv` by exactly:

`(transaction_id, ray_type, ray_id, sign, alpha_key)`

where `alpha_key = floor(alpha * 1e6 + 0.5)`, matching I5A's positive-alpha
`llround(alpha*1e6)` convention. No timestamp or nearest-alpha matching is
allowed. Any missing key, conflicting duplicate key, non-frozen endpoint
source, convergence disagreement, or terminal-pose mismatch blocks selection.
Pose equality is checked at `1e-9 m` translation and `1e-8 deg` quaternion-
sign-invariant SO(3) angle; output pose components are copied as their original
CSV number tokens.

## Frozen inputs and order

The script hashes the I4 raw probe and DENSE summary, I4 preparation manifest,
I5A boundary phenotype, I5B support and exception tables, P6 scan metadata,
packed source-cloud bytes, and both prepared/persistent frozen map copies.
Frozen asset SHA-256 values are checked against the P6-I4/P6-I3 expected
values. The generated `selected_cases.csv`, `selection_manifest.json`, and
`input_integrity.sha256` are read back before the selector reports success.
The output guard refuses to overwrite an existing selection.

The selection artifacts and this rule are committed before any added NDT
alignment. After that commit, sample membership must not be changed in response
to experimental results.
