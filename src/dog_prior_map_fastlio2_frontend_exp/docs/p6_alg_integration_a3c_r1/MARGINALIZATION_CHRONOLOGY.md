# Repeated marginalization chronology

`MARGINALIZATION_HISTORY.csv` contains every `marginalizeOldest()` attempt
captured during the single P3-100 run, including the failing attempt. It has
39 rows: 38 successes and one failure.

The first successful elimination in this captured run was transaction 71,
event stamp `1517157226248733355`, enforcement index 40, attempt 1. It removed
the initialization state at `1517157224188979000` from a 41-node,
`2.059754355 s` window.

Transactions 71–89 each had two successful oldest-state removals during their
enforcement call. Thus, before tx90 there were 19 successful enforcement
episodes and 38 successful elimination attempts. The enforcement indices in
the trace are 40, 42, …, 76; other event-level calls did not trigger an
elimination. Every recorded attempt was triggered by duration only; the node
limit was false.

Tx90 entered enforcement index 78 with 41 nodes and span `2.017077923 s`.
Its first and only attempt failed. Therefore:

```text
TX90_FAILED_ATTEMPT_INDEX_WITHIN_ENFORCEMENT = 1
SUCCESSFUL_OLDEST_REMOVALS_BEFORE_TX90_FAILURE_IN_SAME_EVENT = 0
trigger_duration_limit = true
trigger_node_limit = false
```

## Prior evolution

Across all 38 successful eliminations:

- incoming prior and new prior `lambda_min`: exactly `0` in the recorded
  eigensolver output;
- incoming/new prior relative-negative ratio: exactly `0`;
- incoming prior spectral scale ranged from `9999.931538` to
  `20952.178439`;
- new prior spectral scale ranged from `9999.931538` to `20991.159275`;
- no solve-only jitter was used;
- `H_mm` primary numerical rank was 15/15 in every attempt;
- all recorded LDLT solve pivots were positive; no negative or near-zero
  pivots were recorded.

Conditioning did vary: `H_mm` condition proxy ranged from about `131.76` to
`1.6044e9`, and the LDLT min/max absolute pivot ratio ranged from
`6.2486e-10` to `7.5894e-3`. The most severe relative pivot ratios occurred in
earlier successful eliminations, not tx90. At tx90, the proxy was `131.76`,
the pivot ratio was `7.5894e-3`, and the absolute pivot gate was not close to
triggering.

This history does not show progressively accumulated negative prior modes.
It does show earlier scale disparity, which is retained as an engineering
observation rather than assigned as tx90's immediate cause.
