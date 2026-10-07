# Objective sign correction (offline analysis only)

The pre-run `THEORY.md` and its execution-manifest SHA256 are immutable provenance snapshots. Two raw-score sign sentences in that snapshot are wrong: the exact PCL score returned by `ExactPclNdt::dynamicValueOnly()` is **maximized**, not minimized. This addendum supersedes only those sentences and the corresponding initial analysis implementation.

The frozen implementation in `p9_ndt_energy_contract.cpp` explicitly defines `evaluate().energy = -result.score`. Accordingly, throughout the corrected objective-selection diagnostics:

```
S = raw_ndt_score_sum
E = -S
argmin E = argmax S
```

At each budget, choose the highest score among nominal T0 and the finite candidate returns. Nominal wins exact ties; candidate ties select the lower probe rank. CSV exports retain the raw score and add explicit negative-energy fields. No probabilistic interpretation is permitted.

An independent read-only review found the wrong-sign `min(S)` implementation. A new regression test with candidate scores10 and20 and nominal score15 fails with the original implementation and passes only when selecting score20 (energy-20). Additional tests require preserving nominal on ties and when all candidates have worse energy.

The earlier preliminary GT claim (all9 major and all23 controls worsened under WEAK B12) is invalid and withdrawn because it selected the wrong score extremum. Corrected choices are frozen before repeating post-hoc GT scoring with the unchanged archived reference transform. The old invalid pre-GT choice table had SHA256 `ad52b86fc3b203d05560e4e1ea7d91133601b89ae207296a6794f2443d3b8acc`; it is not a final scientific result.

This correction changes only offline objective choice, associated no-major diagnostics, and post-hoc GT summaries. It does not alter any proposal, random seed, selected probe order, NDT call, terminal, convergence flag, oracle label, admission assignment, recall, AUC, or discovery gate. The2560 NDT calls are not repeated. `execution_manifest.json` and its frozen theory/source/proposal/result hashes are retained exactly.
