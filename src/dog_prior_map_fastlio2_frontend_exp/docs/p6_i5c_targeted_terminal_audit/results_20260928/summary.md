# PAPER-P6-I5C targeted terminal audit — formal result

## Frozen inputs and execution

- Repository: `Map-Matching.git`, branch `research/p6-i5c-stationary-audit`.
- Baseline commit: `edbe47ed68c17a5b44dc2b4d8cb0713abea8d97c`.
- Selection-freeze commit: `a9d841571661d62c7d64b2eeb4eb959a3664ed42`.
- Fix commit: `0eba42c22cb754e84a4aa759abb22b5a3bff1065`.
- The original frozen seven-frame selection was used without rerunning its selector.
  `selected_cases.csv` SHA256 is
  `94e858a39a61f4a653e6b5562e8efac706b12fe314bcdbffddd597f38cb090eb`;
  `selection_manifest.json` SHA256 is
  `ce68154111c95998b1acb2cca9c9703af18d96cd6e4ca4094c60ce6f201d5ed1`.
  Both hashes match the frozen values in every call-ledger record.
- Input integrity manifest: `input_integrity.sha256` (copied unchanged into this
  result directory). The persistent frozen map SHA256 matched the expected
  `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.
- NDT parameters remained frozen: resolution `0.8`, step size `0.08`, epsilon
  `0.001`; original replay max iterations `40`. Refinement used the frozen
  80-iteration / `1e-5` and 160-iteration / `1e-6` stages. No thresholds or
  selection rules were changed.
- Release configure/build passed. CTest: `3/3` passed. Selector tests: `8/8`
  passed. `git diff --check` passed.

## Results

### Q1 — Can the original inside/outside endpoints be replayed?

Yes. All 14 Formal endpoint replays passed; all 4 Smoke replays also passed.
Across Formal replays, translation error was exactly `0 m`, rotation error was
at most `2.04e-6 deg`, and objective difference was exactly `0` at recorded
precision. The Smoke high-jump frame `P2F087` and low-jump control `P2F003`
passed for both endpoints.

### Q2 — Do the two sides merge after further refinement?

By the frozen operational pairwise reference (`0.20 m` **and** `2 deg`), yes:
all seven selected frame pairs are classified `REFINED_COLLAPSE` after the
160-iteration stage. All 28 refinement calls (14 per stage) reported
convergence. The largest refined-160 pair gap was `0.006173 m` and
`0.284373 deg` (`P2F048`), still well inside both reference thresholds.
`P2F083` was the next-largest by rotation (`0.127038 deg`). Most other pairs
were effectively coincident at the recorded precision.

| Frame | Original Δt (m) | Original ΔR (deg) | Refined-160 Δt (m) | Refined-160 ΔR (deg) |
|---|---:|---:|---:|---:|
| P2F087 | 0.657605 | 3.622559 | 0 | 0 |
| P2F083 | 0.298012 | 0.465177 | 0.002715 | 0.127038 |
| P2F073 | 0.287267 | 1.858445 | 0 | 0.000013 |
| P2F075 | 0.030348 | 1.604438 | 0 | 0.000002 |
| P2F003 | 0.000807 | 0.006651 | 0.000000001 | 0.000014 |
| P2F048 | 0.001347 | 0.004622 | 0.006173 | 0.284373 |
| P2F006 | 0.028615 | 0.546337 | 0 | 0.0000002 |

“Collapse” here means only that the selected endpoint pair meets the
predeclared operational tolerances. It is not a proof of one mathematical
minimum or of global mode uniqueness.

### Q3 — If any refined pair remains separated, is there stationarity evidence?

No pair remains separated by the frozen operational reference, so there is no
retained distinct pair for which to claim separated-terminal stationarity.
Independently, the objective audit does **not** establish stationarity:
all 14 endpoint score triples were finite and repeatable, but all 14
analytic-versus-finite-difference gradient audits were `INDETERMINATE`, all 14
Hessian maximum-compatibility checks were false, and all 14 fixed-neighborhood
checks found a positive score-increase neighbor. The finite-difference table
contains direction-sign disagreements (21/84 at the requested step and 15/84
at half-step). These results are not mathematical local-extremum evidence.

### Q4 — Do small perturbations repeatedly return?

All 168 perturbation align calls converged. By the frozen own-terminal return
tolerances, six frames had `12/12` returns for each endpoint; `P2F048` had
`8/12` inside and `1/12` outside. Thus total own-side returns were `153/168`.
Cross-return counts were nonzero for the six operationally collapsed pairs;
they cannot be used as evidence of distinct attraction basins after those
endpoints have merged under the reference thresholds. `P2F048` had zero
cross-returns but also did not show reliable own-side return, and its refined
pair remained within the operational merge reference. No selected frame meets
the protocol’s strong criterion for two distinct, repeatably attracting
terminals.

### Q5 — Does this justify continued `U_nonlocal` mechanism research?

For this frozen targeted sample and this hypothesized persistent
inside/outside-terminal mechanism, the result does not provide a positive
reason to continue: the apparent endpoint differences collapse operationally
under the prescribed refinements, and distinct stationary terminals were not
established. This is a scoped negative result, not evidence that no nonlocal
phenomenon exists outside these seven selected frames or under other scenes,
settings, or initializations.

## Accounting and disposition

- NDT align calls: Smoke `4`; Formal `210` (`14` replay + `28` refinement +
  `168` perturbation); cumulative `214/240`, with `26` remaining.
- Sum of recorded per-align runtime fields: `19.838 s` (excludes setup, input
  loading, and non-align analysis). Smoke plus Formal command wall time was
  approximately `33 s`; build and tests are not included.
- No selection, map, bag, NDT parameter, or production algorithm was changed.
- No STOP gate was triggered: Smoke passed, Formal completed, and the align
  budget was not exceeded. The stationarity result remains inconclusive rather
  than being forced into a pass.
- Final research disposition: `REFINED_COLLAPSE` for `7/7` selected cases;
  `DISTINCT_STATIONARY_TERMINALS = NOT_ESTABLISHED`;
  no global-mode or mathematical-minimum claim.

## Artifacts

This directory contains the required CSVs: replay parity, refinement results,
terminal separation, objective stationarity, gradient finite-difference audit,
local perturbation results, per-case verdict, and call accounting. It also
contains the exact frozen selected-case CSV, selection manifest, and input
integrity SHA manifest used for the run.
