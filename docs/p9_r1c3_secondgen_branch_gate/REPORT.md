# R1C3 mechanism gate

FINAL_RESULT = SUPPORT_BRANCH_NUMERICS_REMAIN_UNCLOSED

No online efficiency or strict stationary-oracle claim. Double reference never feeds search state.

Eight logical cases share six independent nominal roots. Uncertified roots or
failed root closure stop paths. The predeclared root-closure check is distinct
from the numerical branch certificate; it was not relaxed after observing data.

## Decision and limits

The primary mechanism gate was **not evaluated to its target endpoints**:
2226/P05 cannot start after an independent FLOAT directional FD failure;
2350/P01 and P05 have complete certified roots but fail the additional inherited
T0 closure check. Therefore primary 0/3 and endpoint 0/8 do not refute the branch
hypothesis. This result neither establishes nor disproves the desired mechanism.
It does not modify H1, H2, the DUAL-U research direction, or the oracle.

Complete root certificates: 3/8 logical cases, 2/6 independent frames.
Complete certificates **and** closure: 1/8 cases, 1/6 frames. Root closure fails
in 5/8 cases / 3/6 frames; of these, 2/8 cases / 1/6 frames are nevertheless
numerically certified. Independent FD failures occur in 3/8 cases / 3/6 frames.
Support is unresolved in 2/8 cases / 1/6 frames (3341). All eight roots have SPD
Hvv. No root has an unexecuted independent FD audit at its terminal check.

The original `max_alpha` in per_basin_summary.csv/results.json means the maximum
**attempted** node, not an accepted or certified continuation endpoint.
For 616/P10 it is .725; the maximum **accepted** alpha is .68125. Nodes at .725
are uncertified SUPPORT_FIXED_POINT_NOT_CLOSED attempts. No alpha=1 node exists.

## Archived successful-seed audit (no new NDT)

All successful seeds below are H2's converged, uniquely assigned FULL6D results;
the matched WEAK2 seed-index recovery count is zero for all six missed targets.
This is descriptive mechanism evidence, not a causal or independent-sample test.

| Target | FULL successes | median ‖u‖ | median ‖v‖ | median strong fraction | matched WEAK recovery |
|---|---:|---:|---:|---:|---:|
| 2226/P05 | 13 | 1.067022 | .403392 | .275939 | 0 |
| 2350/P01 | 10 | 1.106318 | .442280 | .371677 | 0 |
| 2350/P05 | 7 | 2.079081 | .625016 | .290925 | 0 |
| 3341/P02 | 10 | .315675 | .810320 | .912503 | 0 |
| 3341/P03 | 8 | .428792 | 1.161937 | .912083 | 0 |
| 2722/P05 | 11 | .748013 | 1.932628 | .932585 | 0 |

The successful-control 616/P10 also has zero *same-index* recovery, although
other WEAK2 indices recover it; it is not mislabeled as an H2 miss. 3796/P06
has six same-index recoveries among its eight FULL successes.

## Per-case progression

Certified candidate count includes accepted and rejected/retry-discarded
candidates, counted once per candidate ID. Accepted node count includes alpha0.
Every target has archive-ID recovery=NO, strict certified target recovery=NO,
and final full-NDT diagnostic=NOT_RUN (no certified alpha1 input).

| Target | rho W2 | ‖u_b‖ | root certificate / closure | candidates / certified | accepted nodes | max attempted / accepted alpha | support updates | splits / merges | max active |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|
| 2226/P05 | .850719 | .150145 | NO / YES | 1 / 0 | 0 | 0 / none | 3 | 0 / 0 | 0 |
| 2350/P01 | .864281 | 2.365730 | YES / NO | 1 / 1 | 0 | 0 / none | 2 | 0 / 0 | 0 |
| 2350/P05 | .855949 | 2.349952 | YES / NO | 1 / 1 | 0 | 0 / none | 2 | 0 / 0 | 0 |
| 3341/P02 | .494453 | .085666 | NO / NO | 1 / 0 | 0 | 0 / none | 8 | 0 / 0 | 0 |
| 3341/P03 | .562643 | .125175 | NO / NO | 1 / 0 | 0 | 0 / none | 8 | 0 / 0 | 0 |
| 2722/P05 | .193958 | .037356 | NO / YES | 1 / 0 | 0 | 0 / none | 4 | 0 / 0 | 0 |
| 616/P10 | .972002 | .654313 | YES / YES | 45 / 30 | 16 | .725 / .68125 | 202 | 0 / 14 | 1 |
| 3796/P06 | .999311 | 3.692702 | NO / NO | 1 / 0 | 0 | 0 / none | 7 | 0 / 0 | 0 |

## Specific numerical evidence

- 2226/P05: exact support equality, numerical-resolution PASS, Hvv SPD, but
  FLOAT directional gradient error=1.61543159e-4 and relative error=.09802765;
  neither unchanged FD criterion (1e-4 absolute OR .02 relative) passes.
  dN=6.46746547e-6, EPS_DV=1.44617304e-4. Certification is not inferred from
  this small Newton displacement alone.
- 2350 shared root: support, FD, SPD, numerical resolution all PASS.
  root-to-T0 diagnostic translation is about .00187244m and rotation .224859deg,
  beyond the predeclared .2deg closure cap. These are two logical paths from
  one root, not two independent failures. No oracle-v or closure-gate relaxation.
- 3341 shared root: final frozen certificate components pass, but dynamic
  support is different after all8 rounds. Eight exact-signature updates, no
  cycle, per case. The generic ROOT_CLOSURE_FAIL status also masks the underlying
  support failure; use the separate support_equal field to identify it.
- 2722/P05: final FLOAT direction2 gradient error=1.52544092e-4,
  relative=.03217015; support, resolution and SPD pass, FD does not.
- 3796/P06: support and SPD pass, but resolution and independent FLOAT FD fail.
  Worst final directional gradient disagreement=1.13667804e-3, relative=.3568553.
  dN=5.28175285e-6, EPS_DV=5.79965494e-5; the conjunction still fails and is not
  replaced by a raw-gradient or displacement-only gate.
- 616/P10: root dN=9.64788890e-7 vs EPS_DV=2.41544659e-5, complete PASS.
  Fixed-support forward progression reaches accepted alpha=.68125. The last
  attempt uses delta_alpha=.005, target=.68625; both old/new hypotheses fail
  exact support closure after8 rounds despite resolution and FD passes.

## Support spawning and costs

22 old/new hypothesis comparisons were executed, all with identical predictor
states. They produced **zero certified distinct splits**, 14 merges and maximum
one active branch. There are 236 changed-support outer-round records including
root correction; these are not 236 branch events or causal support certificates.
No support cycle was detected in 52 corrector calls.

FLOAT frozen evaluations=285326; DOUBLE diagnostic evaluations=755682;
dynamic-support evaluations=287; guard comparisons=6264. Sum of measured case
wall times=43.610481581s (not total process wall time or an online cost claim).
Full NDT calls inside continuation=0; final diagnostic calls=0. GT_USED=NO;
oracle-v/support sequence injected=NO. Initial priors, map, source and NDT
settings were not changed; no H2 rerun or new heuristic search was performed.

Release build PASS; P9 CTest 13/13 PASS (including R1C2 numeric tests and
continuation/pruning self-tests); CSV/JSON audit and input/source/
binary hashes PASS. New diagnostic headers do not alter historical R1C2 or
production localization code. Historical H2 CMake provenance is verified via
START content, separately from the authorized current build recipe.

Archival formatting only: the empty `final_branch_evaluation.csv` was normalized
from the CSV writer's default CRLF to LF after generation for `git diff --check`.
No cells, scientific results or frozen execution sources were changed. Its
published sidecar SHA and the report SHA were resealed; the independent CSV/JSON
value and execution-source audit is run after this mechanical formatting step.

NEXT = CLOSE_EXISTING_BRANCH_NUMERICAL_CONTRACT_WITHOUT_NEW_SEARCH.
Do not advance to non-oracle discovery or covariance fusion on this run.

## Machine-derived summary (original experiment fields)

```json
{
  "current_run_state": "COMPLETE",
  "final_result": "SUPPORT_BRANCH_NUMERICS_REMAIN_UNCLOSED",
  "next": "CLOSE_EXISTING_BRANCH_NUMERICAL_CONTRACT_WITHOUT_NEW_SEARCH",
  "primary_recovered": 0,
  "certified_endpoint_cases": 0,
  "roots": {
    "attempted": 8,
    "certified": 3,
    "independent_attempted": 6,
    "independent_certified": 2,
    "support_unresolved": 2,
    "FD_invalid": 3,
    "FD_not_validated": 0
  },
  "per_basin": [
    {
      "tx": 2226,
      "cluster": "P05",
      "rho_W2": 0.8507194141678078,
      "u_b_norm": 0.1501448373071985,
      "forward_candidates": 1,
      "certified_candidates": 0,
      "accepted_nodes": 0,
      "max_alpha": 0.0,
      "support_events": 3,
      "branch_splits": 0,
      "branch_merges": 0,
      "max_active": 0,
      "certified_to_alpha1": false,
      "archive_id_recovered": false,
      "strict_branch_recovered": false,
      "full_ndt_escape": false
    },
    {
      "tx": 2350,
      "cluster": "P01",
      "rho_W2": 0.8642806824177345,
      "u_b_norm": 2.365729579894445,
      "forward_candidates": 1,
      "certified_candidates": 1,
      "accepted_nodes": 0,
      "max_alpha": 0.0,
      "support_events": 2,
      "branch_splits": 0,
      "branch_merges": 0,
      "max_active": 0,
      "certified_to_alpha1": false,
      "archive_id_recovered": false,
      "strict_branch_recovered": false,
      "full_ndt_escape": false
    },
    {
      "tx": 2350,
      "cluster": "P05",
      "rho_W2": 0.8559486460525296,
      "u_b_norm": 2.349952015553473,
      "forward_candidates": 1,
      "certified_candidates": 1,
      "accepted_nodes": 0,
      "max_alpha": 0.0,
      "support_events": 2,
      "branch_splits": 0,
      "branch_merges": 0,
      "max_active": 0,
      "certified_to_alpha1": false,
      "archive_id_recovered": false,
      "strict_branch_recovered": false,
      "full_ndt_escape": false
    },
    {
      "tx": 3341,
      "cluster": "P02",
      "rho_W2": 0.4944527511024945,
      "u_b_norm": 0.0856660867259351,
      "forward_candidates": 1,
      "certified_candidates": 0,
      "accepted_nodes": 0,
      "max_alpha": 0.0,
      "support_events": 8,
      "branch_splits": 0,
      "branch_merges": 0,
      "max_active": 0,
      "certified_to_alpha1": false,
      "archive_id_recovered": false,
      "strict_branch_recovered": false,
      "full_ndt_escape": false
    },
    {
      "tx": 3341,
      "cluster": "P03",
      "rho_W2": 0.5626434931344211,
      "u_b_norm": 0.12517541064224527,
      "forward_candidates": 1,
      "certified_candidates": 0,
      "accepted_nodes": 0,
      "max_alpha": 0.0,
      "support_events": 8,
      "branch_splits": 0,
      "branch_merges": 0,
      "max_active": 0,
      "certified_to_alpha1": false,
      "archive_id_recovered": false,
      "strict_branch_recovered": false,
      "full_ndt_escape": false
    },
    {
      "tx": 2722,
      "cluster": "P05",
      "rho_W2": 0.1939581244617182,
      "u_b_norm": 0.03735573425874087,
      "forward_candidates": 1,
      "certified_candidates": 0,
      "accepted_nodes": 0,
      "max_alpha": 0.0,
      "support_events": 4,
      "branch_splits": 0,
      "branch_merges": 0,
      "max_active": 0,
      "certified_to_alpha1": false,
      "archive_id_recovered": false,
      "strict_branch_recovered": false,
      "full_ndt_escape": false
    },
    {
      "tx": 616,
      "cluster": "P10",
      "rho_W2": 0.9720018682350564,
      "u_b_norm": 0.6543133095488903,
      "forward_candidates": 45,
      "certified_candidates": 30,
      "accepted_nodes": 16,
      "max_alpha": 0.7250000000000001,
      "support_events": 202,
      "branch_splits": 0,
      "branch_merges": 14,
      "max_active": 1,
      "certified_to_alpha1": false,
      "archive_id_recovered": false,
      "strict_branch_recovered": false,
      "full_ndt_escape": false
    },
    {
      "tx": 3796,
      "cluster": "P06",
      "rho_W2": 0.999311458735581,
      "u_b_norm": 3.692701821194768,
      "forward_candidates": 1,
      "certified_candidates": 0,
      "accepted_nodes": 0,
      "max_alpha": 0.0,
      "support_events": 7,
      "branch_splits": 0,
      "branch_merges": 0,
      "max_active": 0,
      "certified_to_alpha1": false,
      "archive_id_recovered": false,
      "strict_branch_recovered": false,
      "full_ndt_escape": false
    }
  ],
  "costs": [
    {
      "tx": "2226",
      "cluster": "P05",
      "frozen_evals": "3909",
      "double_evals": "7299",
      "dynamic_evals": "6",
      "guard_evals": "63",
      "corrector_calls": "1",
      "support_cycles": "0",
      "runtime_ms": "509.99648300000001",
      "full_ndt_calls": "0"
    },
    {
      "tx": "2350",
      "cluster": "P01",
      "frozen_evals": "2821",
      "double_evals": "6149",
      "dynamic_evals": "5",
      "guard_evals": "54",
      "corrector_calls": "1",
      "support_cycles": "0",
      "runtime_ms": "313.10428400000001",
      "full_ndt_calls": "0"
    },
    {
      "tx": "2350",
      "cluster": "P05",
      "frozen_evals": "2821",
      "double_evals": "6149",
      "dynamic_evals": "5",
      "guard_evals": "54",
      "corrector_calls": "1",
      "support_cycles": "0",
      "runtime_ms": "311.24488400000001",
      "full_ndt_calls": "0"
    },
    {
      "tx": "3341",
      "cluster": "P02",
      "frozen_evals": "6676",
      "double_evals": "18415",
      "dynamic_evals": "10",
      "guard_evals": "153",
      "corrector_calls": "1",
      "support_cycles": "0",
      "runtime_ms": "876.13147200000003",
      "full_ndt_calls": "0"
    },
    {
      "tx": "3341",
      "cluster": "P03",
      "frozen_evals": "6676",
      "double_evals": "18415",
      "dynamic_evals": "10",
      "guard_evals": "153",
      "corrector_calls": "1",
      "support_cycles": "0",
      "runtime_ms": "875.88611800000001",
      "full_ndt_calls": "0"
    },
    {
      "tx": "2722",
      "cluster": "P05",
      "frozen_evals": "3927",
      "double_evals": "12751",
      "dynamic_evals": "7",
      "guard_evals": "90",
      "corrector_calls": "1",
      "support_cycles": "0",
      "runtime_ms": "1804.1762490000001",
      "full_ndt_calls": "0"
    },
    {
      "tx": "616",
      "cluster": "P10",
      "frozen_evals": "191617",
      "double_evals": "526388",
      "dynamic_evals": "234",
      "guard_evals": "4203",
      "corrector_calls": "45",
      "support_cycles": "0",
      "runtime_ms": "21671.115528999999",
      "full_ndt_calls": "0"
    },
    {
      "tx": "3796",
      "cluster": "P06",
      "frozen_evals": "66879",
      "double_evals": "160116",
      "dynamic_evals": "10",
      "guard_evals": "1494",
      "corrector_calls": "1",
      "support_cycles": "0",
      "runtime_ms": "17248.826561999998",
      "full_ndt_calls": "0"
    }
  ],
  "success_seed_audit": [
    {
      "tx": "2226",
      "cluster": "P05",
      "full_successful_seeds": "13",
      "median_u": "1.0670216371513597",
      "median_v": "0.40339188427020245",
      "median_strong_fraction": "0.27593944630065886",
      "same_index_weak_recovery": "0"
    },
    {
      "tx": "2350",
      "cluster": "P01",
      "full_successful_seeds": "10",
      "median_u": "1.1063182937166185",
      "median_v": "0.44227988835946824",
      "median_strong_fraction": "0.37167689133702414",
      "same_index_weak_recovery": "0"
    },
    {
      "tx": "2350",
      "cluster": "P05",
      "full_successful_seeds": "7",
      "median_u": "2.079081452793952",
      "median_v": "0.6250155738124822",
      "median_strong_fraction": "0.2909252672760135",
      "same_index_weak_recovery": "0"
    },
    {
      "tx": "3341",
      "cluster": "P02",
      "full_successful_seeds": "10",
      "median_u": "0.31567497788594157",
      "median_v": "0.8103200497399211",
      "median_strong_fraction": "0.9125032197745317",
      "same_index_weak_recovery": "0"
    },
    {
      "tx": "3341",
      "cluster": "P03",
      "full_successful_seeds": "8",
      "median_u": "0.4287923475966777",
      "median_v": "1.1619368686536786",
      "median_strong_fraction": "0.9120825763965408",
      "same_index_weak_recovery": "0"
    },
    {
      "tx": "2722",
      "cluster": "P05",
      "full_successful_seeds": "11",
      "median_u": "0.7480129071800112",
      "median_v": "1.9326275844466307",
      "median_strong_fraction": "0.9325845042251033",
      "same_index_weak_recovery": "0"
    },
    {
      "tx": "616",
      "cluster": "P10",
      "full_successful_seeds": "8",
      "median_u": "0.26614758630300056",
      "median_v": "1.0716536561302377",
      "median_strong_fraction": "0.9634769210424794",
      "same_index_weak_recovery": "0"
    },
    {
      "tx": "3796",
      "cluster": "P06",
      "full_successful_seeds": "8",
      "median_u": "2.6000522505907435",
      "median_v": "0.879461601932254",
      "median_strong_fraction": "0.3112913795305277",
      "same_index_weak_recovery": "6"
    }
  ],
  "continuation_full_ndt_calls": 0,
  "final_diagnostic_ndt_calls": 0,
  "support_cycles": 0,
  "gt_used": false,
  "oracle_v_injected": false,
  "online_efficiency_claim": false
}
```

NEXT = CLOSE_EXISTING_BRANCH_NUMERICAL_CONTRACT_WITHOUT_NEW_SEARCH
