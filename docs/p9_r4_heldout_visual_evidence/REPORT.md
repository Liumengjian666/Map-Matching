# P9-R4 recovered-source held-out candidate gate

FINAL_RESULT = `HELDOUT_LIDAR_CANDIDATE_GENERATOR_NOT_GENERALIZED`

NEXT = `REASSESS_NONLOCAL_CANDIDATE_GENERATOR`

The final prefix is **96 held-out frames: 40 MAJOR and 56 NO_MAJOR**.
The prefix stopped at the first sufficient label count; it was not expanded or
selected using candidate recall, visual evidence or GT. None of the32 development
frames contributes to these scientific statistics.

The fixed B12 predictor-conditioned WEAK2 generator produced a strict separated,
objective-competitive alternative in **17/40 MAJOR frames (42.5%)**, below the
predeclared60% gate. Therefore **no visual extraction, visual coverage assessment,
primary AUC/permutation/LOFO, delete-one-major test, post-hoc GT or LiDAR secondary
classification was run**. These are NOT_RUN, not zero scores or visual failures.

## Scientific interpretation and limits

- Frozen-ID macro frame recall = **0.51625000**.
- Frozen-ID micro recall = **39/84 = 0.46428571**.
- Strict-candidate frame coverage = **17/40**.
- 23 MAJOR frames lack a usable strict objective-competitive candidate.
- This is upstream candidate-generator non-generalization, not a visual evidence
  failure, a source provenance failure, or a rejection of DUAL-U or H1.
- Oracle ID admission retains the frozen .2m/2deg neighborhood and its overlap
  limitation. ID recall does not prove strict dynamic local-minimum identity.
- NO_MAJOR is a frozen BASE263 oracle proxy, not proof of absence of ambiguity.
- No pose switching, EKF, covariance fusion, new frontend/score tuning or budget
  expansion was performed. The unique NEXT is a research decision, not permission
  for this execution turn to implement a new candidate mechanism.

## Frozen contracts

Source closure commit: `86ec15a279b84cc970530976a7e998af354a6d33`. Authorized source cache:
`/tmp/p9_r4_same_objective_source_recovery.l6hlb3lc`. Held-out raw/prepared parity160/160; closure raw SHA checks192/192.
TX2932 remains413 points / FNV3530993910003886054. Prior32/32 raw parity,
4127/4127 trajectory parity and2400/2400 manifest-field parity remain frozen.
No baseline replay was repeated. T0, U_obs/W2 and nominal scores remain the
original SAME_OBJECTIVE archives, not recovery replay replacements.

Ordered pool SHA256:
`e877aeec2b6df1fa49dd11bc837b748612ec17b0b5a2d850fa018ed99b624d37`.
Eligible3630; pool160; development exclusion+/-8; targets separated>=16.
Historical oracle parity24/9/15/22 and original seed/B12 order parity are frozen
prerequisites, hash-verified without new historical NDT calls.

NDT: PCL1.10, resolution .8, step .08, epsilon1e-5, max iterations80.
Oracle: original BASE263 and original right/body seed convention.
Candidates: fixed B12 predictor-conditioned WEAK2, seed122 first, original
deterministic farthest-point ordering, no adaptive/oracle selection.
Clustering: frozen deterministic complete-link .2m AND2deg; representative maximum
raw score; competitive score>=S0+2.747604276e-4. Strict alternative center:
translation>.2m OR rotation>2deg. Converged iteration-limit rows are retained.

## Calls and costs

| Stage | Frames | Calls | Wall seconds | Mean align ms/call | Alignment sum seconds |
| --- | ---: | ---: | ---: | ---: | ---: |
| Oracle |96 |25248 |758.727117 |90.785380 |2292.149266 |
| Candidate B12 |96 |1152 |84.175607 |68.928902 |79.406095 |

Oracle uses4 processes; its wall includes clustering/dispatch and is not the sum
of alignment times. Candidate wall includes dispatch. Mean B12 alignment cost per
frame is827.146820ms; mean iterations/call
28.381944, iterations/frame340.583333.
Oracle status counts: `{'ITERATION_LIMIT': 1546, 'SUCCESS': 23702}`; candidate: `{'SUCCESS': 1102, 'ITERATION_LIMIT': 50}`.
All returned PCL converged=1; iteration limits do not get silently deleted.

Historical source recovery4127 baseline calls /124.151205s remain
**OFFLINE_PROVENANCE_ONLY**, not oracle, candidate, online-method or paper runtime.
This resume performed0 new baseline calls and0 visual extractions.

## Information isolation and audit trail

`candidate_freeze.json` and `lidar_evidence_freeze.json` bind blind candidate
outputs and builder SHA before `evaluation/candidate_gate.json` reads labels.
Candidate stage permission is ID/hash-only. No visual-stage permission was issued
after the candidate gate failed. GT was never loaded.

All96 per-frame outcomes, including the23 upstream counterexamples, remain in
`evaluation/candidate_per_frame.csv`. These are not visual classifier false
positives/negatives because that classifier was not evaluated. No frames removed.

The old blocked topic-source manifest, source audit, topic parity, stop receipt,
original execution manifest and oracle/engine_96.log remain untouched. Previous
root REPORT/results/hash manifests are additionally preserved byte-exact under
`prior_blocker_receipts/`. The accepted source_recovery archive is unchanged.

New resumed outputs are in oracle/, candidate/, and evaluation/. Visual and
unreached evaluation CSVs are header-only with explicit NOT_RUN JSON/coverage
receipts; they do not claim visual-unavailable measurements for96 frames.

Verification: Release build;40/40 P9 CTest; recovered cache/source admission;
oracle input/source guard; candidate parity; frozen frontend SHA/environment;
label/GT read denial tests; independent code review; CSV/JSON/hash audit;
git diff --check. A failed pre-alignment hash guard was resealed before any align
after the reviewed runner fix, retaining the original preflight receipt. The
earlier incorrect CTest invocation found no tests and is preserved separately;
it is not counted as a test pass.

## Git delivery

Workspace: `/tmp/dog_loc_paper_r4_ws.Fq21k2`; branch `research/p9-r4-heldout-visual-evidence`; start `86ec15a279b84cc970530976a7e998af354a6d33`.
End SHA is the containing commit, obtained from Git after commit.
PUSH_EXECUTED=NO. Portable bundle `/tmp/p9_r4_heldout_visual_evidence.bundle` must
contain the completed branch and frozen source commit `9945c4f5c3d7759104de108a594bcaf2553fd78c`.
Only execution/guard/archive code changed; frozen solver and frontend algorithms
were not modified. Original read-only paper/stable workspaces remain untouched.
