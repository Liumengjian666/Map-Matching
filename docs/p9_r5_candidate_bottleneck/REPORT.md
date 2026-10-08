# R5 post-hoc candidate bottleneck attribution

FINAL_RESULT = `MIXED_CANDIDATE_BOTTLENECK`

NEXT = `REASSESS_NONLOCAL_AMBIGUITY_ADMISSION_CONTRACT`

R4 remains HELDOUT_LIDAR_CANDIDATE_GENERATOR_NOT_GENERALIZED (17/40).
This is POSTHOC FAILURE ATTRIBUTION, not new confirmatory evidence.

## Observed coverage and failure mechanisms

| Recorded terminal pool | MAJOR strict coverage | NO_MAJOR strict coverage |
| --- | ---: | ---: |
| B12 | 17/40 | 3/56 |
| FULL263 | 30/40 | 5/56 |

B12 and FULL263 both positive: 17; B12-only: 0; FULL263-only: 13; neither: 10 MAJOR frames.
Frozen ID-recovery versus strict eligibility cross-table: `{"ID_and_strict": 12, "ID_only": 17, "neither": 6, "strict_only": 5}`.
FULL263 is only the attainable coverage of the existing recorded pool.
The conditioned B12 starts are not a nested subset of FULL263, so this is
not a matched-budget causal efficiency claim or a global reachability bound.

FULL263 MAJOR exclusive causes: `{"ELIGIBLE": 30, "REPRESENTATIVE_SUPPRESSION": 4, "SCORE_REJECTION": 6}`.

Mutually exclusive B12 frame causes:

| Cause | MAJOR frames |
| --- | ---: |
| NO_NONNOMINAL_CLUSTER | 0 |
| INSIDE_CENTER_ONLY | 0 |
| SCORE_REJECTION | 22 |
| REPRESENTATIVE_SUPPRESSION | 1 |
| ELIGIBLE | 17 |

Raw B12 MAJOR terminals: 480; strictly separated: 233; strictly separated but score-rejected: 185; raw strict+score-pass: 48; raw eligible suppressed by representative choice: 3.
These terminal counts are not independent statistical samples and are not frame counts.
Overlapping frame flags: `{"N0": 0, "N1": 13, "N2": 40, "N3_after_clustering": 17, "N3_raw": 18}`.

## Oracle versus online definition

Oracle: supported cluster, separated center, finite per-source score gap from the best supported basin.
Online: non-nominal complete-link cluster, strictly separated representative, raw score>=nominal+EPS_SCORE.
EPS_SCORE=2.747604276e-4 remains frozen. No alternate tolerance is evaluated.
Across 84 major centers: score classes {'NEGATIVE': 39, 'POSITIVE': 45}; actual online center separation holds for 84.
Frames with some score-rejected major: 27/40; frames with every major center score-rejected: 14/40.
Thus major ID membership and online eligibility are mathematically different events.
If nonlocal ambiguity is intended to include near-optimal alternatives admitted by the oracle,
requiring an objective improvement over T0 expresses a narrower property. This is a semantic
interpretation of the measured definition mismatch, not a claim that a new threshold is valid.

## The 17 ID-recovery disagreements

Counts below are recovered terminal rows; several rows may recover the same frozen ID.
Rejection columns overlap. Exact poses, score differences, members and representative geometry
are retained for every row in id_recovery_disagreement.csv.

| Frame | Recovered IDs | Hit rows | Actual inside | Actual score rejected | Representative inside | Primary frame cause |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 2932 | P03 | 3 | 2 | 2 | 3 | SCORE_REJECTION |
| 3813 | P04 | 1 | 0 | 1 | 0 | SCORE_REJECTION |
| 3929 | P09 | 1 | 0 | 1 | 0 | SCORE_REJECTION |
| 2398 | P04;P08 | 7 | 7 | 0 | 7 | SCORE_REJECTION |
| 1944 | P02 | 1 | 1 | 1 | 1 | SCORE_REJECTION |
| 3783 | P01;P05 | 3 | 2 | 2 | 2 | SCORE_REJECTION |
| 2374 | P04 | 3 | 0 | 3 | 0 | SCORE_REJECTION |
| 184 | P03 | 6 | 6 | 0 | 6 | SCORE_REJECTION |
| 655 | P15 | 3 | 0 | 3 | 0 | SCORE_REJECTION |
| 3200 | P04 | 2 | 0 | 2 | 0 | SCORE_REJECTION |
| 2675 | P03 | 1 | 1 | 0 | 1 | SCORE_REJECTION |
| 2889 | P04 | 5 | 1 | 4 | 1 | SCORE_REJECTION |
| 3899 | P09 | 1 | 0 | 1 | 0 | SCORE_REJECTION |
| 2483 | P06 | 4 | 4 | 0 | 4 | SCORE_REJECTION |
| 4110 | P05 | 1 | 0 | 1 | 0 | SCORE_REJECTION |
| 542 | P06 | 3 | 1 | 3 | 1 | SCORE_REJECTION |
| 3829 | P05 | 2 | 0 | 2 | 0 | REPRESENTATIVE_SUPPRESSION |

Aggregate recovered-terminal rejection reasons: `{"NOMINAL_CLUSTER": 24, "REPRESENTATIVE_INSIDE_CENTER": 26, "REPRESENTATIVE_SCORE_REJECTED": 23, "TERMINAL_INSIDE_CENTER": 25, "TERMINAL_SCORE_REJECTED": 26}`.

Recovered-terminal geometry/score counts: `{"inside": 25, "inside_and_score_rejected": 4, "score_rejected": 26, "separated_score_pass": 0, "separated_score_rejected": 22, "terminals": 47}`.
Frame cause uses all B12 terminals, not only ID-recovering rows. Raw eligible suppression details:

```json
[
  {
    "frame": "2997",
    "probe_rank": "7",
    "seed_index": "248",
    "rotation_deg": "2.297844618769635",
    "score_difference": "2.510919943058525",
    "representative_rank": "11",
    "representative_rotation_deg": "1.7187560143611464"
  },
  {
    "frame": "2997",
    "probe_rank": "12",
    "seed_index": "210",
    "rotation_deg": "2.019461742974747",
    "score_difference": "2.674694517182843",
    "representative_rank": "11",
    "representative_rotation_deg": "1.7187560143611464"
  },
  {
    "frame": "3829",
    "probe_rank": "10",
    "seed_index": "245",
    "rotation_deg": "2.4652640900569183",
    "score_difference": "6.597032233744358",
    "representative_rank": "5",
    "representative_rotation_deg": "1.8470270431097875"
  }
]
```

## NO_MAJOR controls and interpretation

All 56 controls are listed in no_major_controls.csv. Frames 3147, 3951 and 167 have full
B12 and FULL263 terminal records in no_major_positive_details.csv, including poses, nominal
separation, score difference, cluster membership and rejection/eligibility reason.
NO_MAJOR is the frozen oracle proxy; its strict-positive events are not automatically errors.

## Main conclusion and boundaries

In 13 MAJOR frames B12 lacks a usable candidate that exists in BASE263; this supports a finite-pool search shortfall. Separately, the funnel documents eligibility
losses among already reached terminals, and oracle centers can be lower-scoring than T0.
These mechanisms overlap; their counts must not be summed into a causal percentage.
The data do not justify describing the entire 42.5% coverage as a failure to reach any
other terminal, nor as proof that increasing search to 263 universally solves the gate.
The sole proposed next decision is `REASSESS_NONLOCAL_AMBIGUITY_ADMISSION_CONTRACT`.
This report does not implement a new eligibility definition or change R4.

## Provenance, validation and cost

Start: 06ff017831d2654a62c72f854926b9d3b459ce40; branch: research/p9-r4-heldout-visual-evidence; workspace: /tmp/dog_loc_paper_r4_ws.Fq21k2.
Input manifest pins 14 R4 artifacts to that commit plus the original helper/geometry hashes.
All 96 B12 clusters and strict counts must reproduce their frozen records exactly.
The full263 self-test checks rank263 inclusion, score equality admission, convergence/iteration-limit
handling and a representative-inside suppression example. Required Release/P9 checks and hash
audit are recorded in verification/; the frozen solver/frontend code was not changed.
Offline analysis wall time: 709.690142 s (4 processes).
This is offline post-hoc analysis cost, not online robot runtime.
NEW_NDT_CALLS=0; BASELINE_REPLAY_CALLS=0; VISUAL_EXTRACTION=0; GT_LOADED=NO; PUSH_EXECUTED=NO.
The containing Git commit supplies the end SHA, avoiding a self-referential hash.
