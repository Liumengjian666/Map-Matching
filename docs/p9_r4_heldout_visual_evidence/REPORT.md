# P9-R4 local recovery and pre-alignment input STOP

## Decision status

Local Git recovery succeeded. **The held-out scientific experiment has not run.**
The first oracle frame failed the frozen source point-count/FNV check before
`ndt.align()`. This is an input-provenance blocker, not a held-out visual evidence
failure. None of the six R4 scientific result classes is established.

`scientific_final_result = null`

`NEXT = R4_SOURCE_CLOUD_PROVENANCE_CLOSURE`

No NDT parameters, frontend parameters, source hashes, T0, U_obs, selection rules,
budgets, clustering rules or success gates were substituted. No baseline replay
or source reconstruction was started. Extra baseline NDT replay calls require
explicit authorization and accounting before proceeding.

## Git and environment

- Independent workspace: `/tmp/dog_loc_paper_r4_ws.Fq21k2`.
- Branch: `research/p9-r4-heldout-visual-evidence`.
- Start: `3a98a3cd1d64d7aba6888d3136295a0b41c38fa6`.
- Local clone used `--local --no-hardlinks`; source `.git` remained read-only.
- Writable `.git` and `git update-index --refresh` passed.
- Origin URL is `https://github.com/Liumengjian666/Map-Matching.git`; no
  clone/fetch/pull/push network operation was performed during this recovery.
- The original paper workspace, stable workspace, and all datasets were not
  modified. Historical unrelated untracked files were not copied into this clone.
- Local commit identity copies the original repo's scoped `Codex / codex@local`;
  no global configuration was changed.
- Portable delivery is `/tmp/p9_r4_heldout_visual_evidence.bundle`. In addition to
  branch ancestry it must include frozen source commit
  `9945c4f5c3d7759104de108a594bcaf2553fd78c`, which is present in the source object
  database but is not an ancestor of the R4 branch. Bundle validation is a Git
  delivery check, not an experiment result. End SHA is obtained from Git after
  commit; it is deliberately not inserted into its own committed artifacts.

## Completed pre-experiment gates

The fully label-blind ordered pool has 3,630 eligible transactions and 160
accepted targets. Development +/-8 exclusion and target separation >=16 remain
unchanged. Ordered pool SHA256:

`e877aeec2b6df1fa49dd11bc837b748612ec17b0b5a2d850fa018ed99b624d37`

The frozen C++ geometry carrier and original complete-link implementation
reproduce **24 historical BASE263 frames: 9 MAJOR, 15 NO_MAJOR, all 22 major IDs**.
Iteration-limit converged terminals are retained. The float boundary regression
uses the original Eigen float subtraction/norm, not a double approximation.

The literal frozen BASE263 seed generator reproduces 8,416 historical start
poses (32 x 263): maximum translation difference 0 m, maximum rotation difference
`6.556881449863406e-15` deg. Frozen R2A conditioned proposal matrices and first-12
farthest-point order match historical rows. The 42,080 held-out proposals are
geometry only; no candidate alignment was run.

## Actual blocker and read-only diagnosis

The attempted oracle batch was the first 96 ordered targets. It stopped at the
first target, **TX2932**, before any alignment:

| Source property | Frozen SAME_OBJECTIVE baseline | Topic-bag extraction |
| --- | ---: | ---: |
| Prepared source points | 413 | 427 |
| FNV source hash | 3530993910003886054 | 1188874421812550009 |

The read-only source audit applies the frozen source preprocessing to existing
raw XYZ float files; it calls neither NDT alignment nor the visual frontend.

- Historical archived source admission: **32/32 PASS**.
- Decoded held-out source admission: **0/160 PASS**; all 160 differ in source hash
  and/or point count. These source paths are **invalid experimental inputs**.
- Exact raw XYZ parity, original SAME_OBJECTIVE export vs R10B topic-bag decoded
  cloud: **0/6**, tested TX120/368/616/2226/2350/3341.
- SAME_OBJECTIVE exported sources found: **32**, all development frames.
- Exported held-out sources found: **0**.

The actual source-generation code is archived verbatim in
`frozen_p7_source_export_reference.txt`, Git `9945c4...`, SHA256
`610b48160eda879b70e683ba6c530ba977338bc57c84e6027239c26b802de708`.
It builds sources through the later P7 current-baseline timed-cloud scan-end
deskew replay, then exports only `isP5I1CohortFrame(transaction)`—the 32 historical
development targets. The prior R10B runtime topic bag is valid historical input
context, but its `cloud_end_frame` bytes are not these later replay's source bytes.

Consequently, the topic-bag extraction in `run_r4_oracle.py prepare` must not be
treated as a recovered SAME_OBJECTIVE source. Merely accepting its new hashes
would violate the experiment's objective/input identity. The immutable preparation
receipt is retained as the attempted input record, not marked admitted or repaired.

## What did not execute

| Requested output | Status |
| --- | --- |
| Final 96/128/160 cohort selection | Not reached; no held-out labels |
| Oracle263 calls | 0 |
| Candidate B12 calls | 0 |
| New visual pair extraction | 0 |
| Blind non-oracle evidence freeze | Not reached |
| Coverage, recall, AUC, permutation, LOFO | Not computed |
| False positive/negative and GT arbitration | Not computed |
| GT loaded | NO |
| Push executed | NO |

Historical labels were read by the isolated parity process and read-only archive
verification, never by the evidence builder. The unexecuted blind evidence
scaffold has no oracle/canonical/GT inputs. Its entry points refuse
execution when the input STOP receipt exists. No synthetic empty experimental
CSVs or statistical zeros stand in for missing results.

## Verification and independent review

The offline Release build, historical P9 tests and R4 self-tests are rerun by
`verify_r4_input_stop.py`. Actual command logs, CTest XML, CSV/JSON consistency,
frozen input/source/binary hashes and frontend numerical carrier audit are saved
under `verification/` and `artifact_hashes.json`. An audit PASS means the archive
truthfully reproduces the input FAIL; it does not mean source admission PASS.

Fresh-context read-only reviews led to concrete corrections: literal float
oracle boundary math; partial/repeated-call guards and truthful zero-iteration
status; exact cohort hash/coverage binding; revalidation of reused calibration;
and readable append/canonical-file isolation regressions. Synthetic tests cover
these corrections. The evidence scaffold remains **unexecuted and not
experimentally validated**. No external CLI second opinion was run.

## Required next authorization

Recover/export exact current-baseline SAME_OBJECTIVE scan-end sources for the
already frozen held-out IDs, and require point-count/FNV parity against the
existing registration/U_obs records before any oracle alignment. Keep source
preprocessing, T0, W2, oracle seeds, held-out order, B12, EPS_SCORE, visual frontend
and all scientific gates unchanged. If a faithful baseline replay is required,
first approve its scope and additional nominal NDT cost. Do not resume R4 by
changing a source hash or removing a selected frame.
