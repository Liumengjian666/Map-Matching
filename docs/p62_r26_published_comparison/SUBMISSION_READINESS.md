# Submission readiness: complete scientific-review draft, conditional claims

**R26 outcome: LITERATURE_AND_MANUSCRIPT_CLOSURE_COMPLETED.** A full revised English paper, three protocol-separated tables, seven published-paper records and an explicit software record are present. No new estimator experiments are implied. This is not editorial acceptance or authorization to submit.

## What is now ready

- Complete manuscript with Abstract, Introduction, RelatedWork, Methodology, Setup, Results, Discussion, Conclusion and linked bibliography.
- Corrected publication attribution: FAST-LIO2 T-RO2022, DLIO ICRA2023, STEAM-LIO T-RO2025 (online2024), Singer Robotica2025, SuperLoc ICRA2025, Koide ICRA2024 and2Fast IJRR2026.
- Six distinct Newer College method/task result rows, separating aligned odometry from direct-map localization. 2Fast Table7 five requested values verified in public author v3; copied baseline provenance explicit.
- Existing actual controlled ablation and resource measurements reused without new runs; missing robot validation, map-anchored reference, conditional frame interpretation, transient rotation errors, failed relative admission and non-demonstrated freshness switch remain visible.
- Two available published trajectories inspected,15301timestamps each exactly matching the local scan order; no fabricated frame conversion or test-window alignment.
- C+G mathematics and production/history remain frozen. R26 nativeNDT calls0, external reproduction0, new datasets0.

## Items affecting the strength of a submission

|Issue|Why it matters|Treatment in current manuscript|Decision before submission|
|---|---|---|---|
|Conditional Base/CSV frame binding|A mathematically valid transform chain is not final certification of the local export axes; absolute rotational claims are conditional|Label every accuracy comparison reference-relative, explain official RS_C1/Base distinction, no new fitted correction|Authors must accept this limited benchmark claim or resolve provenance outside this task; do not silently call it independent accuracy|
|No matched external prior-map baseline|Published results have different task/range/alignment/init; TableA cannot establish SOTA|No external ranking or superiority claim. Main claim is a controlled within-system failure/ablation|Choose a venue accepting system-design evidence, or preregister one genuinely matched baseline only if editorial scope demands it; no automatic run|
|Limited C+G validation|One decisive exposed6s/60scan perturbed window plus normal controls, not full-sequence/cross-dataset validation|Explicit development-case claim and no generalized safety guarantee|Do not advertise cross-scenario robustness; manuscript may need broader evidence for a methodological journal|
|Modest originality|Gate placement and bounded multistart are not new mathematical principles|Technical comparison to2Fast/SuperLoc/Koide; contribution is reproducible causal integration|Research controller/editorial judgment required; do not relabel it as a new optimizer or certified innovation|
|Robot experiment absent|Real assets exist but physical LiDAR–IMU calibration provenance blocked legal comparison|DOG experiments NOT_RUN, no robot accuracy/time claims|Does not block literature/manuscript completion; retain limitation rather than claiming deployment|
|Resource scopes and hardware detail|Historical P45 HWM and recent sampled two-nodeRSS differ; selected receipts lack complete historical CPU metadata|Units/scope/hardwareNR disclosed; no cross-platform speedup or10Hz claim|Recover already-existing hardware record if needed for formatting; current machine model is not retrospective proof|
|Publisher version availability|2Fast publisher full download403/timeouts; inspected v3 is post-first-online author version|Version/page explicitly pinned; other publisher/author PDFs hashed|Before final copyedit, check version-of-record if accessible; no substituted claim of PDF identity|

These are evidence limits, not reasons to postpone the already completed paper draft or automatically reopen algorithm development. The absence of a same-protocol external result particularly constrains a SOTA paper; it does not invalidate the observed G-only/C+G ablation.

## Mechanical submission items, not scientific results

Author names/order/affiliations, corresponding-author approval, funding/conflicts/ethics statements as applicable, selected journal template, word/figure/reference style, data/code availability links, final PDF layout and cover letter remain team decisions. No journal suitability, fee or policy claims are made here. R26 has not submitted, emailed, pushed or published anything.

## Unique NEXT

科研总控先核查本轮真实归档与完整英文稿，决定是否以“有明确边界的系统设计与因果消融研究”进入目标期刊的作者审定和投稿格式整理；不自动追加算法、标定任务或外部复现实验。若判定证据不足以支撑拟投期刊，明确收窄主张或另行下达唯一的补充证据合同，不能用TableA充当缺失的公平基线。
