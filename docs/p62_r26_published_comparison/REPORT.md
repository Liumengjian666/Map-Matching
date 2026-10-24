# R26: Published baseline comparison and paper finalization

TASK: P62-R26-PUBLISHED-BASELINE-COMPARISON-AND-PAPER-FINALIZATION

**Result: LITERATURE_AND_MANUSCRIPT_CLOSURE_COMPLETED.** Published evidence has been used to revise a complete English paper. No external GradeA comparison, SOTA superiority, independent physical accuracy or new robot validation is claimed.

## Git and scope

START `6aeecf2adbc77c233ee0febaed192643a20c2f54`.
Branch `research/p62-r26-published-comparison-paper`.
Workspace `/home/jian/livox_ws/p62_r26_published_comparison_ws`.
Repository `https://github.com/Liumengjian666/Map-Matching`.
The final manuscript/review Git identities are in `GIT_IDENTITY.json` and the final handoff. The estimator code remains exactly at START. R26 changes only this new documentation/evidence/tools directory. Local commits only; no push, merge, publication, email or submission executed.

## What was done

1. Inspected seven2022–2026 published papers: FAST-LIO2, DLIO, STEAM-LIO, Singer-LIO, SuperLoc,2Fast-2Lamaa and Koide's tightly coupled prior-map localization; treated FAST-LIO-LOCALIZATION separately as software.
2. Recorded23 metadata fields per entry, including all requested categories, primary links, original table/page locators, ownership of results, metrics, maps, alignment, timing/hardware and NR fields.
3. Verified2Fast Table7's0.378/0.360/0.304/0.262m odometry values and its separate0.128m localization result in the actual public author v3 text. Older v2 table6 is not mixed into this extraction; full publisher fetch was blocked, and that version limitation is disclosed.
4. Traced the quoted baselines back to DLIO TableI0.3775/0.3606m and STEAM TableII0.3042m; added Singer TableIII0.3020m. These are not all new baselines run by2Fast authors. DLIO's initial100-pose exclusions and exact alignment-command NR are retained.
5. Established Short/01-Short/01_short identity for the originalOS1 sequence through official1530s description, paperlength1609m and two author-output files,15301timestamps each matching localPCD order exactly. Did not equate laterOS0 extensions.
6. Found and inspected public STEAM/Singer trajectories, pinned repository819c6077effc9f9597a8ad6a1318091d6be7b8a1. Time/schema passed; certified Leica/body-frame binding and matching disturbed-window initialization did not. No transform fit or manufactured windowRMSE.
7. Classified six distinct NewerCollege method/task rows as B; SuperLoc/Koide/FAST-LIO-LOCALIZATION context as C. Direct external A rows0. Internal original/G-only/C+G ablation remains directly comparable within its own fixed protocol.
8. Created three separate tables and a complete revised English manuscript titled **Gate-Aware Budgeted NDT Recovery for Prior-Map LiDAR–Inertial Localization**. RelatedWork now compares the actual mechanisms of2Fast,SuperLoc and Koide instead of only comparing numbers. Setup, quantitative context, resource scope, contribution wording and Discussion were rewritten.

## Key scientific facts preserved

|Controlled W1/C3|TranslationRMSEm|RotationRMSEdeg|Native nominal+extra|Map observations|
|---|---:|---:|---|---:|
|Original R23|7.986422|13.052649|0+0|0/60|
|G-only R25|7.986422|13.052649|60+0|0/60|
|C+G R24|0.209943|2.984149|60+8|57/60|

C+G first admits raw2603's-5deg restart;2 initial high-error outputs remain, with maxrotation11.245245deg. G-only60converged runs yield0map admissions, supporting additional contribution of the retry in this exposed development case. It does not show universal recovery.

P45's82.2% observed NDT-process HWM reduction is kept separate from R24 two-nodeRSS and external final-RAM reports. P45 steady alignment rises7.086→8.079ms, translation max and rotationRMSE worsen, and the recorded wall exceeds sensor span. No inter-paper speed or memory ratio and no10Hz claim.

The R22 relative-admission safety counterexample is retained; D/E/F/Coupled are not revived. Conditional Base interpretation, map-registered official reference, Full-source Coverage Consistency Mean (not independent holdout), and blocked dog calibration provenance are explicit. Robot experiments were not rerun or completed by literature review.

## Outputs and verification

The six requested files are present: `PUBLISHED_BASELINE_RESULTS.md`, `PAPER_COMPARISON_TABLES.md`, `COMPARABILITY_AUDIT.md`, `RELATED_WORK_COMPARISON.md`, `PAPER_FINAL_DRAFT.md`, `SUBMISSION_READINESS.md`. Metadata, fetch receipts, public-output checks, frozen input hashes, necessary local verification and Git identities accompany them. Full copyrighted PDFs/public trajectories reside outsideGit at `/home/jian/livox_ws/p62_r26_published_baseline_sources/`; their URLs/versions/hashes are archived.

Actual new NDT calls0; external algorithm reproductions0; external algorithm builds/clones0; dataset downloads0; new error-fit evaluations0. Only small papers, documentation and already-published output files were retrieved. R25 inputs, algorithm and stable workstation identity are checked without re-running research. Commands and results are in `TESTS_AND_REVIEW.md`, `logs/`, and `FINAL_VALIDATION.json`.

## Readiness and unique NEXT

The paper now has a usable international comparison and a complete author-review draft. It does **not** acquire a fair external ranking through published numbers. The most consequential remaining scientific limits are the conditional frame binding, absence of a matched external prior-map baseline, and limited C+G test range. Novelty remains a bounded system-design claim subject to venue judgment; physical deployment and independent truth are not established. These limitations do not require repeating published algorithms just to recover known values.

Unique NEXT: 科研总控核查真实稿件与证据后，以准确收窄的系统研究主张决定目标期刊和作者审定／投稿格式整理；本执行轮不自动补实验、标定、算法或下一阶段。
