# Submission checklist — advisor-review package, not submission authorization

Target: **IEEE Sensors Journal, Regular Paper**. Backup: **Measurement Science and Technology**. Checked2026-10-24 against the official sources listed below; no portal login/upload/submission was performed.

## Journal format and current official requirements

|Item|Official basis|Package treatment / status|
|---|---|---|
|Double-column manuscript|[Sensors Journal Guide for Authors](https://ieee-sensors.org/ieee-sensors-journal/for-authors/)|PASS: IEEEtran journal mode,10pt, US Letter,7pages in the final preview|
|Normal length|Same guide: normally≤8double-column pages, with overlength charges after8; not stated as an absolute rejection limit|PASS:7pages including references and visible author placeholders; adding authors may change pagination|
|Template|[IEEE Author Center templates](https://journals.ieeeauthorcenter.ieee.org/create-your-ieee-journal-article/authoring-tools-and-templates/tools-for-ieee-authors/ieee-article-templates/)|Standard IEEE journal class supplied by[CTAN IEEEtran](https://ctan.org/pkg/ieeetran), version1.8b. Interactive IEEE selector returned418; legacy jsen.zip403. This is not falsely described as the downloaded Sensors-specific ieeecolor package. Verify any journal-specific front-page styling in the author portal before upload|
|Abstract|[IEEE article structure](https://journals.ieeeauthorcenter.ieee.org/create-your-ieee-journal-article/create-the-text-of-your-article/structure-your-article/), [IEEE style manual](https://journals.ieeeauthorcenter.ieee.org/wp-content/uploads/sites/7/IEEE-Editorial-Style-Manual-for-Authors.pdf)|PASS:198words, single paragraph, no citations/equations. Problem, method, complete-window improvement,8extra calls and limits included|
|Keywords|IEEE article structure; journal provides[editorial categories](https://ieee-sensors.org/wp-content/uploads/2017/02/IEEE-Sensors-Journal-Editorial-Keywords.pdf)|PASS:5index terms. Suggested portal category DATP(sensor data processing) or SYST(integration), with APPL(robotics) if allowed; authors select actual portal fields|
|Graphical abstract|Journal guide explicitly makes it mandatory;[IEEE supplementary guidance](https://journals.ieeeauthorcenter.ieee.org/create-your-ieee-journal-article/prepare-supplementary-materials/)|PASS: `figures/gagraphic.png`,660×295px,300dpi metadata,33321bytes; below recommended45KB. PDF/SVG available, also previewed in paper. Upload during peer review as separate graphical abstract|
|Figures and tables|[IEEE resolution/size guidance](https://journals.ieeeauthorcenter.ieee.org/create-your-ieee-journal-article/create-graphics-for-your-article/resolution-and-size/)|PASS:vector PDF figures, two-column tables, embedded fonts, numbered captions and complete trajectories. No plot-estimated accuracy numbers|
|Mathematics|Journal guide; IEEE style|PASS:8numbered equations; right/left map transform and map-yaw seeds explicit; no new solver/filter claim|
|References|IEEE numbered citation style|PASS:14BibTeX entries compiled using IEEEtran.bst; all cited keys resolved. DOI/page/year checked against R26 primary extraction; Autoware is explicitly official software documentation|
|Data/software availability|Public dataset terms plus reproducibility disclosure; not asserted here as a newly verified Sensors-specific mandatory portal field|Present in main paper. FINAL RELEASE URL/DOI and permissions require author confirmation; no false claim of uploaded results|
|AI-assisted text/graphics/code|[IEEE AI-generated content policy](https://open.ieee.org/author-guidelines-for-artificial-intelligence-ai-generated-text/)|Draft acknowledgment identifies Codex/ChatGPT and writing/summary/plotting/typesetting use. Authors must verify exact version, scope and all content; author responsibility/approval is not assumed|
|Submission portal|Journal guide's Author Portal link resolves from Atypon to Research Exchange|Use the link on the current official guide; direct attempts were read-only. Do not rely on old ScholarOne instructions. No account action taken|

The source checks establish current public instructions, not a guarantee that every portal field or production rule is unchanged. Fees are not estimated or paid. Journal acceptance and sufficient novelty remain editorial judgments.

## Evidence and claim checks

- [x] TableI:2Fast/SuperLoc/Koide/Autoware technical differences; established gates/retries/fusion acknowledged.
- [x] TableII:original/G-only/C+G independent actual causal executions, complete60frames.
- [x] TableIII:accuracy tails,8extra calls, measured callback time, NDT/process-pair memory scope.
- [x] TableIV:published full-sequence context, clearly not a fair external ranking.
- [x] Exact RMSE preserved:7.986422m/13.052649°;7.986422m/13.052649°;0.209943m/2.984149°.
- [x] C+G57/60map observations,8extra calls, initial max11.245245° and2high-error outputs retained.
- [x] P45 historical82.2%processHWM reduction includes latency/tail-error caveats; no whole-machine claim.
- [x] Conditional Base/reference limitations and the material relative-admission safety counterexample remain visible.
- [x] No claim of10Hz certification, completed robot deployment, independent mocap accuracy, external SOTA superiority or new NDT/EKF theory.
- [x] No new experiment, external algorithm reproduction, dataset download or production modification.

## Mandatory author/advisor actions before submission

- [ ] Approve scientific scope, novelty wording and adequacy of the existing evidence.
- [ ] Replace author/order/affiliation/corresponding-author placeholders with confirmed information; verify ORCID/portal requirements.
- [ ] Complete funding and conflict-of-interest declarations using actual information; do not assert unsupported institutional approvals.
- [ ] Confirm originality/prior publication/preprint disclosure and no simultaneous submission; do not submit to the backup journal concurrently.
- [ ] Approve data/software release permissions and a genuine accessible archival revision or DOI.
- [ ] Verify exact AI-assistance disclosure and take responsibility for all equations, text, citations and claims.
- [ ] Rebuild after author edits; check final page count, template appearance, figure upload categories and portal-generated PDF.
- [ ] Explicitly authorize the corresponding author to upload/submit. This package alone supplies no authorization.

## Advisor decision on minimum evidence

Recommendation: **send to the advisor now; do not claim immediate journal-readiness merely because formatting passed**. The key limits are one exposed successful recovery window, no matched external disturbed-initialization baseline and unresolved file-level frame certification. If the advisor requires one further experiment, the narrowly useful option is a preregistered, parameter-frozen real-IMU causal comparison on a previously unused time window. It is not executed or scheduled here.

Measurement Science and Technology is a backup only. Its[official scope](https://publishingsupport.iopscience.iop.org/journals/measurement-science-and-technology/about-measurement-science-technology/) requires an advance in measurement technique and consideration of uncertainty/precision/accuracy. The same evidence limits remain; no claim is made that changing journals removes them.
