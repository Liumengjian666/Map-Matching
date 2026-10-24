# R27 advisor-review and journal submission package

**Algorithm and experiment results are frozen. Nothing was submitted or pushed.**

Primary editable manuscript: `PAPER_SUBMISSION_DRAFT.tex`.
Review PDF: `PAPER_SUBMISSION_PDF.pdf` (7pages, standard IEEE journal double column).
Bibliography: `REFERENCES.bib`, compiled reference list `REFERENCES.md`.

Accompanying files:

- `COVER_LETTER_DRAFT.md` / `.docx`: author-approval draft with no invented declarations.
- `HIGHLIGHTS.md` / `.txt`.
- `FIGURE_CAPTIONS.md`; `figures/gagraphic.png/.pdf/.svg` and vector figures.
- `SUPPLEMENTARY_MATERIAL.tex/.pdf`: protocol lineage, negative evidence and resource/reference limits.
- `NOVELTY_POSITIONING.md`.
- `SUBMISSION_CHECKLIST.md`: current official instructions and uncompleted author actions.
- `ADVISOR_REVIEW_SUMMARY.md/.pdf`: Chinese, no more than2pages.
- `EVIDENCE_MAP.csv`, input identities, figures/provenance, typesetting/validation receipts.
- `P62_R27_ADVISOR_REVIEW_PACKAGE.zip`: convenient portable review bundle.

## Rebuild

With a normal TeX distribution, run twice after BibTeX:

```bash
pdflatex PAPER_SUBMISSION_DRAFT.tex
bibtex PAPER_SUBMISSION_DRAFT
pdflatex PAPER_SUBMISSION_DRAFT.tex
pdflatex PAPER_SUBMISSION_DRAFT.tex
```

The actual local build used portable Tectonic0.15.0:

```bash
/home/jian/livox_ws/p62_r27_typesetting_assets/tectonic -k --keep-logs --outdir build PAPER_SUBMISSION_DRAFT.tex
/home/jian/livox_ws/p62_r27_typesetting_assets/tectonic -k --keep-logs --outdir build SUPPLEMENTARY_MATERIAL.tex
```

Copy the resulting main PDF to `PAPER_SUBMISSION_PDF.pdf`; the build helper performs this automatically. `IEEEtran.cls` and `IEEEtran.bst` are unmodified CTAN1.8b files with their original license notices. Required TeX fonts/packages are standard. The local Tectonic executable/cache is outsideGit and is not required if standard TeX is installed. No system-level TeX or ROS installation was changed.

The journal's interactive selector and legacy jsenzip returned418/403. This package uses the standard IEEE journal class, not a claimed byte-identical Sensors-specific color template. Check portal-specific front-matter styling after authors approve submission.

## Review priorities

Approve the narrowed sensing-system contribution and whether the existing short-window evidence suffices. Check the material limits before changing any claim: map-anchored reference, conditional Base axes,2initial large rotation errors, exposed development window, no fair external rank, and uncompleted robot calibration/deployment. Author names, affiliations, funding, corresponding author, release URL and exact AI disclosure remain explicit placeholders.

No new NDT, ROS, map preparation, optimizer, data download, external baseline run or R28 was performed. This directory is a publication package, not a new research experiment. Source/runtime identities of the underlying experiments remain in their sealed archives.
