# R28 blocked validation delivery

Start with [FINAL_RESEARCH_DECISION.md](FINAL_RESEARCH_DECISION.md) and [FRAME_CONTRACT.md](FRAME_CONTRACT.md).

- Editable complete review draft: UPDATED_MANUSCRIPT.tex.
- Eight-page PDF: UPDATED_MANUSCRIPT.pdf.
- Exact raw candidates, not executable experiment admission: WINDOW_CANDIDATES.json.
- Actual source/import checks: sources/manifest.json and logs/.
- No new localization or external baseline results were generated.

Read-only contract check, from the R28 worktree root:

```bash
python3 docs/p62_r28_independent_validation/tools/check_contract.py
```

It refuses to overwrite differing existing evidence; a later changed production snapshot will fail, not rewrite the original receipt. It does not run NDT or decode reference poses.

Document generation and actual PDF build, from the archive directory:

```bash
python3 tools/update_manuscript.py
/home/jian/livox_ws/p62_r27_typesetting_assets/tectonic -k --keep-logs --outdir build UPDATED_MANUSCRIPT.tex
```

The final root PDF is copied from build/UPDATED_MANUSCRIPT.pdf. BUILD_RECEIPT.json records the verified delivered identity; retypesetting later may change PDF metadata and requires new verification. R27 originals remain unmodified. Fonts are embedded; existing Tectonic/font-substitution/underfull warnings are retained in logs rather than called a warning-free build.

Local audit code commit: 9e5d41528c12228b613bce79b31b95fcc159c813. Final delivery END is the local branch HEAD reported at handoff. No push or journal submission was performed.
