# Necessary local verification and review

R26 changes literature/documents and small evidence tools only. No C++ estimator source, thresholds, launch, filter, map, original experiment or production file is changed. No estimator build or run is needed to validate a manuscript revision.

## Actual commands and artifacts

In `/home/jian/livox_ws/p62_r26_published_comparison_ws`:

```bash
python3 docs/p62_r26_published_comparison/tools/fetch_sources.py
python3 docs/p62_r26_published_comparison/tools/fetch_additional.py
```

These were bounded one-time public paper/README retrievals, not estimator/dataset downloads. Their per-source errors and exact bytes/hashes are retained in `SOURCE_FETCH_MANIFEST.json` and `SOURCE_FETCH_ADDITIONAL.json`. Earlier failed403/404/SSL requests are not described as successful PDF inspection. Available PDFs were converted with `pdftotext -layout input.pdf output.txt` in the external source directory. Small official documentation/result-directory listings and two pinned already-published TUM files were retrieved by read-only urllib requests; receipts are `DOCUMENTATION_FETCH.json`, `STEAM_*PATHS.json`, and `PUBLISHED_TRAJECTORY_INSPECTION.json`. No clone/build/run command was issued for any external method.

```bash
python3 docs/p62_r26_published_comparison/tools/inspect_public_trajectories.py
python3 docs/p62_r26_published_comparison/tools/build_evidence.py
python3 docs/p62_r26_published_comparison/tools/revise_paper.py
python3 docs/p62_r26_published_comparison/tools/validate_closure.py
```

The fetch scripts deliberately refuse existing destination files. They are provenance tools, not rerun prerequisites. Full papers and public trajectories remain outside Git; structured facts and SHA receipts are committed. The trajectory inspector reads PCD filenames only, not reference CSV values or scans for a new experiment. A local read-only import used for the stable snapshot is configured not to write Python bytecode into historical directories.

`FINAL_VALIDATION.json` and `logs/validation.json` record49 passed checks: exact table tokens on correct extracted PDF pages, eight complete23-field records, two exact15301time matches, unchanged selected R25 source documents/figures, complete manuscript sections and local figure links, tool syntax, frozen estimator diff and stable HEAD/index/status/protected-file identity. Source URLs/papers were inspected before manuscript generation; these checks do not automatically certify interpretations or novelty.

## Focused five-axis local review

- **Correctness:** separated author-run versus quoted values, original0.3606 versus later printed0.360, oldv2/Table6 versus currentv3/Table7, preprint versus formal publication year, and per-sequence versus dataset-average timing. No inferred rotational result or fabricated alignment command. Candidate-admission rule was not rewritten to treat tie-break0.8m/5deg scales as hard C limits.
- **Readability:** a full English paper accompanies three separate comparison tables and one metadata ledger. Detailed version/provenance/NR explanations reside in the extraction/audit; the manuscript retains the qualifications needed to interpret every table.
- **Architecture/scope:** helpers operate only on this archive or external literature directory. They do not import an estimator pipeline or generate source/initial guesses. R25 paper remains immutable; its figures are linked rather than silently regenerated.
- **Security/data handling:** public author/publisher/official repositories only. Failed publisher/API access is recorded; no permission bypass, credentials export, external CLI second opinion, automatic publication or push. Literature byte caps prevent accidental dataset retrieval.
- **Performance/resource interpretation:**0new native calls and0external reproductions. Published component-RAM/CPU reports are not compared by ratios with our process HWM/two-nodeRSS. Existing tests and historical binaries were not rerun just to validate prose.

## Review limits

No external GradeA result or journal novelty certificate is produced. Public output timestamps are verified, while their precise paper-run configuration and transformation into our conditional frame remain uncertified. Author v3 is the numeric inspection source for2Fast, not a byte-verified publisher PDF. Crossref returned429 for FAST-LIO2; IEEE publisher metadata supplied its formal journal identity instead. No retry campaign followed rate limits. Submission limitations are explicitly enumerated in `SUBMISSION_READINESS.md`.
