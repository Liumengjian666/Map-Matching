# R5 verification receipt

Workspace: `/tmp/dog_loc_paper_r4_ws.Fq21k2`.
Scientific inputs: committed R4 artifacts at
`06ff017831d2654a62c72f854926b9d3b459ce40`.

- Release configure/build: PASS; `release_configure.log`, `release_build.log`.
  Existing optional VTK/pcap/png/libusb discovery warnings did not fail the build.
- P9 CTest: 41/41 PASS, including new full263 admission self-test; `p9_tests.log`.
- New self-test covers all 263 rows, inclusive score boundary, nonconverged
  rejection, converged iteration-limit retention, and representative suppression.
- Actual B12 complete-link cluster and strict count parity: 96/96 PASS.
- Input hashes: all 14 pinned R4 artifacts PASS. Frozen helpers are checked
  against the input manifest and starting Git commit.
- Frozen chart library after Release build remains
  `2a71c945f5b23b1eacdf4f73a63aba86fd509d2c556392652a0eda7c723cb711`.
- CSV/JSON/report and complete artifact inventory hash audit: PASS using
  `summarize_r5_attribution.py write` followed by its built-in audit.
- Terminal inventory: 26,400 unique method/frame/rank rows; 96 x (263 + 12).
- The original R4 archive and solver/frontend source files remain unchanged.
- NEW_NDT_CALLS=0; BASELINE_REPLAY_CALLS=0; VISUAL_EXTRACTION=0; GT_LOADED=NO.

## Independent review

A read-only review checked the frozen admission reuse and then independently
recomputed the resulting counts. No required findings remain. It reproduced
B12 17/40 and 3/56, FULL263 30/40 and 5/56, the exclusive failure partitions,
and the exact 47 archived ID-hit rows for the 17 disagreement frames.

It checked TX3829 specifically: ID-hit ranks3/8 fail score, whereas a different
raw terminal, rank10, passes geometry and score but its representative rank5
is inside the nominal center neighborhood. Frame-level and ID-hit-level reasons
are therefore distinct. The report now generates its next-step text from NEXT
instead of prescribing a different hardcoded next step.

Current independent review only; no external-model CLI was invoked.

## Audit ordering incident and resolution

The first summary audit failed closed with `artifact inventory incomplete`.
The summary had been launched concurrently with Release configure/build, which
created verification log files while the artifact inventory was being frozen.
This was an archive scheduling error, not a scientific input/result mismatch.

Resolution: wait for configure/build and CTest logging to terminate, then create
the final hash manifest serially. The same audit subsequently passed. The
inventory equality check is retained and guards against adding an untracked
artifact to a frozen manifest; no mismatch is suppressed or allowed. Subsequent
receipts are added before the final serial manifest refresh. Analysis CSV hashes
from the original analysis freeze remain unchanged.

## Reproduction

Use the existing Release chart library and frozen input files, then:

```sh
env PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 LD_LIBRARY_PATH=/lib/x86_64-linux-gnu python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/p9_r5_attribution.py self-test
env PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 LD_LIBRARY_PATH=/lib/x86_64-linux-gnu python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/summarize_r5_attribution.py audit
```

Do not invoke `run` against the already frozen output directory: it intentionally
refuses to overwrite the analysis freeze. No scientific rerun is needed to audit.
