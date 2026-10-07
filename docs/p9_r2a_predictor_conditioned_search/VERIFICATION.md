# Verification and review record

## Frozen execution

Branch: `research/p9-r2a-predictor-conditioned-search`.
Start/execution SHA: `cfd76389688c72f181b294c316c5cd1911359ecb`.
The commit containing this record is the final handoff commit; its SHA is reported in the handoff rather than embedded self-referentially in its own tracked content.

Release build: `/tmp/p9_r2_build`, `CMAKE_BUILD_TYPE=Release`, PCL1.10 system package.
Binary SHA256: `3dd2d4f48c86aa9feb55c0ea6efe7128fa90c6c5a3c6c4c4364cc230cfd56db6`.
Immutable `execution_manifest.json` records the full map, source, header, proposal, execution-code, RNG, binary and result hashes.

Exactly2560 new NDT calls: WEAK512, STRONG512, RANDOM1536. Each frame/method/replicate has one16-call nested sequence. Historical FULL6D and old WEAK results are reused; no new FULL6D calls or later search extensions are performed.

## Verification commands

```
cmake --build /tmp/p9_r2_build -j2
env LD_LIBRARY_PATH=/lib/x86_64-linux-gnu ctest --output-on-failure -j2
python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/summarize_r2a_predictor_search.py audit
git diff --check
```

CTest is run with working directory `/tmp/p9_r2_build`. Release build and22/22 P9 tests pass. Explicit system-library selection prevents the unrelated machine `/opt/MVS` libusb search path from affecting diagnostics.

The audit verifies frozen input/source/binary hashes; all42080 proposal formulas, complementary-coordinate preservation and reconstructed poses; all160 FPS orders;2560 selected/actual start identities; predictor and chart parity; raw-terminal admissions and convergence filtering; frame-macro/22-ID micro recall; common-prefix AUCs/random aggregates; historical H2 comparisons and each discovery gate; correct minimum-energy objective choices; unchanged choices after GT loading; post-hoc counts; no-major clustering/status summaries; CSV/JSON denominators and artifact hashes. No audit invokes an optimizer.

## Independent review and corrective action

Two fresh-context read-only reviewers examined proposal/execution provenance and evaluation/statistics separately. Their reviews do not modify files or run NDT. No external Codex/Gemini CLI is invoked; the user selected the current independent review only.

The evaluation reviewer found a substantive score-sign defect: positive PCL raw score was treated as a minimizing objective. The regression test reproduces the failure before the fix, then passes with `E=-S`. `OBJECTIVE_SIGN_CORRECTION.md` documents the correction; the original hashed pre-run theory is preserved. Only offline selection and GT/safety summaries are recomputed. This correction never changes proposals, NDT results, admissions, recall, AUC or the discovery gate.

Further review led to explicit reporting of random-replicate ranges, current-versus-historical common budget intervals, exact-prefix versus tested-checkpoint costs, same-index conditioning controls, and nominal quaternion-carrier sensitivity. Final review found no further actionable contract violation.

## Limitations kept in the handoff

- The prescribed discovery gate passes first atB12, but same-index old WEAK AUC0.479157 exceeds conditioned WEAK0.399270. Conditioning-only aggregate efficiency is not established.
- PRIMARY2/3 consists of two newly recovered2350IDs;2226/P05 remains unrecovered. Frozen overlapping-ID admissions are not a certificate of distinct dynamic minima.
- Historical FULL70 cost32 is the median first tested checkpoint. Exact-prefix median is18;32/12 is not an exact matched-recall speedup.
- Corrected minimum-energy WEAK selection atB12 has major GT improved/same/worse3/0/6. Two no-major frames leave nominal and both become worse. Discovery support does not authorize an objective-only pose selector.
- Float nominal quaternion-carrier sensitivity affects tiny score gaps and exact tie identity. The audit covers23 byte-identical archived nominal-pose comparisons, not all32; no materially separated selected pose changes in those comparisons.
- Mean additional WEAK B12 alignment time is738ms/frame: not yet online-competitive. No EKF, covariance fusion, support search, visual subsystem or stable workspace is changed.

Only this task's paths are explicitly staged. Existing `.vscode/`, `Testing/` and anomalously named untracked files remain untouched and uncommitted. No push is executed.
