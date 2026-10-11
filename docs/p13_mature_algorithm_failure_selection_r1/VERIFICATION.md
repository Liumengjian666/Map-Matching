# Bounded P13 verification

- New behavior: descriptive analysis of already committed artifacts only.
  No NDT/DCReg/SOLiD/EKF/OOSM or build configuration modified.
- Run `python3 docs/p13_mature_algorithm_failure_selection_r1/analyze_existing.py`
  from the repository root to reproduce metrics, two evidence CSVs and input
  hashes. Two successive runs produced identical hashes for all four outputs.
- Built-in checks: percentile interpolation, singleton, empty/NaN/Inf
  rejection, archival row counts, finite/nonnegative timing, original far
  hypothesis count30, range-only cosine pair count2, representative case IDs.
- All JSON parsed; CSV rows are complete; each archived input SHA256 verified.
  Original input files and all prior archives remain unmodified.
- Code-quality review: single standalone offline script, standard library
  only, deterministic output, no shell interpolation, no runtime dependency
  or source changes, bounded389-reference pair loop. Nonfinite statistics fail
  explicitly instead of manufacturing numbers. Assertions are archival
  consistency checks, not online scientific admission thresholds.
- Fresh-context bounded adversarial read-only review completed. No finding
  crossing the scientific-judgment/reproducibility correctness threshold.
  External model CLI NOT_RUN. Optional second opinion offered this turn;
  no invocation without a separately confirmed command/input.
- C++ Release build / registration tests / ROS tests: NOT_RUN, since this
  analysis-only task changes no compiled sources or runtime configuration.
- `git diff --check`: PASS. Only the new P13 archive is staged for commit.
  Preexisting untracked directories retained. Main paper worktree remains
  dirty at its old SHA; stable robot workspace untouched.
- Sources: input_hashes.json identifies archive inputs; artifact_hashes.json
  covers new P13 files except itself. No large raw data/cache is committed.

Review constrains interpretation, not the chosen scientific direction:
nonconvergence cost is not proven waste, relative GT drift is not absolute map
truth or proof of multiple match basins, and mature reranking/verification
must be tested before claiming a new method is needed.
