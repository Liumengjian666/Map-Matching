# Optional external CLI review — executed once with confirmation

The user requested Codex CLI and then confirmed this exact command and input
SHA256. CLI version `0.159.2` was checked before invocation. Input contained
only the download/extraction scripts, tests, explicit public asset plan and
review contract; no email address, credentials or dataset payloads.

Input SHA256:
`788cd6f0719f14fba91be5ca38d5c5a0732dbf3be96883f8e068f8bf30063d43`.

```bash
timeout --kill-after=10s 300s codex exec --sandbox read-only --ephemeral --skip-git-repo-check --cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p40_download_review.HmFOnb --model gpt-6.1-sol --config 'model_reasoning_effort="xhigh"' - < /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p40_download_review.HmFOnb/review_input.md > /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p40_download_review.HmFOnb/review_output.log 2>&1
```

Exit code: **0**. Header confirmed `gpt-6.1-sol`, `xhigh`, `read-only`.
Invocation count: **1**; no retry. This was an independent CLI session, not a
claim of a different model architecture. Local raw input/output remain in that
untracked cache. Only this small factual summary is committed.

## Findings reconciled against the artifact

1. **P1 — valid/actionable, fixed.** On restart, the downloader created fresh
   receipt state and accepted newly computed hashes of existing same-size
   finals. It could therefore lose the previous corruption-detection baseline.
   The delivered tool now loads and retains previous receipts, compares the
   file ID, byte size and SHA256, and stops on mismatch without overwriting the
   payload or its earlier hash. A tiny isolated test first failed with
   `RuntimeError not raised`, then passed after the fix. Final suite: **9/9**.
   This fix did not re-download, re-extract or change any dataset assets.

2. **P2 — valid documented trade-off.** The extractor writes into its final
   directory and refuses an existing extracted directory, so interruption
   requires explicit recovery rather than automatic resume/merge. The tool is
   deliberately fail-stop to preserve existing data; this task did not promise
   resumable extraction. Download resume is supported. No interruption happened
   in the successful extraction: all ten archives exited successfully with CRC
   checks, all 15,302 headers were inspected, and final output was flushed with
   `sync -f`. No data was deleted or relocated to silence this finding.

The post-review fix is at
`d71ebef7457835a52daa6d3b810ecbe05cfe32c1`.
It was not externally re-reviewed; the failing-to-passing local regression and
the full nine-test suite are the validation evidence. No further review call
or dataset/algorithm experiment was started.
