# One authorized post-run Codex CLI review

- Tool: `/usr/bin/codex`, version0.159.2; model `gpt-6.1-sol`, reasoning `xhigh`.
- Ephemeral/read-only, existing login, one confirmed invocation, exit0.
- The asynchronous selection arrived **after** the single formal run completed.
  This was not a pre-run gate and did not cause any new input, align or replay.
- Package SHA: `870d760678bcbbe82a0caf257b3b711b00643204f59d7033595686f6ff8534d9`.
- Prompt SHA: `966f544fdd52c747921135379e888d6e58887ce5c2af8d0b9c1302bbb45938dd`.
- Full log, internal only: `/home/jian/livox_ws/p61_r8_geode_runs/cli_review.log`.
- Log SHA: `e87beb97a3dac97da3a2a7ea2c9b11e021bcf6af95107c875ec0d22a5d3e2279`.

Exact confirmed command:

```bash
codex exec -m gpt-6.1-sol -c 'model_reasoning_effort="xhigh"' --sandbox read-only --ephemeral -C /home/jian/livox_ws/p61_r8_geode_full_return_ws - < /home/jian/livox_ws/p61_r8_geode_full_return_ws/docs/p61_r8_geode_full_return_control/cli_review_prompt.md > /home/jian/livox_ws/p61_r8_geode_runs/cli_review.log 2>&1
```

## Findings and reconciliation

1. Required: `analyze_run` presence-only `evidence_complete` did not itself require
   semantic agreement. The bounded material showed no actual conflict, but mere
   existence was not a sufficient generic certification gate.
   **Closed in post-run analysis only:** require same-frame successful Deskew,
   convergence, applied OOSM/fusion, exact native correction time, current replay
   watermark, corrected delivery order, real event order and state revision.
   Recompute statistics from the same frozen outputs; no new localization run.
   A newly strengthened check initially double-rounded integer epoch ns through
   `ns/1e9`, producing652 false flags. Root cause was reproduced independently:
   integer rational→nearest double gives zero mismatches, as the CLI's own numeric
   check also showed. A regression test now guards this; the false-checker snapshot
   remains internal at `rounding_checker_false_negative.tar.gz`.

2. Required: actual runtime recorder treated visible OOSM APPLIED with not-yet
   visible matching lineage as immediate TIME_CONTRACT_BLOCKED. This is a possible
   false abort caused by asynchronous logging, not proof of an illegal update.
   No such abort occurred in this run. **Closed as an archive-only tool patch:**
   missing/stale recorder evidence now waits to the existing deadline; a matching
   correction at the wrong time still fails. Four focused helper cases cover
   missing event, previous event, valid matching event and actual early correction.
   No estimator or numerical parameter changed, and no formal replay was repeated.

The reviewer found no package-supported contradiction of the conservative
WITH_FAILURES wording. Package scope was scripts, derived rows and summary; it
could not independently certify all original raw messages or the C++ process
identity. We do not claim it did. Independent local raw-hash/event-order checks
are separately recorded in CODE_AND_TESTS.md and verify_archive.py.

## Runtime versus archival code

The actual recorder remains exactly identifiable at
`1367e1d9587d1f90a47bf6f44405239a7b65e4f6`; use `git show` at that commit to inspect
its original replay_full.py/terminal_contract.py. The final archive additionally
contains the post-run tool guard above. **It did not participate in the run.**
Actual NDT/EKF C++ binary identities and all original raw output hashes are
unchanged. Pre-review derived CSVs are retained under `p61_r8_geode_runs/pre_cli_derived`.
Full pre-review archive snapshot SHA:
`e83837632e2932a223fe25f966ef7a21b548d441ccd68157dcbcde6504f479f7`.
The approved immutable package is also retained; neither was overwritten.
