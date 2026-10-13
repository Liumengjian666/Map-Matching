# Bounded review

One independent fresh-context read-only review of runtime/math found two
actionable interface issues before replay: diagnostic output was mandatory;
binary search assumed sorted raw IMU even though the old receiver appends before
rejecting nonmonotonic samples. Both were corrected, with timestamp regression
tests. No remaining contract-breaking issue found. This was a finite static
review, not a new replay, full numerical audit or scientific certification.

User requested an additional external CLI and confirmed the exact command and
immutable input SHA256:
`1e27be61ffb7c37adbcc31f6c47237d8470b421ee682a9516670d603e7d994e6`.
35,642 bytes, scoped source/contract/tests only; no bag, raw image/point cloud,
GT or credentials supplied. One Codex CLI0.159.2 invocation, gpt-6.1-sol/xhigh,
read-only and ephemeral, exited0: `No substantive issue.` No tool calls or
second invocation. This is an independent external session, not a claim of a
different architecture. It did not review final GT metrics. Exact input/result
and command are preserved in external_review/.
The exact prompt contains literal git-diff blank context markers. A local
.gitattributes whitespace exception applies only to this immutable input;
no normalization changes its authorized SHA. All other archive/source files
retain the normal git diff --check gate.

Final local artifact tests independently recompute all nine EKF window metrics
from per-sample errors, deltas/percentages, source hashes and frozen outputs;
due NDT and due visual releases, root validity, clock/parameter contracts and
timing source/counts checked. The normalized diagnostic-rotation regression
does not alter GT errors, alignment or frozen replay outputs.
