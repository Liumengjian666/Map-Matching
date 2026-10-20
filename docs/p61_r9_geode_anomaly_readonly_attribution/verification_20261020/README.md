# R9 repeated-request verification — 2026-10-20

The requested R9 analysis already exists at local and queried remote commit
`d1f9fa9464dac835598e72d52ea2484fd9aa801c` on
`research/p61-r9-geode-anomaly-readonly-attribution`; its worktree was clean.
The original analysis started at R8 archive
`eba814000a06a2b110acf3835247cd6cffe9db6b`, with numerical analysis source
`f1d4ad321b6c77d7305ed42f015f95fc77c59e1f`.

This receipt directory preserves the completed analysis rather than resetting,
recreating the branch, overwriting old reports, or re-running an experiment.
Only repeat frozen-asset hashing, original archive checksum verification, and
existing pure-math/evidence tests. Write all new receipts here using new paths.
Do not run analyze_frozen.py, inspect raw sensor messages, or regenerate any
old derived output. No GT, ROS node, bag playback, NDT, EKF, Hessian/jet, pose
modification, source reconstruction, optimization or parameter changes.

Each full asset check is bounded by a 600-second command timeout and uses the
already archived asset_gate.py, which imports no ROS and hashes executables
without executing them. On any asset mismatch, stop and preserve the failure.

The old report's NEXT is a historical recommendation, not a new authorization.
The conversation already records subsequent R10–R12 work; this repeated R9
request does not authorize repeating those tasks or modifying their conclusions.
The unique next action after verification is scientific-controller acceptance
of the existing bounded attribution, using the already available later-stage
evidence when deciding a new contract. No next-stage experiment starts here.
