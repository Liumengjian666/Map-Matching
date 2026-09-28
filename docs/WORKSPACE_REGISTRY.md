# Workspace Registry

Read-only snapshot recorded for PAPER-P6-I6C on 2026-09-29. P6 repository worktrees are from `git worktree list --porcelain` in `/home/jian/livox_ws/dog_loc_p6_i6b_ws`. No listed worktree was deleted or modified by cleanup. The separate `dog_visual_loc_ws` row was queried independently; it is a different repository, not a worktree of the P6 Map-Matching repository.

| Directory | Branch | HEAD | Relationship / handling |
|---|---|---|---|
| `/home/jian/livox_ws/dog_loc_paper_ws` | `review/paper-p6-i3-uobs-ndt-20260928` | `3e416e136c0ae7541a9190847100705b33f59a43` | Existing P6 repo worktree; untouched |
| `/home/jian/livox_ws/dog_loc_p6_i5a_audit_ws` | `research/p6-i5a-semantic-audit` | `0a8d6fe6283b421813dd3c2eac4dbb46f6befceb` | Existing P6 repo worktree; untouched |
| `/home/jian/livox_ws/dog_loc_p6_i5b_local_null_ws` | `research/p6-i5b-local-response-null` | `edbe47ed68c17a5b44dc2b4d8cb0713abea8d97c` | Existing P6 repo worktree; untouched |
| `/home/jian/livox_ws/dog_loc_p6_i5c_ws` | `research/p6-i5c-stationary-audit` | `a24dd5bb65264374734275903f7075d4367b3f63` | Existing P6 repo worktree; untouched |
| `/home/jian/livox_ws/dog_loc_p6_i6a_ws` | `research/p6-i6a-convergence-control` | `191fe2732539396a22a41c5dc9ff986281256d01` | Existing P6 repo worktree; untouched |
| `/home/jian/livox_ws/dog_loc_p6_i6b_ws` | `research/p6-i6c-framework-integration` | `ffe4cd82d28820f4abb74ae0990d9a97dfb51ba6` | Active I6C development worktree; registry snapshot predates the final report/index commit |
| `/home/jian/livox_ws/dog_loc_paper_ws_p6i4_20260928` | `paper` | `ec35ced556f0f1eba192b928bf07452543c5c0dc` | Existing P6 repo worktree; untouched |
| `/home/jian/livox_ws/dog_visual_loc_ws` | `feature/visual-factor-window` | `41999ea700c66c4cadf0eca9e0c5d73caa2783fd` | Separate repository; explicitly out of scope and untouched |

No worktree cleanup was performed. Future cleanup requires its own explicit scope and recovery check.
