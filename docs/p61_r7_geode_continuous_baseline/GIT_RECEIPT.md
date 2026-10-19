# Git receipt semantics

- Repository: https://github.com/Liumengjian666/Map-Matching
- Branch: research/p61-r7-geode-continuous-baseline
- START_SHA: c649c9fa1743aada3301ddb4f75bce54eaf08532
- Actual experiment CODE_SHA: 17922f2b63dae1f2af34f353287728d50a312f73
- END_SHA: final archival commit; exact full value reported by execution AI
  after committing and in internal `p61_r7_geode_runs/final_git_receipt.json`.
  A commit cannot contain its own resulting SHA; no misleading self-hash is
  embedded here. Obtain the receipt commit with `git log -1 --format=%H -- docs/p61_r7_geode_continuous_baseline/REPORT.md`.
- REMOTE_HEAD for the R7 branch: absent at final read-only query.
- R6 remote parent: c649c9fa1743aada3301ddb4f75bce54eaf08532.
- PUSH_EXECUTED: false. Local commits are not remote publication.

No global identity was set. Per-command commit author is the user's previously
provided real name/email. No large data/binary is staged, no force push.

After separately authorizing publication, use the actual R7 repository:

```
git -C /home/jian/livox_ws/p61_r7_geode_baseline_ws push -u origin research/p61-r7-geode-continuous-baseline
```

The protected dog_loc_paper_ws is a different repository and does not hold
this research branch. Do not create/modify refs there to disguise that fact.
