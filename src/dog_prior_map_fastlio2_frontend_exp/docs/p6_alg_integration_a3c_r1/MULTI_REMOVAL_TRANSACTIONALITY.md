# Multi-removal transactionality

For tx90, `marginalizeIfNeeded()` entered enforcement 78 and the first
`marginalizeOldest()` attempt failed. There were zero successful removals
earlier in that enforcement. Therefore:

`MULTI_REMOVAL_TRANSACTIONALITY = NOT_EXERCISED`.

The diagnostic failure capsule recorded identical before-enforcement and
after-failure state stamp lists, prior FNV-1a diagnostic hash, and active
factor counts:

```text
state count: 41 → 41
prior hash: 15827469298727093934 → 15827469298727093934
factors IMU/LiDAR/visual: 40/17/0 → 40/17/0
```

This proves there was no partial commit in this tx90 attempt. It does not
prove atomic rollback when an earlier removal succeeds and a later removal in
the same enforcement fails; that case was not exercised and remains an
independent engineering question. The 38 earlier successful removals belong
to prior enforcement calls and are not a partial commit of tx90.
