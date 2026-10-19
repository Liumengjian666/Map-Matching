# Bounded adversarial review record

Local fresh-context review inspected the state/frame/time boundary. Its
substantive finding was accepted: the legacy OOSM 20ms gate admits a
measurement but does not propagate the head snapshot to measurement time.
The single permitted time repair is documented in PROTOCOL and tested with
a constant-velocity counterexample plus the actual causal deskew kernel.
Other accepted findings: use LiDAR, not IMU, reference pose; keep the uint32
offset separate from output floating seconds; do not require corrected
messages' double-roundtripped epochs to be bit-identical to original ns.

At the user's explicit second confirmation, exactly one external command ran:

```
codex exec -m gpt-6.1-sol -c 'model_reasoning_effort="xhigh"' --sandbox read-only --ephemeral -C /home/jian/livox_ws/p61_r7_geode_baseline_ws - < /home/jian/livox_ws/p61_r7_geode_baseline_ws/docs/p61_r7_geode_continuous_baseline/cli_review_prompt.md
```

Input package SHA256: 26b1cae8fb0895bccfa958c509061cc487e91dca350be2c4feea5603c039b00a.
It completed with exit0. Full stdout/stderr is retained internally at
`/home/jian/livox_ws/p61_r7_geode_runs/cli_review.log`. No second CLI run.
One initial child-shell here-document read failed in its read-only sandbox;
the same invocation continued using read-only Python code and finished.

External Required findings, each **valid and actionable**, all resolved before
any R7 align:

1. First scan could lose an unlatched prediction before cross-topic TCPROS
   negotiation completed. A real getBusInfo connected-peer readiness barrier
   now covers the EKF/Deskew/prediction/NDT/corrected/diagnostics chain.
2. The source-selection ledger had no pinned hash. Pin and verify SHA256
   21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb.
3. zip(clouds,manifest) could stop at19 yet write scans_sent20. Pin manifest
   SHA dda2ac02550f6ee5f294261094ff0b42c5494c5c5b0b3c5bc6ca56ed2a880cb4,
   require20 indices, verify initialization YAML SHA and active ROS parameters,
   cross-check every cloud header/data hash, and record actual completion.

Post-package deltas were locally reviewed, not claimed externally reviewed:
the above guards; a passive diagnostic subscriber for performance CSV;
the existing optional NN-geometry diagnostic; unregister callbacks before
closing files under their lock, with an explicit closed-recording guard;
postprocessing/report scripts. These introduce no additional align/jet/value
or state estimator. Local fresh review stopped after three bounded cycles.
No unresolved Critical/Required finding is knowingly deferred.

Skill references `../../references/definition-of-done.md` and
`../../references/orchestration-patterns.md` are absent in this environment;
the available skill instructions plus explicit task gates were used.
