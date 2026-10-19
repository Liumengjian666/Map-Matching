# Incremental review and regression evidence

Initial9 synthetic tests passed. Fresh adversarial review then identified:
nonfinite median could bypass gaps; noninteger endpoint acceptance; an extra
in-window timestamp condition inconsistent with a valid short causal head hold;
independent near-pi log check missing; classifier allowed negative residuals.

All are actionable numeric-contract guards, not a changed scientific threshold.
We added regression cases **before** repairing the implementation. The real
test run produced11tests:2failures/1error:

```
test_formal_thresholds FAIL:
  classify([-1]*11, valid11) returned LIMITED_ANGULAR_CONSISTENCY_SUPPORTED
  rather than NOT_EVALUABLE.
test_nonfinite_period_and_noninteger_endpoints FAIL:
  ValueError not raised for NaN period.
test_short_interval_real_head_hold ERROR:
  ValueError NO_IN_WINDOW_SAMPLE for a bounded valid head hold.
Ran11 tests; FAILED(failures=2, errors=1).
```

Root causes were localized to integrate_zoh entry guards and classifier input
validation. Repaired integer endpoints, finite positive median and nonnegative
residual checks. Removed the extra timestamp-presence rule: fresh true head
measurements still supply gyro over short intervals within the same gap bound.
No invalid gap is bridged. Near0/nearpi/pi angles are independently checked using
the Frobenius norm of scipy.linalg.logm /sqrt(2), including the complex principal
matrix log at exactpi; the angle remains the principal rotation magnitude.

No real gyro residual or formal scientific decision was calculated while these
tests failed. Further execution resumes only after the complete suite passes.

Second bounded local review found two additional application-boundary issues:
run_audit rejected the legitimate complex principal matrixlog branch at exactpi,
and its JSON reader cast timestamps to int before validating their type. Fixed
by using the same independently justified principal-log norm at the actual
application call site, and rejecting noninteger raw sample stamps before any
cast. Regressions test production helper paths, including boolean/string/float
stamps.15 tests now pass. No scientific threshold, interval or bias changed.

User requested a Codex CLI second opinion. The bounded final input archive and
precise read-only command were supplied for separate confirmation. Review is
limited to packaged source/contract/tests, not real sensor assets or execution.

The user confirmed the exact one-shot CLI command. It completed exit0 with
gpt-6.1-sol/xhigh, ephemeral/read-only, after matching package SHA
c637106c7485d5e4784d9c8aed484d0b11e84745f07a6209521390b605e3e60c.
Two Required findings were valid evidence-binding gaps, not findings of actual
data tampering or an actual shifted measurement:

1. Altering a finite gyro value in the decoded JSON while retaining its metadata
   could previously pass run_audit. Fixed by preserving original serialized IMU
   bytes for the same111 messages and checking each payload against the immutable
   R8 ledger SHA. A pure-Python sensor_msgs/Imu layout decoder verifies header,
   frame, sequence and the exact gyro double values against those verified bytes
   before any integration. Synthetic finite-value/payload tampering is rejected.
2. The original freeze checked current scans5155–5165 but not precursor5154's
   measurement time. run_audit and new freeze calls now independently compare
   all12 raw pose measurement times to exact integer scan ends. A synthetic
   shifted5154 counterexample is rejected; the real audit must separately pass.

The original immutable input pin and decoded gyro JSON are preserved; the new
payload binding is an additive artifact. Runtime threshold/bias/pose/interval
semantics are unchanged. No second CLI invocation or fourth review cycle is
authorized or performed. The final calculation uses a NEW committed code SHA,
not the earlier package's source identity. The external review is a static code
second opinion, not an independent validation of sensor values or results.
