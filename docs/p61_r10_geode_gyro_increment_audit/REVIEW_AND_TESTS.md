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
