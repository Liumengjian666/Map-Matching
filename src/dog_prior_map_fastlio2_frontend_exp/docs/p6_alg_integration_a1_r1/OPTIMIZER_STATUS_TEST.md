# Optimizer status and state transaction test

The optimizer now has four terminal outcomes in addition to `NOT_RUN`:

* `ACCEPTED_UPDATE`: at least one objective-decreasing step was committed.
* `CONVERGED_WITHOUT_STEP`: no step was needed because the gradient infinity
  norm was already below the configured tolerance.
* `FAILED_ALL_CANDIDATES`: every attempted candidate was rejected; return is
  false and no feedback is available.
* `INVALID_LINEAR_SYSTEM`: objective linearization, factorization, step, or
  state application was invalid; return is false.

The regression includes:

1. an exact zero-gradient prior returning true with
   `CONVERGED_WITHOUT_STEP`;
2. a nonzero-gradient case with maximum step norm zero, returning false with
   `FAILED_ALL_CANDIDATES` and unchanged state;
3. an injected factor relinearization failure returning false with
   `INVALID_LINEAR_SYSTEM`;
4. a two-state stacked increment where the first state is updated and the
   second overflows. `applyGlobalIncrementAtomically` restores both states,
   proving that a prefix update cannot escape.

The previous fallback `optimizer_success || optimizer_iterations > 0` has
been removed. Executing an iteration is no longer reported as success.
