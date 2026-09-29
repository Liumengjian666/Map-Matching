# Observation ID lifecycle test

The unbounded historical `std::set` was replaced by:

* an active-ID set containing exactly IDs of active raw factors;
* an explicit active-ID capacity (default 4096);
* one retired high-watermark for the globally monotonic observation-ID stream.

When a raw factor is consumed by marginalization, its ID is erased from the
active set and advances the watermark. Any ID at or below the watermark is
rejected as `retired_measurement_id`, so clearing active memory cannot allow an
old IMU, LiDAR, or visual observation to re-enter. A fresh ID is also rejected
if the active capacity is reached.

The contract requires one globally monotonic ID stream across factor families.
This is necessary for a scalar watermark to provide bounded exact replay
rejection; arbitrary unordered historical IDs would require unbounded history.

The lifecycle regression executes 10,000 state/factor/marginalization cycles:

```text
maximum active LiDAR factors = 1
maximum active IDs = 1
retired historical ID replay = rejected
```

The separate repeated optimization test reached only five active IDs, exactly
equal to active IMU + LiDAR + visual factor counts. Its Schur `H,g` checks also
verify that retired raw factors are not duplicated in the dense prior.
