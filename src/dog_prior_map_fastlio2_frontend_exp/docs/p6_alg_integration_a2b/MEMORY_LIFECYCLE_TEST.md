# Memory lifecycle test

History pruning runs only after successful optimization/marginalization, when the
oldest retained state is known. IMU history keeps the last sample at or before
that state for interpolation and all later samples. Source records older than the
retained window are removed; they cannot re-enter because their endpoints are
already marginalized. LiDAR transactions additionally obey a monotonic scalar
watermark, so removing active keys cannot permit historical replay.

The deterministic 10,000-event test measured:

- peak IMU buffer: 301 samples;
- final IMU buffer: 300 samples;
- peak active source records: 8;
- final active source records: 6;
- expired source records removed: 192.

The existing fixed-lag observation-ID watermark remains responsible for factor-ID
replay rejection, and the Schur information-conservation regression still passes.
