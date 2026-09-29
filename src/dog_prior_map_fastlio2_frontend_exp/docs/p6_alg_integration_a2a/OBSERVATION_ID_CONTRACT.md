# Observation ID contract

The adapter owns one monotonically increasing internal stream beginning at 1.
An ID is assigned to an IMU interval, LiDAR factor, or visual factor only after
all source, causality, timestamp, covariance and endpoint checks pass and the
controller accepts the factor.

Raw NDT transaction IDs and visual timestamp pairs are not passed through as
window IDs.  Source records are checked before allocation, so a duplicate
transaction or visual pair cannot bypass de-duplication by receiving a new
internal ID.  The window continues to enforce its bounded active-ID set and
retired watermark; the adapter's source ledger separately prevents old source
re-entry after a state is marginalized.

The deterministic A2A test checks the sequence of IDs across IMU, LiDAR and
visual factors and verifies duplicate rejection without an ID increment.
