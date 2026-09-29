# Atomic state commit test

`initializeWithPriorAtomic()` validates the state and PSD prior before modifying
the window. A rejected prior leaves an empty window and a corrected retry works.

`addStateWithImuFactorAtomic()` prevalidates timestamps, state, factor validity,
SPD whitening covariance, observation ID availability, and prior dimensions.
Only then does it extend the prior, append the state and IMU factor, register the
ID, and advance the window revision once. It avoids copying the full fixed-lag
Hessian on the normal event path.

The regression injects a singular covariance and verifies unchanged state count,
factor count, and window revision. The observation ID is not consumed.
