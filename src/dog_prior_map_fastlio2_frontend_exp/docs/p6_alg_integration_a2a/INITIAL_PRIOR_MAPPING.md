# Initial prior and state mapping

`FixedLagEventAdapter::initialize()` accepts a 15x15 information matrix and a
15-vector gradient at the initial state.  The order is exactly:

```text
[right SO(3) rotation, position XYZ, velocity XYZ, gyro bias XYZ,
 accelerometer bias XYZ]
```

The adapter passes this prior once to `FixedLagWindow::setInitialPrior()`.
The initial state is the already initialized 15D window state.  No arbitrary
identity covariance is synthesized by the adapter, and no 23D IKFoM covariance
is compressed or expanded.

The A2A boundary deliberately does not call `setWindowPredictionSeed()`.
After successful window optimization, `latestOptimizedState()` exposes only the
latest 15D `WindowState` through the controller's revision-checked feedback
API.  A future integration must provide a mathematically audited mapping for
the remaining IKFoM blocks (including fixed gravity/extrinsic cross terms)
before claiming a full 23D posterior handoff.
