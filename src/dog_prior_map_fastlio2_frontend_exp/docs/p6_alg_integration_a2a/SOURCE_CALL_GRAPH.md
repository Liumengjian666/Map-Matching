# Source call graph

The adapter is intentionally below the formal runtime boundary:

```text
Frozen sensor products
  ├─ appendImu(ImuSample)
  │    └─ causal IMU buffer
  ├─ processLidarEvent(FrozenLidarEvent)
  │    ├─ source/timestamp/map-support checks
  │    ├─ ensureStateAt()
  │    │    ├─ preintegrateImu()
  │    │    ├─ FixedLagExperimentalController::addState()
  │    │    └─ addImuFactor()
  │    ├─ map_T_lidar × inverse(T_imu_lidar)
  │    ├─ residual-consistent basis callback (weak LocalRisk only)
  │    └─ addLidarFactor()
  └─ processVisualEvent(FrozenVisualEvent)
       ├─ source/timestamp checks
       ├─ assessVisualQuality()
       ├─ ensureStateAt(current timestamp)
       └─ addVisualFactor(reference,current)

optimizeCurrentWindow()
  └─ FixedLagExperimentalController::optimizeAndMarginalize()
       └─ existing FixedLagWindow optimizer + Schur marginalization

latestOptimizedState()
  └─ predictionFeedbackSeed()
```

`p6_i1_branched_recovery.cpp` remains a frozen producer-side source of NDT
terminals and visual records; A2A does not call it or invoke NDT.  The adapter
is compiled into the standalone experimental library only.  The formal FULL
runtime target is unchanged.
