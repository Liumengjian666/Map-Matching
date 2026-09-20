# Target architecture (minimal, low-compute)

The target is a few explicit boundaries, not a framework. Keep ROS 1 adapters thin and keep algorithms independent of `ros::NodeHandle`.

```text
src/dog_prior_map_localization/
├─ include/dog_prior_map_localization/
│  ├─ nodes/
│  │  ├─ ndt_node.hpp                 # ROS adapter + small orchestration state
│  │  └─ ekf_node.hpp                 # ROS adapter + callback scheduling
│  ├─ core/
│  │  ├─ state_types.hpp              # State15, snapshots, timestamped measurements
│  │  ├─ imu_propagator.hpp            # pure propagation/covariance API
│  │  └─ measurement_types.hpp         # NDT/visual measurements + validity
│  ├─ map/
│  │  ├─ prior_map.hpp                 # cloud ownership/KD-tree/local submap
│  │  └─ scan_preprocessor.hpp
│  ├─ ndt/
│  │  ├─ ndt_matcher.hpp               # alignment policy, no NodeHandle
│  │  └─ ndt_timing.hpp                # prediction/pending watermark policy
│  ├─ vision/
│  │  ├─ visual_frontend.hpp            # KLT/essential relative pose only
│  │  ├─ lidar_depth_associator.hpp     # later, metric depth association
│  │  ├─ metric_visual_estimator.hpp    # later, image+depth metric motion
│  │  └─ visual_measurement.hpp
│  ├─ fusion/
│  │  ├─ oosm_manager.hpp               # rollback/replay only
│  │  └─ measurement_manager.hpp       # gating/subspace policy, future Stage3B
│  ├─ diagnostics/
│  │  ├─ telemetry_sink.hpp
│  │  └─ csv_writers.hpp
│  └─ common/math_utils.hpp
├─ src/nodes/                         # ROS subscriptions/publications
├─ src/core/                          # deterministic pure algorithms
├─ src/map/
├─ src/ndt/
├─ src/vision/
├─ src/fusion/
├─ src/diagnostics/
└─ tools/offline/                     # Schur/uncertainty/direction probes
```

## Ownership rules

1. `State15` is owned by an EKF state owner; only `ImuPropagator`, `OosmManager`, and a typed measurement-update API may write it.
2. `NdtMatcher` owns no ROS objects and returns `NdtMeasurement` plus `NdtDiagnostics`; it must not directly write EKF state or velocity.
3. `NdtTiming` owns prediction-history/pending-queue semantics and returns a deterministic `InitialGuess` record.
4. `PriorMap` owns map cloud/KD-tree and provides local targets; no ROS publication.
5. `VisualFrontend` returns a relative-pose result and quality/covariance metadata; it cannot call EKF update.
6. `MeasurementManager` is the only future entry point that decides whether/how a measurement enters EKF. In normal mode it can pass full NDT; in future directional mode it can apply projectors. It must receive explicit data, not read diagnostic globals.
7. `TelemetrySink` consumes immutable snapshots. Turning telemetry on/off must not change an algorithm branch except by documented resource cost.

## Deliberately omitted abstractions

No plugin framework, factory hierarchy, global singleton, service locator, or inheritance tree is needed. One concrete class per boundary is sufficient.
