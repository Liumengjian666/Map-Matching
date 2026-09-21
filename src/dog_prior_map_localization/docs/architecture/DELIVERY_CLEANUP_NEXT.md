# Delivery cleanup: next architecture proposals

This document records follow-up design only. No item below is executed by
`DELIVERY-CLEANUP-1`.

## Priority A: extract NDT observation/OOSM correction

Move the NDT observation and OOSM correction implementation out of
`vision_observation.cpp` into a focused module such as:

```text
fusion/ndt_observation.cpp
fusion/ndt_observation.hpp
```

The module should own measurement timestamp handling, bounded future
deferral, rollback/replay invocation, and the corrected-state bookkeeping. The
vision file should retain only image/feature processing and its explicitly
gated observation interface.

## Priority B: narrow the delivery EKF boundary

Separate the delivery path into explicit owners for:

```text
IMU propagation
NDT observation/OOSM
state history and replay planning
ROS output/TF
```

The integrated LiDAR matcher, map loader, visual frontend, and directional
fusion state should become optional modules behind clear interfaces. This is a
structural refactor only; each extraction must preserve the split runtime
topics, timestamps, and OOSM contract before any algorithm change is attempted.

## Priority C: split the NDT monolith

Decompose `dog_prior_map_ndt_node.cpp` by ownership into:

```text
nodes/ndt_node
ndt/ndt_localizer
ndt/scan_preprocessor
ndt/map_manager
```

The node layer should retain ROS queues and publication, while map loading,
scan timing/preprocessing, NDT alignment, and diagnostics each receive a
focused interface. Existing NDT parameters and diagnostic CSV fields must be
kept stable during that refactor.

## Guardrails for the next stage

- Preserve the canonical split launch and all topic names.
- Keep visual, directional-fusion, Schur, and uncertainty code available for
  research; do not delete it as part of delivery cleanup.
- Compare cloud hashes, NDT initial/raw/final poses, OOSM results, and TF
  warnings after every extraction.
