# Root-cause status

**Classification: `A3G-R2-F ROOT_CAUSE_NOT_ISOLATED`.**

The data gate establishes that the frozen raw input identities and A3G diagnostic artifacts are intact. It does not establish whether the prediction had left the prior-map overlap region, whether NDT moved a supported seed into an unsupported pose, whether geometric-observation validity rejected nearby map structure, or whether scan deformation was responsible.

The decisive step required by this task—strict point-level reconstruction of production deskew—was unavailable from the stored artifacts. As a result, no transformed-scan nearest-neighbor distribution, map-neighborhood count, production 0.8 m support count, or seed-versus-terminal counterfactual was calculated. The logged NDT and deskew summary fields are not substitutes for those geometric measurements.

No classification A–E is supported. In particular, a logged zero-iteration NDT result or a large logged deskew displacement statistic cannot by itself distinguish a poor prediction basin from absent map overlap, observation filtering, or actual point-cloud distortion.

## Missing evidence needed to resume

For each selected transaction, preserve either:

1. the exact deskewed source point cloud before preprocessing/NDT, plus its provenance and frame; or
2. enough state to reproduce it strictly: the exact 15D scan-start `WindowState`, the exact causal IMU sample interval, the exact calibration/noise/gravity values, and a point-level reference to check the reconstruction.

For exact support reproduction, also preserve the exact preprocessed source cloud used by `geometricObservations()` (or the exact deterministic preprocessing inputs and configuration) and the corresponding target-map voxel representation/configuration. Pose summaries and raw point timestamps alone are insufficient.
