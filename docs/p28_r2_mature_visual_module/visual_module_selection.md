# Route and provenance

Chosen: independent OpenVINS-style FAST/KLT + feature triangulation + MSCKF
nullspace measurements, not a PnP pose wrapper. Corridor01 frozen PCD header is
`FIELDS x y z`, float32, 338210 points, binary. No reference images/patches or
visual-map association is supplied. Existing FAST-LIVO2 Log/image is empty;
other mapping point clouds/poses do not establish the frozen map's identity.
Therefore a fixed-map FAST-LIVO2 sparse-direct residual cannot be constructed
without inventing texture provenance. Only this feature route is implemented.

Read official sources and primary papers:

- [OpenVINS](https://github.com/rpng/open_vins), frozen local HEAD
  `69488123ed9362dd44b6f28e7f4680abbff1442b`: TrackKLT, FeatureInitializer,
  UpdaterMSCKF, UpdaterHelper, StateHelper. [ICRA paper](https://yangyulin.net/papers/2020_icra_ov.pdf).
- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2), HEAD
  `0d2c0346107b75b59934975adec9a6eeeb913c64`: vio.cpp/vio.h/visual_point;
  [paper](https://arxiv.org/html/2408.14035v1). True photometric residual requires reference patches.
- [R3LIVE](https://github.com/hku-mars/r3live), HEAD
  `6143a38537f28cb36eb24e9bbe2e39c8f7967157`: VIO/map tracking, reprojection and
  frame-to-map photometric residuals; [paper](https://arxiv.org/html/2109.07982v1).
- [Fuse only what matters](https://arxiv.org/html/2412.17235v1): visual measurement
  Jacobians/information and degeneracy subspaces, not PnP pseudo-absolute pose
  plus fixed CI. The inspected [SelectiveKF repository](https://github.com/jiejie567/SelectiveKF)
  exposes a README, not a runnable official full SKF implementation.

## License boundary and implementation status

Package metadata currently says MIT. FAST-LIVO2/R3LIVE have GPLv2; OpenVINS
headers specify GPLv3-or-later. No upstream protected source is copied into
this patch. Standard published equations are independently implemented using
existing Eigen/OpenCV; existing project MEI calibration/rectification/decoder
utilities only are reused. Reading source is not a legal "clean-room" proof,
and this does not audit license compatibility of the whole repository.

This is not official OpenVINS, complete original SKF or fully tight-coupled
LiDAR-inertial-visual SLAM. NDT stays a pose-level measurement. New visual
measurements are pixel residuals in a temporary joint clone filter. The SKF
idea is adapted via weak-direction constrained gain and positive Joseph root
update. It must be evaluated as this concrete hybrid implementation.
