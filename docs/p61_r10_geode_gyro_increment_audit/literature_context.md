# Background only — no novelty claim or exhaustive review

The three task-specified references were verified through publisher/author
primary pages. DOI resolver access initially failed; the publisher/author pages
below were then available through web retrieval. No experimental sensor asset
was downloaded or changed by this limited bibliographic check.

1. Zhong et al. (2023), *Low-cost solid-state LiDAR/inertial-based localization
   with prior map for autonomous systems in urban scenarios*,
   DOI10.1049/itr2.12273.
   [Publisher](https://ietresearch.onlinelibrary.wiley.com/doi/abs/10.1049/itr2.12273),
   [author university record](https://research.polyu.edu.hk/en/publications/low-cost-solid-state-lidarinertial-based-localization-with-prior-/).
   Uses the complementarity of solid-state LiDAR and IMU with a prior map;
   IMU preintegration supplies map-matching initial estimates. Relevant background
   for why NDT appearance, initialization and inertial information must be
   distinguished. R10 is only an increment-consistency diagnostic, not a new
   localization or preintegration algorithm or a performance comparison to it.

2. Chiang et al. (2023), *High-Definition-Map-Based LiDAR Localization Through
   Dynamic Time-Synchronized Normal Distribution Transform Scan Matching*,
   DOI10.1109/TVT.2023.3237275.
   [IEEE publisher](https://ieeexplore.ieee.org/document/10018201/),
   [author university record](https://researchoutput.ncku.edu.tw/en/publications/high-definition-map-based-lidar-localization-through-dynamic-time/).
   Discusses initialization/evaluation, time synchronization and geometric
   density in NDT map localization. These motivate checking timestamp/frame
   conventions before interpreting a motion discrepancy. R10 does not reproduce
   or claim to improve its synchronization or uncertainty estimation.

3. Park and Chung (2024), *Uncertainty-aware LiDAR-based localization for
   outdoor mobile robots*, DOI10.1002/rob.22392.
   [Publisher](https://onlinelibrary.wiley.com/doi/10.1002/rob.22392),
   [author university record](https://pure.korea.ac.kr/en/publications/uncertainty-aware-lidar-based-localization-for-outdoor-mobile-rob/).
   Estimates map-matching uncertainty from an NDT-map pose distribution and
   combines map matching, LiDAR-inertial odometry and GNSS in a factor graph.
   Relevant to the risk of indiscriminately accepting unreliable matches.
   R10 estimates no pose covariance and implements no weighting/fusion method.

This list is task background, not a five-year novelty search, survey, or proof
that prior work solves/fails on GEODE. The independent fixed-bias gyro check is
not presented as a scientific innovation. Any future mathematical-method claim
requires a separately approved comparison of assumptions, observations, failure
mechanisms, complexity and experiments.
