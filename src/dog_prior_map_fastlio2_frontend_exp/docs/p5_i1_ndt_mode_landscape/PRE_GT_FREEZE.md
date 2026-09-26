# P5-I1 pre-GT mode discovery freeze

GT was not opened or read in this stage. Seed generation, objective ranking, clustering, stable-mode tagging, basin entropy, inter-mode separation and scatter were completed using fixed scan inputs only.

Runtime topic bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag`.
Frozen H1 map: `/tmp/floor01_candidates/floor01_h1_map.pcd`; PCD points 549637; final fixed target points 549606 after finite filtering and two 0.15 m voxel passes.
NDT: PCL `NormalDistributionsTransform<PointXYZ,PointXYZ>`, package version `1.10.0+dfsg-5ubuntu1`; resolution 0.8 m, step 0.08, transformation epsilon 0.001, max iterations 40; target-cell radius search. Full fixed target; no local crop.

Frames: 32; total runs: 8800; stable curvature requests: 108.
Runtime-topic bag SHA256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`.
Prior-map PCD SHA256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`.
Primary thresholds: translation <= 0.20 m and rotation <= 2 deg; strict = half translation/1 deg; loose = 0.40 m/4 deg. Stable-mode diagnostic tag: >=5 converged seeds and >=2% of converged seeds in the fixed seed domain.
Clustering: deterministic complete-link agglomeration using max(translation/tau_translation, rotation/tau_rotation); clusters merge only if all cross-cluster pairs satisfy both cutoffs. Equal-distance ties use the minimum original seed index. This prevents single-link chaining.
Seed schedule: each frame has 245 planar + 18 axial seeds; targeted frames add 48 wide-ring seeds with yaw {-15, 0, +15} deg. Pose perturbation is right/body: T_seed = T0 Exp(delta).
GT-blind clustering/scatter script wall time: 72.863776 s (includes deterministic clustering and table aggregation; excludes plotting/write-out).

## Freeze closure before GT post-hoc

Before opening GT, the PCL optimizer-reported analytic Hessian and finite-difference sensitivity outputs were also generated for all 108 selected representatives. `mode_curvature.csv` SHA-256: `25af7bf4ce25995214908d3114ec5d11010625a2eb8ef97eae03db000bc3fa1c`. The frame manifest and every seed row carry the verified bag/map digests above; curvature requests carry those digests and the curvature helper rejects a map or bag digest mismatch. GT was read only after this closure.
