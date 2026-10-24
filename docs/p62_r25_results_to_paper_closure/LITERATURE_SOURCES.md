# Bounded primary-source check for the manuscript

Checked2026-10-24; no new dataset retrieval or broad literature audit.

- Native NDT background and API: https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html . The live page is newer documentation; actual runtime is local PCL1.10 with frozen source/parameters. Magnusson thesis primary PDF was located at https://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf but direct full-PDF fetch timed out; no claim of new full-text review. The paper cites the thesis for bibliographic background and PCL for implemented API, not unverified numerical equivalence.
- X-ICP: https://arxiv.org/abs/2211.16335 , submitted2022, revised2024. Correspondence-based principal-direction localizability and constrained ICP optimization; not a source for claims of our novelty or numerical superiority.
- GenZ-ICP: https://arxiv.org/abs/2411.06766 ,2024 arXiv and listed RA-L DOI10.1109/LRA.2024.3498779. Adaptive use of point-to-point and point-to-plane metrics; no direct experimental baseline in our archive.
- SuperLoc: https://arxiv.org/abs/2412.02901 , accepted ICRA2025; predictive alignment risk assessment before optimization. Our evidence concerns a particular hard rejection policy, not a refutation of predictive risk analysis or proof of superiority to SuperLoc.
- FAST-LIO2: https://arxiv.org/abs/2107.06829 . Related LiDAR–IMU odometry background; our frozen custom EKF is not claimed to be an unmodified FAST-LIO2 implementation. Dog prior map is FAST-LIVO2-derived, a distinct system/name.
- Newer College paper: https://arxiv.org/abs/2003.05691 ; official2020IROS citation/DOI10.1109/IROS45743.2020.9340849 on https://ori-drs.github.io/newer-college-dataset/ .
- Reference construction: https://ori-drs.github.io/newer-college-dataset/ground-truth/ explicitly describes Ouster-to-Leica ICP reference construction. This page still says RS_C1 for stereo, Base for multicamera. The historical paper-supported Base interpretation used here remains conditional; no CSV-export identity resolution or alternate-axis experiment is invented.
- Historical Base evidence: pinned https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/README.md and archived R14B BASE_FRAME_EVIDENCE.md. Not a new coordinate audit.

No literature claim that simple threshold gating, multistart, tiling or eigenvalue tests are original. No head-to-head performance ranking against the above external systems is claimed.
