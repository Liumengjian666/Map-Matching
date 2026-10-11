# Directly relevant 2024–2026 work

Checked2026-10-11. Scope is these five requested works only. Primary papers,
author repositories and official documentation; no new implementations,
downloads of datasets, training or reproduction experiments. Performance
claims from different hardware/datasets are not our measured results.

|Work|Verified capability and source|Implication / remaining evidence|
|---|---|---|
|SOLiD, RA-L2024 / ICRA2025|[Paper](https://arxiv.org/abs/2408.07330), [authors' code](https://github.com/sparolab/solid). Lightweight structured range/azimuth descriptors, place retrieval and initial-heading estimation. Existing pin9a4f2895e5211f69fbbd6d4732577e1a67e9b882, BSD-3-Clause.|Existing adapter uses range cosine; authors already implement yaw estimation. Adding it is mature-code reuse, not originality. Our5816 valid queries are NOT labelled recall.|
|DCReg, 2025 preprint / IJRR2026|[Paper](https://arxiv.org/abs/2509.06285), [authors' code](https://github.com/JokerJohn/DCReg). Schur-based local conditioning/degeneracy treatment and preconditioned optimization preserving the original least-squares objective. Actual experiment pin8ce8451b15491a4bbe17cf85ab02a8bed6696861.|Addresses local registration numerics, not a demonstrated certificate of global place correctness. Does not make our approximate initialization failure an inherent solver defect. License remains unconfirmed for our pin; no vendoring.|
|RTR-Net, ISPRS2025|[Publisher abstract and method preview](https://www.sciencedirect.com/science/article/abs/pii/S0924271625003065), DOI10.1016/j.isprsjprs.2025.07.036. Retrieval → training-free spatial-consistency trigger → local/global/context reranking.|Selective reranking is existing work, not our proposed innovation. Official runnable author repo/license NOT_VERIFIED in bounded search; full-paper reproduction NOT_RUN. No claim that code is unavailable everywhere.|
|InLiER, RA-L2026|[Paper](https://arxiv.org/abs/2607.16862), [authors' implementation](https://github.com/LTU-RAI/InLiER), [MIT license](https://github.com/LTU-RAI/InLiER/blob/main/LICENSE). Learning-free MINT shortlist, BEAM yaw/reranking, token-guided6DoF verification; released C++17 core and Python API.|Closest reusable remedy for expensive poorly initialized full-map verification. Verify function returns query-to-reference sensor transform, not an absolute map pose. Requires reference keypoints/tokens and an explicit map-frame transform; our descriptor-only database is not a drop-in InLiER database. Performance/false-positive behavior on our data NOT_RUN.|
|Autoware Multi-NDT, maintained implementation inspected2026|[Official documentation](https://autowarefoundation.github.io/autoware_core/latest/localization/autoware_ndt_scan_matcher/). MULTI_NDT converges multiple initial poses; MULTI_NDT_SCORE uses nearest-voxel likelihood; Hessian-oriented initial offsets and 2D covariance modes.|Multistart, Hessian-informed offsets and score-only alternatives already exist. Documentation warns about computation. This is a live software baseline, not a newly dated2026 paper. Its native quality/covariance output is not independent place truth.|

Inference, not a literature-proved absence claim: none of the inspected scope
alone entitles us to certify actual location from solver convergence. We have
not tested whether RTR-Net or InLiER already resolves our suspected aliases.
The safest next task is an author-code labelled baseline, not new Dual-U math.
