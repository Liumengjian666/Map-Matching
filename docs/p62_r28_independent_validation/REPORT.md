# P62-R28-INDEPENDENT-VALIDATION-AND-MATCHED-BASELINE

**Formal result: G0 COORDINATE_CONTRACT_BLOCKED.** START c8a02bf1cf16413569f940ef532f14ae19a38d9a. Branch research/p62-r28-independent-validation-matched-baseline. Independent workspace /home/jian/livox_ws/p62_r28_validation_ws. Local commits only; not pushed.

The exact existing reference CSV is official2020 Stereo-Cam and matches its historical checksum. The current official website identifies RS_C1 as optical, while the pinned robot-model description identifies robotics Base as the GT frame. Both calibrated chains are valid SE(3), but no inspected exporter/per-file record resolves their binding to this payload. No test pose errors or alignment fit were used to choose between them.

Requested Window-A overlaps the R19 development interval. Before any new pose evaluation, A was deterministically replaced by anchor12000/test12001–12060; B remains13500/test13501–13560. Raw identities are sealed, but executable Source/map/anchor freeze remains incomplete behind G0. This is disclosed, not represented as a completed independent experiment.

No new NDT, IMU/EKF run or external algorithm run was performed. Error/latency/RSS fields are unavailable, not zero. The selected method, all historical experiments and R27 manuscript were preserved. FAST-LIO-LOCALIZATION's official source was inspected and actual Python imports were checked; several compatibility issues were observed without changing the environment. GICP headers/config exist but no fallback experiment was launched under an unresolved frame contract.

The manuscript was updated as a complete editable draft/PDF, retaining historical results as conditional and adding the actual blocked-validation disposition. See FRAME_CONTRACT.md, FINAL_RESEARCH_DECISION.md and PAPER_EVIDENCE_UPDATE.md. Tool/build logs and critical input SHA are retained. END is the final local delivery commit shown by git rev-parse HEAD and the final handoff; it is not embedded recursively in its own tree.
