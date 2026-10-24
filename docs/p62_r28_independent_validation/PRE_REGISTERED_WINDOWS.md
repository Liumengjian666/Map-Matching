# Window identity before any new accuracy

Status: INDEX_AND_RAW_IDENTITY_FROZEN_NOT_EXECUTION_READY.

Requested A raw11000–11060 overlaps R19's formal STRONG raw10972–11032 (and reused R20/R21 exposure). It cannot serve as an independent confirmation window. Before any new NDT or reference-pose decoding, replace A by the next multiple-of-1000 anchor after the exposed end: **raw12000**, evaluation **12001–12060**. Keep B **raw13500**, evaluation **13501–13560**. This deterministic index rule was not optimized by error, quality, map coverage or point geometry.

Compared formal historical intervals: R14B 767–787; R16/R17 7552–7582; R19–R21 10972–11032 and 14542–14602; R22–R25 2600–2660, 4800–4860, 6500–6560, 9400–9460. Existing report/method/window-registration files were checked. No formal C+G result/parameter-choice use of 12000–12060 or 13500–13560 was found. R16/R19 sparse geometry-only prescreens covered parts of the full sequence, so these are not claimed to be never-inspected raw geometry. No new reference error was read to establish the window choice.

`WINDOW_CANDIDATES.json` freezes one-based ordinals, integer ns, canonical PCD paths and SHA256 for all 122 scans. The first filename inventory attempt encountered a historical `(1)` duplicate; the tool now uses the existing exact canonical `cloud_<seconds>_<9digits>.pcd` naming rule and finds 15,301 canonical scans. It changes no PCD and no window choice.

G0 is BLOCKED: source FNV, target ROI/map SHA, anchor pose matrices and IMU boundary initialization have deliberately **not** been asserted ready. These null fields prevent a raw-file identity list being mistaken for complete executable pre-registration. No map ROI was selected under an unresolved reference transform. This list authorizes no automatic later run; complete remaining input freeze before a formal experiment once G0 is resolved.

Planned, not executed: each window x C0/C3 x Original/G-only/C+G = 12 independent 60-scan real-IMU trajectories. C3 preserves LiDAR position and rotates attitude +8 degrees about map Z, recomputing the IMU lever arm. All frozen algorithm/budget/timestamp rules remain unchanged. No replacement window after observing accuracy is permitted.
