# Finite Newer College closure: CONDITIONAL_REFERENCE

Existing R28 frame contract, matrices, source manifest and check tool were read; no old preflight was rerun. The original 2020 Stereo-Cam CSV provenance is identified but exact exporter-axis semantics remain uncertified. Official reference page names RS_C1 optical while pinned robot description/paper support robotics Base. No new official clarification was found in these retained primary sources. This does not block Koide experiments.

T_A_B maps B into A; M=map, B=robotics Base, C=left optical, I=IMU, L=cloud. Numeric chains are saved in sources/NC_HYPOTHESIS_MATRICES.json from unmodified official calibration.

- H_B: T_M_L=T_CSV T_B_C T_C_I T_I_L; T_M_I=T_CSV T_B_C T_C_I.
- H_C: T_M_L=T_CSV T_C_I T_I_L; T_M_I=T_CSV T_C_I.

The same timestamp interpretation is retained under both; no fitted time shift. Co-located Base/optical origins do not imply co-oriented axes. T_B_C has 120-degree rotation and zero translation, but composing it before the camera-to-sensor lever arm changes the resulting LiDAR/IMU position as well as attitude. The two LiDAR chains differ by the fixed **right-hand SE3** D=(T_C_I T_I_L)^-1 T_B_C (T_C_I T_I_L), generally with nonzero translation. They are not related by a fixed rotation about the map origin; the corresponding map-frame left transform depends on T_CSV(t).

Questions requiring file-specific author confirmation: which rigid-body axes does the exact downloaded registered_poses.csv describe, pose direction, whether it is the camera-rate or LiDAR-rate export, map origin/orientation, scan timestamp convention, and whether any additional camera-clock conversion was already applied? Supply Drive ID11VWvHxjitd4ijARD4dJ3WjFuZ_QbInVy and SHA b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e. No message was sent automatically.

No choice of H_B/H_C is made by error, gravity agreement, Overlap or gate success. Any Newer College runs must record both initialization hypotheses separately; dual evaluation of one output cannot substitute for different initialized runs. Reference remains map-anchored, not independent motion capture.
