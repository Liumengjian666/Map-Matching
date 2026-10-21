# Calibration identified, frame-to-reference binding not admitted

All T_A_B map B into A. Official local Kalibr cam0.T_cam_imu is T_C_I; the
official Ouster YAML labels translations in meters and quaternion xyzw and gives
`os1_lidar_to_os1_imu` as T_I_L (and its inverse separately). The files and SHA
are recorded in ASSET_MANIFEST.json. No unit transform or C1=L was assumed.

Both interpretations requiring these same fixed calibration files are preserved
in MAP_AND_FRAME_CONTRACT.md. The unknown is whether to insert the documented
Base/optical rotation between the published reference and Kalibr C. Co-located
origins cannot settle this. We did not select a chain using NDT, previous geometry
scores, a fitted external transform, or any test error. No native code adaptor
was declared admitted, and no matrices were applied to a localization experiment.

SOCC's hardcoded approximate T_CL and inv(first_pose) normalization were read,
not copied as exact calibrated map-frame truth. Do not confuse this source
inspection with a new calibrated coordinate certificate.
