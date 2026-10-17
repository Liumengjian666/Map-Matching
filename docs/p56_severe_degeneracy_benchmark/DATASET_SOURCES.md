# Dataset sources and access checks

## Ground-Challenge

Official repository: `https://github.com/sjtuyinjie/Ground-Challenge`

Official README says the VLP-16 data use `/velodyne_points`, while Xsens MTi-300 data use `/imu/data`; Corridor2 is listed as 2.9 GB / 98 s / straight-forward long corridor, Corridor1 as 2.9 GB / 100 s / zigzag long corridor. These are Ground-Challenge names and are not SuperLoc Corridor01/02.

- Corridor2 official OneDrive share: `https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/EcWKsukr7nNLrqKloXY3UAIBKakA6ppET_SGJZT-VCYO5Q?e=Ccp0fd`
- Corridor1 official OneDrive share: `https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/Ed4KpGMgT2RMgzqt0HU0fw8BOvtgx18YnkdJQVekzwfe8g?e=5S9uLK`
- Official `psudo_gt` directory: `https://github.com/sjtuyinjie/Ground-Challenge/tree/main/psudo_gt`
- Official calibration: `https://raw.githubusercontent.com/sjtuyinjie/Ground-Challenge/main/calibration.txt`

Access evidence for Corridor2: `curl -I -L` redirects to a SharePoint `Authenticate.aspx` / Microsoft sign-in flow. A single byte-range request ended at `login.microsoftonline.com`, HTTP 200 `text/html`, 58,672 bytes. It was not saved as a dataset file; no authentication bypass or alternate source was used. Corridor1 was not attempted because the requested priority gate (Corridor2 validation first) was not met. The repo root does not list an official prior map. The downloaded calibration file is 932 bytes, SHA-256 `f8f591c46fa21c88d3986e65bf0558c7f79f2830b017165ec86b87190f745e08`; inspection shows `body_T_cam0` and `body_T_wheel`, but no explicit LiDAR-IMU transform.

## SubT-MRS Long_Corridor

Official dataset portal: `https://superodometry.com/datasets`

Official ICCV challenge page: `https://superodometry.com/iccv23_challenge_Mul`

The challenge page's official ROS-bag folder is `https://drive.google.com/drive/folders/1bV5oCLrpVoc6xKcIduUcco47nG0jPH6i`. It contains `SubT_MRS_Hawkins_Long_Corridor_RC.zip`, Drive file id `1-WF_cIPXVEzcStfYjJWoZfJkld0Ozmxk`. The official page lists Long_Corridor as RC2, 616.45 m, 295 s. The official direct-download response advertises 3,888,497,930 bytes and `Accept-Ranges: bytes`; a `Range: bytes=0-0` probe returned HTTP 206 and `Content-Range: bytes 0-0/3888497930`.

Official Long_Corridor extrinsics link: `https://drive.google.com/file/d/1bB3jfEJeTf_XoLUHKOaxCNF_MCkiQTol/view?usp=drive_link`. The 584-byte file has SHA-256 `03df4bc1d32ba7148fd90a35f370d319bb83531c774d1848b3da0acf25f390bc`. It declares `laser_to_imu`; direction and sensor-frame semantics still require source/format validation before use.

The official page's separate `Multi-Modal-Sensor-Fusion-Track` ground-truth Drive listing marks the Long_Corridor item `Limited access`. No GT was downloaded or read. The official Long_Corridor row lists extrinsics/intrinsics, not a prior map. Any same-recording map will therefore be labeled self-map diagnostic unless an official independent map is verified.

Download outcome: the official file advertised 3,888,497,930 bytes and accepted byte-range requests. Attempt 1 received 1,699,970,593 bytes and exited 18 (partial transfer). Attempt 2 resumed the same official URL and received another 1,780,770,630 bytes (HTTP 206) before exiting 56. Cumulative partial size is 3,480,741,223 bytes; 407,756,707 bytes remained. The partial remains on the external drive as `SubTMRS/LongCorridor/raw/SubT_MRS_Hawkins_Long_Corridor_RC.zip.part`, SHA-256 `6fb37b96d9ac80ebe3fd32cc113504d1c8c6c791919a30b073a0b4a8786f16fa`. It is not a validated ZIP; no extraction or bag inspection was performed. The two-attempt limit was reached, so no further download was tried.

## References used for preparation format

- SOCC-ICP dataset preparation: `https://github.com/josch14/socc_icp/blob/main/DATASETS.md`
- SOCC-ICP Ground-Challenge extraction helper: `https://github.com/josch14/socc_icp/blob/main/scripts/prepare_ground_challenge.py`
- GenZ-ICP reproduction guide: `https://github.com/cocel-postech/genz-icp/blob/master/ros/README.md`

These are implementation references, not alternate data authorities.
