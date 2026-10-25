# External methods: bounded published-result check

No official HDL Localization or FAST-LIO Localization framework has been run in R29. Internal complete causal comparison has priority. R29 methodD is native PCL NDT in our existing EKF, not Koide's complete HDL system and not the discontinued R21 relative-admission D.

Primary paper: Koide et al., *Tightly Coupled Range Inertial Localization on a3D Prior Map Based on Sliding Window Factor Graph Optimization*, ICRA2024, DOI10.1109/ICRA57147.2024.10611195. Official author PDF retained as sources/koide_icra2024.pdf with source receipt/SHA. TableI (PDF p5) reports indoor ATE[m]:

|Method|Easy01|Hard|
|---|---:|---:|
|FAST_LIO (odometry)|2.485 ±1.216|27.942 ±23.022|
|FAST_LIO_LOCALIZATION|.068 ±.044|27.265 ±25.353|
|hdl_localization|.210 ±.177|25.507 ±24.431|
|Paper proposed method|.054 ±.008|.282 ±.253|

The paper states139s Easy01/164s Hard, manual initial pose for all methods, batch GICP map+IMU reference, and evo ATE. The inspected passage does not specify an exact evo command, post-alignment option or the interpretation of every±quantity; do not silently call these our unaligned RMSE or confidence intervals. Our exact reference-anchor/C3 definition, fixed preprocessing/NDT/quality rules and frozen IMU model differ. These published values are protocol-B context, not an A-grade matched ranking or new reproductions. No improvement percentage against them is justified.

Official HDL uses NDT map registration with UKF/IMU fusion; FAST-LIO Localization adds map corrections to its own FAST-LIO2 frontend. Running them requires matching initialization, IMU/scan adapters, output frame/time and full trajectory. No one-frame optimizer experiment is substituted. No bulk clone/compile was started; external execution is deferred until internal admissibility/safety results are known, and must not bypass an active safety stop.
