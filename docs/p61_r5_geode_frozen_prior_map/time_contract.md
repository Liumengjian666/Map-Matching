# Actual author FAST-LIO time contract

The original internal bag SHA is `162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091`.
Its 5427 Livox CustomMsg headers have ROS epoch time; `timebase` has a different device origin. No device-to-epoch substitution is made.

In fixed author revision `1f008a7249e36393a1752622de50660b77b5b7f4`:

- `FAST_LIO/src/laserMapping.cpp::livox_pcl_cbk` enqueues `msg->header.stamp.toSec()` as LiDAR begin time.
- `FAST_LIO/src/preprocess.cpp::avia_handler` stores `offset_time / 1000000` as curvature in **milliseconds**. Both feature paths implement this same conversion. This experiment uses the author's feature-disabled path.
- `sync_packages` derives scan end from begin + last admitted point curvature /1000; it waits until an actual Xsens message at/after the scan end has arrived. It consumes the measured IMU samples up to the scan end.
- `FAST_LIO/src/IMU_Processing.hpp::UndistortPcl` uses actual IMU header differences, carries the prior IMU sample, integrates to scan end and backwards transforms points into the scan-end LiDAR frame using their curvature /1000 offsets.
- `imu_cbk` subtracts configured `time_offset_lidar_to_imu`. The author's carol configuration fixes this to 0 and `time_sync_en=false`; this run preserves both values.

Operational point time is therefore `ROS header + offset_time_ns`, not device `timebase + offset_time`. The R2 ledger independently found header−timebase constant within 238ns over the full bag, supporting a changed clock **origin**, not a seconds/milliseconds unit substitution. The actual consuming implementation and author's matching Livox/Xsens configuration now establish the implementation-side contract. No claim is made that undocumented acquisition/retiming firmware has been independently recertified.

The physically filtered input has 2441 complete point-time-bounded scans and 24418 measured external IMUs. Every delivered header lies inside the immutable mapping window. The first IMU after mapping start is 1706671289400330543ns, 0.330ms after the first LiDAR start; the author's original first-scan discard and IMU initialization remain intact, rather than fabricating pre-roll. Map-producing scans are audited separately for integration support. Scan 2442 ends at 1706671533600190259ns, beyond the fixed 1706671533569869338ns cutoff, and is excluded whole.

Replay sends measured IMUs through the first real tail-bracketing sample before publishing each scan and waits for that scan's actual completion ACK. Raw sensor timestamps are unmodified. `use_sim_time=false` controls only ROS loop scheduling so a per-frame ACK cannot deadlock waiting for a clock that the publisher itself has paused. All estimator input/state times continue to be sensor header times. FAST-LIO uses double seconds internally; the exact original integer ns are retained in replay.csv and joined by consumed sequence, not recovered by pretending the double retained nanosecond precision.

Record `T_M_I` and `T_M_L` at the mapper's scan-end state. M is the non-GT mapper's `camera_init` world; IMU body is `body`, not the LiDAR optical/sensor origin. The author reads its matrix as `offset_R_L_I, offset_T_L_I` and transforms `p_M=R_M_I*(R_I_L*p_L+t_I_L)+t_M_I`. The configured rounded matrix is checked for SO(3) closeness; the actual manifold representation used by the mapper is also recorded, never silently replaced with another calibration.
