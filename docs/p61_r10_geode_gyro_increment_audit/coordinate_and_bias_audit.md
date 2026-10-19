# Frozen coordinate, bias and input provenance

T_A_B maps B coordinates into A. M=camera_init frozen map gauge;
I=external Xsens imu_link; L=Livox Avia livox_frame.

Actual runtime17922f2b63dae1f2af34f353287728d50a312f73:

- `include/dog_prior_map_localization/core/frame_conversions.hpp:9-18`:
  T_M_I=T_M_L*inverse(T_I_L), T_M_L=T_M_I*T_I_L.
- `src/fusion/ndt_observation.cpp:217` calls the LiDAR-to-IMU conversion.
- `src/imu_processor.cpp:116` reads angular_velocity directly in IMU axes.
- `src/core/imu_propagation.cpp:50-55` subtracts body gyro bias and
  right-multiplies the rotation increment, consistent with dR/dt=R[omega_I]x.
  R10 executes none of these runtime functions or EKF gravity/acceleration logic.

The four source files were byte-checked against actual runtime Git objects;
hashes and version are in inputs_frozen.json. This is not an assumption based
on variable names. No production file was edited.

## External IMU and time

Actual prepared input inspection decoded111 /imu/data sensor_msgs/Imu messages,
all imu_link, original header/sequence/serialized SHA matched frozen input/imu.csv.
Gyro field is angular_velocity xyz, rad/s per installed sensor_msgs/Imu.msg
definition (`/opt/ros/noetic/share/sensor_msgs/msg/Imu.msg:1-3`). It is used
unchanged by the actual writer and runtime. No orientation or acceleration field
contributes to the calculation. No /livox/imu data was read.

Writer source at1367e1d9587d1f90a47bf6f44405239a7b65e4f6 preserves the original
IMU message and writes bag record time=header time. R10 selects raw bag records
by exact integer boundaries, then independently verifies header=record=ledger,
frame and serialized SHA. ROS epoch timestamps are used, never device timebase.

Target interval union:
1706671804799723787 .. 1706671805899593546ns.
First decoded real head1706671804791549444ns; last decoded1706671805892444849ns.
No sample after the union end was decoded. Each interval independently takes
only its nearest causal head and later samples<=its own end. Later-window
samples cannot contribute to an earlier integral.

This confirms the recorded ROS-time/units/frame contract, not independent
hardware clock synchronization, physical sensor calibration or true trajectory.

## Fixed initialization bias and extrinsic

Authoritative R6 initialization SHA:
9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d.
Its initial_state.gyro_bias_I exactly equals R8 initialization YAML
gyro_bias_imu_radps, with fullprecision vector:

[-0.0008725237314739192,
  0.0023757483405765214,
 -0.010976615256704694] rad/s, I axes.

Rounded9decimals equals the task display exactly. The values are not inferred
from this window, Raw NDT, future EKF or ending state. Fixed bias is216.969883664s
old at the window start. No claim of proven long-term bias stability is made;
small discrepancies could reflect drift, integration approximation or clock
error rather than NDT. No bias or offset was fit in R10.

Frozen T_I_L from the same R6 JSON exactly matches the R8 fullprecision YAML:

```
 0.9996198206617928    0.027463349486431855  0.002445117092812397  0.049258
-0.027516650513568146  0.9992986370810932    0.025398187217603792 -0.0125
-0.0017458829071876037 -0.025455812782396205 0.9996744237539305    0.026946
 0                     0                     0                    1
```

Only rotation affects relative angular comparisons; translation is preserved
but is not used to fabricate any trajectory. T_M_I rotations are obtained by a
stable right-division solve, without replacing the frozen calibration matrix.
Both determinant and orthogonality are checked. Numerical relative-frame
invariance is tested synthetically, not claimed as physical calibration validation.

Raw pose source is R8 frame_results.csv, raw_ndt_pose only, scans5154–5165.
Initial/Used/fused rotations are not used as gyro truth or to repair Raw.
The separately reported Raw-vs-Initial correction is only the already-frozen
R9 diagnostic and is not confused with Raw adjacent-frame rotation.
