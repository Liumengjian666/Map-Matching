# 冻结HDL原生IMU路径

上游de2dc7769aa2412876d05ae6161408d3295b6a3c，完整源码路径根：/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src/hdl_localization。原源码与动态库不修改、不重编译。

- apps/hdl_localization_nodelet.cpp:174 imu_callback仅push_back，未用IMU frame、姿态、协方差，未TF、校准或预积分。
- 同文件:183 points_callback先原source转换/降采样。:234 use_imu时循环缓存，stamp>scan则break，否则按原stamp及acc/gyro调用predict，最后erase。invert设置是整体乘±1，R9都false。缓存不排序，故监督器严格升序且按原回调返回握手。
- src/hdl_localization/pose_estimator.cpp:80 predict重载。首次/cool-time/同stamp不执行UKF，仅设置prev_stamp；否则dt=stamp-prev_stamp，过程噪声×dt，调用原ukf->predict(control)。没有max_dt截断、零加速度补偿或重力初始化。
- include/hdl_localization/pose_system.hpp:48–85：状态[p,v,q,b_a,b_ω]；p+=v dt；v+=(R(q)(f−b_a)−[0,0,9.80665])dt；q右乘normalize([1,(ω−b_ω)dt/2])。Bias作为状态、过程模型常值，初始0并有小过程噪声。不能说原版完全没有Bias状态；也不能说其完成了独立静态Bias标定。不是FAST-LIO2式扫描预积分/deskew。
- pose_estimator.cpp:139 correct把last_correction_stamp设为scan；:189 align一次，原终值位置/四元数直接形成7维观测，:205附近原UKFcorrect。没有由inlier或Gaussian触发的新门。correct不设置prev_stamp，无OOSM/IMU重放。原NDT质量诊断不等同UKF观测拒绝。

## 只读实际调用测量

R8的ELF wrapper仍每次原样转发一次原NDT与点云callback。本轮只追加imu_callback和predict的ELF wrapper：前者原回调返回后记录到达；后者读取实际init/prev/correction时间戳和UKF均值，原样forward一次，记录实际耗时、传播判定和输出有限性。访问仅为只读，未更改private数据、原库参数或状态。编译头的访问可见性仅方便读已有ABI字段，不改变类布局。异常核心状态/未来时刻立即中止监督运行，不是质量门。

IMU_RECEIVED计数是原缓存入队证据；IMU_PREDICT包含所有消费（含cool-time跳过）；active_predict才是原ukf->predict(control)调用数量。NDT记录每帧实际guess，避免以单独计算的IMU姿态冒充真实初值。

## 已知时间边界

冻结数据raw68无新IMU，raw69首批传播起点仍在raw67校正前。预检的跨校正时刻区间合计.815374866s是原时钟路径统计，不能直接称物理运动被双重积分这么久或由此推出唯一误差根因。用户已接受这一原版近似；本轮保持并测量，不进行时间补样/修复。
