# R9 IMU资格与冻结对照合同

START b2e2470c4cc9e89df70db73803a8df2ad994ced6。使用R8相同Hard raw4一次Oracle Start、raw5–164、相同PCD/93223点Target（XYZ SHA 166c4d2a4f5d24a4c79d4c90a8a275e0fe905fcc9650d559aabb775780e81654）。所有原NDT参数、.1m处理、UKF、地图和cool-time=2s不变。IMU-on只有本次一次，不重跑R8，不执行候选。

## 物理合同

M=官方地图；L=depth_camera_link；I=imu_link。T_A_B将B坐标变换到A。官方发布及GLIM方向注释（R29封存sources）给出T_L_I的xyz/xyzw=[.003,.004,-.051,-.476,.474,.524,.525]，只按出版四元数精度归一化，不估计外参。p_L_I是L原点指向I原点的向量，长度.051244512m。

原sensor_msgs/Imu规定m/s²、rad/s。原CDR与R29 CSV逐值相同：44398总IMU，本次前端资格检查解码到raw164，不解析测试GT。实际拟发布anchor之后的8625条，额外前一条仅作因果角加速度差分，不将锚点之前的运动积分到锚点状态。IMU原始header严格单调，保持整数ns，不采用bag记录时刻。IMU比力含重力反向项；不能直接把原负Z加速度按猜测翻号。唯一允许的参考使用是R8已有raw4姿态，变换其之前最近实测比力得[-.254443,-.155350,9.472263]m/s²，与地图+Z静止比力方向相容，但设备当时有运动，不能由此估计重力/Bias或认证独立静态标定。

原HDL状态是L位姿/速度，固定世界重力加速度g_M=[0,0,-9.80665]m/s²。继承本地图Z-up解释及原模型；这是一项模型近似，不声称地图有新的独立重力标定。原姿态右乘gyro增量，需要L系角速度。适配只执行刚体输入转换：

ω_L=R_L_I ω_I。
α_L,k=R_L_I(ω_I,k−ω_I,k−1)/(t_k−t_k−1)。
f_L=R_L_I f_I−α_L×p_L_I−ω_L×(ω_L×p_L_I)。

这是同一刚体不同原点的比力关系（忽略5cm范围重力梯度）。角加速度是两条已有样本的后向差分近似，不是新IMU测量；没有合成采样/补间、平滑、GT拟合、零Bias校准或可调增益。差分会放大噪声，杆臂项范数预检Mean .279337/P95 .670845/Max4.948109m/s²，不能声称转换后是无噪声真实L加速度。输出保持原stamp，frame显式改L；orientation未知，协方差未知，原HDL也不读取消息协方差/姿态。原invert_acc/invert_gyro均false。减重力由原HDL完成，不在适配器重复减。实现和刚体测试见tools/imu_geometry.py、test_imu_geometry.py。

## 时间近似：用户已显式接受

用户确认原文：**“接受原版近似并记录实际时间缺口，不修改时钟或数学”**。

原HDL按到达顺序缓存，点云回调消费stamp≤scan的IMU；不自动排序/TF/补齐到扫描端点。每批真实消息按header升序发布，等待原imu_callback返回记录，确认已入缓存才发布扫描。对每个predict实参及实际prev_stamp读取记录，未来IMU不得进入本扫描。不能将publisher成功视作传播成功。

原predict首次收到IMU才设置init_stamp，2s内更新prev_stamp但不传播；correct(scan_stamp)更新last_correction_stamp但不修改prev_stamp。这种原生离散近似导致校正状态与下一预测起点不严格共时。保留、不修复、不合成端点IMU、不改时间偏移。此次首IMU1690354429663965819ns，所以预检raw5–33无有效传播；与R8首次无IMUpredict在raw5开始计cool-time的差别需披露，不能称完全相同传播次数。

实测相关IMU间隔Max102.216553ms、4个>50ms；扫描最后IMU年龄Mean5.272799/P9519.265613/Max75.723881ms。raw68无新IMU；raw69继续原来的时间跨度，不能把raw68伪称有惯性传播。按源码时钟推演5358次有效传播、3267次cool-time/首次跳过；这些数尚非实际运行计数。逐帧模型在evidence/NATIVE_TIME_PREFLIGHT.csv，实际值必须另以原生调用回执认证。

时间假设经用户确认后，判定PASS_WITH_EXPLICIT_NATIVE_APPROXIMATIONS；不是精确同步预积分/惯导时间合同证明。IMU采样缺口、原生异步近似、后向差分、地图Z-up/零初速及未知真实Bias都是对结果解释的限制，不事后调参。

## 运行、封存及预注册

160原生align上限，额外align0，原UKF正常工作；禁止未来参考/外部重置/robot odom/global/C+G/候选/新门。保留原生全部质量处理，无新拒绝条件。仅合同错误、非有限核心或不可恢复执行故障停止。

独立imu_on_01目录独占，一次正式回放。保留R8真实Source SHA、Target SHA检查。完整callback含IMU调用与记录开销；适配在独立输入模块（监督进程内）中计时，进程内共享RSS不能伪称其单独峰值。先封存所有原生输出与调用日志，再用R8相同参考插值、无拟合评价全部160帧。原参考是地图锚定GICP+IMU，非独立动捕，故IMU对照与参考还可能共享建模信息。

H1报告所有160帧预测误差和固定83/84/85/105/130/142，描述性“明显降低”取至少1°作为本轮事前解释尺度，不作为在线阈值。H2严格采用总控条件：旋转RMSE≤R8×.8、平移RMSE≤R8+.10m、持续高误差数量或最长失锁跨度改善、无非法状态/重置/参考泄漏。原>.5m或>5°、连续3帧确认/恢复不变。只改善预测不能代替H2。若无输入/执行故障且H2通过，判IMU_PREDICTION_AND_LOCALIZATION_IMPROVED；否则按实际H1/完整恶化分别报告其余标签，不追加实验。
