# 主验证集选择：Hilti-Oxford 2022 Exp18，尚未准入

## 决策

唯一主候选：**Exp18 Corridor Lower Gallery 2 / Sheldonian**。

状态：**PRIMARY_CANDIDATE_SELECTED_BUT_NOT_ADMITTED**。存在独立 TLS 地图和稀疏测量位置参考，公开稠密6DoF参考却包含测试扫描到该地图的配准；当前还不能满足“独立稠密六自由度定量验证集”这一完整目标。任务 B 的候选选择完成，实验合同资格未完成。不得将准备稿写为 BENCHMARK_LOCK_PASSED。

数据合同依据：[官方数据页](https://hilti-challenge.com/dataset-2022)、[官方仓库](https://github.com/Hilti-Research/hilti-slam-challenge-2022)、[官方托管文件](https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022)、[数据集论文](https://arxiv.org/abs/2208.09825)，DOI 10.1109/LRA.2022.3226077（2022接收，RA-L 2023卷8期1，408–415）。

## 可获取性与最低资产集合

HF 树/API 在本轮可读、gated=false；冻结 revision `e62017f907007fdc5ab8c721842e4ae7359d7f49`。这证明公开目录与下载入口存在，不等于已下载并验证 rosbag 内容。未下载任何大 bag/E57。

| 资产 | 字节数 | 官方 LFS SHA256 或本轮实算 |
|---|---:|---|
| rosbags/exp18_corridor_lower_gallery_2.bag | 8657654860 | 98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280 |
| laser_scans/sheldonian_full.e57 | 5447128064 | 46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756 |
| ground_truth/exp18_corridor_lower_gallery_2_imu.txt | 88743 | 138def6a40884a87372a4f18938cce5180d6aa7ad4e705349abf47ac4ec9fa32 |
| ground_truth/exp18_corridor_lower_gallery_2_imu_3dof.txt | 185 | 13f963c4a2e9d583532051b581a19cf73ade0aadc83f530ea3e66c6d52036dec |
| calibration/calibration_files/lidar_calibration.yaml | 747 | ea5127989317270d83d9a701318d7e284031a21bd9cbf466b8f9f169547db43c |

前两行是托管元数据，不冒充本机实算；后三行已仅为合同检查读取，不评价任何算法。精确下载URL以该revision的resolve路径构造；不得换main后仍声称文件身份相同。

本地有界文件名盘点未找到 Hilti/Exp18 bag 或 E57（仅既有 FAST-LIVO2 Hilti launch/RViz，不能当作数据）。本轮只写内盘来源缓存与文档；没有访问 HIKVISION。

bag+地图合计 **14104782924 bytes**。内盘可用曾测 **50319212544 bytes**；20GiB安全余量+暂定8GiB转换工作区+两大文件约需44169553996 bytes，名义余量约6149658548 bytes。转换实际膨胀和其他进程占用尚未知，因此**下载前必须重测；不能据此批准无界E57转存**。分块导出目标、内存和最大临时体积需预先固定。现轮不因“有空间”自动下载。

## 地图、参考与独立性

1. 独立的静态 TLS 扫描/Scantra 注册构成地图，非 Exp18 测试点云逐帧拼图。官方说明注册平移残差量级<1mm，不可把它当作每帧定位真值精度保证。
2. 论文 Sec.V-C 的稠密参考由 VILENS 与测试点云到同一 TLS 的 ICP 生成，IMU用于去畸变。可用于**注册相关的条件性诊断误差**，不算独立外部稠密姿态真值；其旋转参考也不独立。
3. Sec.V-B 的稀疏地面靶标由 TLS 测量，传感器装置 measurement_tip 置于标记。它不由 NDT 测试结果生成，但与地图共享测量基准，要披露相关性。只支持位置，不支持旋转RMSE。
4. 不能使用测试 GT 拼地图、初始化、选参数或候选；不能全段 Umeyama 对齐。地图导出坐标与参考世界系的官方关系仍待 E57 元数据/官方消费者代码证据闭合，不假定恒等。

## 实际检查到的参考支持

稠密文件789条、8列，首1649856227.723471s，末1649856314.319139s，跨度86.595668s。尚未核验 quaternion 列顺序/方向，不直接按 TUM 套用。

稀疏文件3条时间戳：1649856228.9143598、1649856287.0620654、1649856335.6804302s。文件头与实际8列并不完全吻合，末尾单位四元数为占位，不能当作测得姿态。仅前两条位于**稠密参考**时间支持内；第三条是否位于原始bag支持内尚未检查，不可先行排除。

**3是可用标签数，不是本方法已评价样本数**。三个都须保留并报告可配对性；不外推、不用最近帧/稠密结果偷换稀疏测量。100个均匀诊断帧也不自动拥有独立测量标签。禁止用789个相关参考姿态冒充789个独立位置真值样本。

## 传感器、许可及开发集

官方平台包含 PandarXT-32 与 Alphasense/BMI085，点云有逐点时间；官方描述共同同步/时间差<1ms。当前只核验发布说明，**实际bag字段、时间单位、IMU覆盖和标定适用性尚未核验**。不像GEODE那样仅因ROS epoch相近就宣布物理同步。

HF许可为 **CC-BY-NC-SA-3.0**：学术非商业使用须遵循署名、非商业、相同方式共享要求；衍生地图发布前另核对许可范围，不默认允许商业应用。数据不入Git。

若后续合同准入通过，可预选同环境不同记录 Exp16 为开发集、Exp18固定为测试集，地图共用独立TLS；不得按测试效果换开发集。Exp16 bag元数据约15523312816bytes，本轮不下载。只有一个测试序列不能证明跨场景泛化；第二独立序列的扩展须另行批准。

## 唯一备用检查：NCLT（未获准入）

已读[官方页](https://robots.engin.umich.edu/nclt/index.html)和[数据集论文](https://robots.engin.umich.edu/nclt/nclt.pdf) Sec.6：发布参考使用跨会话联合SLAM、激光匹配和RTK GPS，非纯独立稠密姿态测量。用某天的发布参考建图也可能通过联合优化间接依赖测试会话。独立RTK原始位置可进一步核验，但其覆盖、质量及地图系尚未知，亦无此轮已验证TLS地图。

因此NCLT不自动替补为“合同通过”；未再扩展KITTI/MulRan，无任何备用大数据下载。若总控要求独立稠密6DoF作为硬门，此轮结论必须是**主验证集资格仍阻塞**，而不是降低独立性定义让任务通过。
