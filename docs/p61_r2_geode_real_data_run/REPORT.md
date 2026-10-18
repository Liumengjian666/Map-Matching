# P61-R2 — GEODE真实原生隧道几何实验

正式结果：`GEODE_NATIVE_GEOMETRY_COMPLETED`。

已验证真实bag并转存内盘，完成20对连续Smoke和100对全段均匀筛查。
**没有运行融合Smoke或先验地图定位。** 原生六维谱各向异性比冻结Ground-Challenge更强，
但完整P60严重弱约束联合判据仍为0/100，不能称为定位失败。

## 1. 身份、隔离及数据恢复

- 仓库：<https://github.com/Liumengjian666/Map-Matching>
- 分支：`research/p61-r2-geode-real-data-run`
- worktree：`/home/jian/livox_ws/p61_r2_geode_real_data_ws`
- START_SHA：`ed31ade24a6bc7a36720c65321997fba1ef93ab4`
- CODE_SHA（实际NDT运行时）：`25b24e607d50b9859b6609861f58a1ca4c7fa64a`
- END_SHA：见最终回执及内盘`/home/jian/geode/Shield_tunnel1_gamma/git_receipt.json`，提交后记录完整SHA，避免自引用。
- REMOTE_HEAD／PUSH_EXECUTED：最终查询另见回执；本轮不执行push。
- 生产`src/`、NDT/EKF/Coupled数学、历史R1/R5归档及稳定机器狗工作区均未修改。

外盘来源（仅只读）：
`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/Shield_tunnel1_gamma.bag`

内盘正式输入：
`/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag`

实际大小：**4,500,527,001 bytes**，与R1冻结的官方Drive公布值一致。
源和内盘独立SHA256均为：
`162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091`。
没有作者发布SHA可比较，不把本地两端一致称为官方密码学校验通过。

外盘UUID F627-4133、exFAT经FUSE挂载；本次实际设备sda、USB4-2。
复制使用rsync `--partial --append-verify --bwlimit=16384`，274.687s；
之后外盘独立限速SHA读取268.670s。内盘部分文件哈希一致后以排他硬链接发布正式文件，保留.part名字且不重复占用4.5GB空间。
没有新USB reset/disconnect或块I/O故障；copy/hash各观察到12/7次D态采样，最长连续仅1次采样，
保持进度，没有按瞬时D态误判失败。详见storage_*证据。
这仅是本次读取成功，不是SSD全面健康认证。后续计算全部来自内盘。

本次未发生中断恢复。rsync保留部分文件且支持append-verify；工具新增显式尝试标签可以保留旧证据目录，
该接口经过单测，未因添加接口重新读取外盘。

## 2. 实际bag和传感器

ROS bag V2.0，indexed，compression=none。
**180,523条原始记录全部遍历成功，各话题计数与索引完全一致**；传感器消息实际解码，
图像仅遍历序列化记录，不解码图像或运行视觉算法。
bag record时间：1706671289.3605947至1706671832.1014986秒，跨度542.740904秒。
内容、大小、传感器家族及持续时间与GEODE Shield_tunnel1_gamma发布信息一致，不因父目录名SuperLoc而混用数据。

| 实际话题 | 类型 | 数量 | frame_id | header中位间隔 |
|---|---|---:|---|---:|
| /livox/lidar | livox_ros_driver/CustomMsg | 5427 | livox_frame | 99.843264ms |
| /imu/data | sensor_msgs/Imu | 54276 | imu_link | 10.399580ms |
| /livox/imu | sensor_msgs/Imu | 109966 | livox_frame | 4.997254ms |
| 左／右compressed相机 | sensor_msgs/CompressedImage | 各5427 | 本轮不用于算法 | — |

LiDAR首末header：`1706671289400000810`、`1706671831999708652`ns；跨度542.599707842秒。
每帧24000点，line范围0–5；offset_time真实范围0–99,871,325ns。
原始XYZ、reflectivity、tag、line、offset_time、header与timebase全部保留于原bag；
用于诊断的选中帧另保存完整数值字段NPY，XYZ PCD只是几何导出，不冒充可直接去畸变输入。

严格核对bag内CustomMsg/CustomPoint完整schema后才使用19byte packed点解析；
57个真实消息与ROS原生反序列化器交叉验证一致。
没有从XYZ重建时间、填写外参或把点时间置零。

## 3. 点时间与IMU：真实双时钟问题

Livox官方驱动源代码证明offset_time为ns、基于首点timebase；参照文件及SHA已保存official/。
**本包timebase不是ROS epoch**：首帧timebase=287199614370ns，而header为1706671289400000810ns。
全5427帧的header−timebase差位于：
`1706671002200386206`至`1706671002200386444`ns，变化仅238ns。
全部LiDAR header与record时间相同。不能直接用设备timebase+offset与ROS IMU时间相比较。

候选ROS时间表达`header.stamp + offset_time_ns`具有上述近常量时钟差和官方首点语义支持；
尚未取得GEODE发布重定时实现的源码级认证，本轮没有用该候选做Deskew或修改任何时间戳。
原inspection里raw-timebase覆盖为0是**不同时间原点比较的条件结果**，不是证明真实IMU缺失。

按header锚定的条件覆盖：Xsens可包围5427/5427个完整扫描区间；
内置IMU可包围5425/5427，缺首帧1及末帧5427的严格时间边界。
两路IMU时间均严格递增，最大间隔分别15.870094／5.109549ms。
本轮没有选择任何一路IMU进行Deskew，因为外参和地图尚未准入。

## 4. 真实Native NDT执行

仅`RAW_NATIVE_LIDAR_GEOMETRY_DIAGNOSTIC`，不是先验地图定位精度。
Target严格使用k−1帧，Source使用k帧；每对独立identity初值，无IMU、GT或历史位姿引导。
沿用冻结Source0.25m体素、0.5–80m范围、1400点上限；Target0.15m体素两次。
没有新增FoV裁剪、方向搜索、Source点数调节、Coupled或Recovery。

NDT：resolution0.8m、step0.08、epsilon0.001、max_iterations40、outlier0.55。
使用R5修正的Target初始化顺序，直接检查target_cells_三个轴：
float32(0.8)=0.80000001192092896；**360条实际网格审计全部通过**。

| 阶段 | 输入对数 | align实际调用 | 收敛 | 有效／可比较／SPD Hessian |
|---|---:|---:|---:|---:|
| 连续Smoke，Source2–21 | 20 | 20 | 20/20 | 各20/20 |
| 全段时间均匀筛查 | 100 | 100 | 100/100 | 各100/100 |

实际合计120align，另120次只读native jet；119个不同扫描对，pair2在Smoke与筛查各运行一次。
重叠pair2的输入哈希、NDT矩阵、Fitness及迭代次数完全一致，没有额外重复实验。
筛查Source索引2至5427；Source时间跨度542.499870777秒，100个预冻结样本，不是全5427帧定位。
Source全部1400点；Target5504–12709点，中位数11134。

Hessian使用冻结chart：`p=p0+0.8η_t[m]`、`R=Exp(η_r[rad])R0`，
原生平均负score的完整非线性拉回。它不是协方差／Fisher信息保证。
120矩阵NumPy独立复核：最大相对特征值差9.7381e−13，最大弱向量相对残差3.0152e−16。
输出先冻结，后进行分析；没有GT读取、轨迹拟合或精度筛选。

## 5. 实际弱约束证据及限制

历史Ground只读取P60-R5冻结FULL_360新共同位姿CSV，没有重新运行Ground-Challenge。
以下为中位数，使用同一归一化chart，但跨场景／传感器比较不是严格因果消融。

| 指标 | GEODE原生Avia，100对 | 冻结Ground FULL360，100对 |
|---|---:|---:|
| 6DoF λmin/λmax | 0.000784695 | 0.008274522 |
| 6DoF条件数 | 1274.380 | 120.853 |
| 平移3×3 λmin/λmax | 0.366313 | 0.436616 |
| 平均能量λmin | 1.389561 | 2.955768 |
| Gaussian邻域支持率 | 78.179% | 81.679% |
| 完整6D弱向量平移分量主轴X/Y/Z计数 | 1/68/31 | 28/0/72 |

GEODE **94/100**满足单独6DoF谱比值≤1e−3分量；完整联合判据仍为：
SPD且6DoF比值≤1e−3 **且平移块比值≤1e−2**，结果**0/100**。
实际平移块比值最低0.269403，未接近该冻结门槛；没有改阈值让样本变“严重退化”。

须区分两种方向：完整6D弱向量的平移部分主要scanner Y（68/100），
而**独立平移3×3子块**最弱轴主要scanner X（99/100），二者不是同一个量。
完整弱向量包含旋转分量，不能把它直接描述为纯隧道轴向平移漂移。
全部方向属于前一扫描Target的published scanner坐标，不是已认证机器人／隧道／世界轴。

连续Smoke的单独6D低比值观测：索引5–13，9个连续扫描，0.799891秒；
索引16–21，6个连续扫描，0.500013秒。这些仅是判据分量，不是严重联合标签。
完整严重判据没有持续区间。全段100对之间未计算的扫描不能被说成持续退化。

因此：**相对六维低曲率各向异性更明显，有真实原生观测证据；
尚无完整冻结判据下严重平移退化、错误匹配、错误盆地或定位失败证据。**
无合法GT／地图坐标合同，不报告绝对RMSE、RPE或由稀疏对拼成的伪全局轨迹。

## 6. 运行资源

100对筛查align均值8.744ms、P95 14.294ms、最大22.401ms；Target构建均值0.782ms。
Fitness中位数0.067656m²、P95 0.136918m²、最大0.165953m²，不能当作位姿误差。
整段离线探针wall1.85秒，峰值RSS30456KiB（29.742MiB）。
20对Smoke wall0.34秒、峰值30540KiB（29.824MiB）。
这些不是生产ROS NDT节点、完整先验地图或融合系统资源，不宣称部署内存／10Hz实时性通过。

## 7. 融合、地图及收尾

实际bag只有5个传感器话题，没有map/TF/odometry；不能从同名frame反推单位阵外参。
R1公开地图目录只发现stairs.las，没有可认证盾构隧道地图。
没有可信全局轨迹／独立建图资产可证明分离建图段和测试段的空间重叠，未用GT选择分段或建图。
没有创建自地图或将局部前一扫描Target称为独立先验地图。

R1 gamma_config的Ouster话题、16线设置与实际CustomMsg、line0–5不匹配，未直接使用其矩阵。
Avia–Xsens和Avia–内置IMU外参均未认证，T_map_lidar、T_map_imu和初始化未建立。
所以融合Smoke未获准入：Deskew=0、EKF=0、OOSM=0、Coupled=0。
详见MAP_AND_FUSION_STATUS.md、INPUT_CONTRACT.md和time_contract_audit.json。

13项Python测试及1项实际网格C++测试通过；所有大数据、完整字段NPY及二进制留内盘。
本轮已停止。唯一NEXT：请科研总控核查并验收真实Native结果，
再决定是否单独下达融合准入合同；不自动开启Coupled或完整定位回放。
