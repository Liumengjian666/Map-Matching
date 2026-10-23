# P62-R18 FEE Corridor 资产准入报告

**结论：BLOCKED。** 本轮只核查 FEE Corridor 的公开资料和有界小文件；没有下载完整数据集、构建地图或运行 NDT。阻塞集中在独立参考位置能否可靠转换到 LiDAR 地图系和扫描时刻，见 [坐标与时间合同](FRAME_AND_TIME_CONTRACT.md)。

## 研究起点与可复用代码

- 研究仓库：`https://github.com/Liumengjian666/Map-Matching`；R17 本地与远端 `research/p62-r17-consensus-admission` 均为 `cae5b8dbaeb5f7d76ddd75de5c6f798d94165c16`，工作树干净。R18 从该提交建立独立工作区 `/home/jian/livox_ws/p62_r18_failure_benchmark_ws`、分支 `research/p62-r18-independent-failure-benchmark-admission`。
- 已阅读 R16/R17 报告和 R17 `results.json`、`tools/policy.hpp`、`tools/run_policies.cpp`。R6 `tools/native_kernel.hpp` 中 `NativeNdt` 继承 PCL `NormalDistributionsTransform`，配置为 resolution 0.8 m、step 0.08、epsilon 0.001、40 次迭代、outlier ratio 0.55。R14B `common.hpp` 的因果 `Predictor`、R9 Coverage V2 和 R10 观测准入可复用；FEE 的点云格式、地图系、传感器外参和时间读取器尚需另行适配，不能把 Newer College 输入代码直接视为通过。
- R17 的 30 帧中，5° 基线接纳 29 帧，6° 和事件一致性各接纳 30 帧；后二者输出位姿全 30 帧完全一致。该窗口没有建立错误 NDT 吸引域，也不用于本轮 FEE 选窗。历史详情见 [R17 报告](../p62_r17_consensus_admission/REPORT.md)。

## 官方来源与有界资产观察

官方研究代码固定查询提交为 `dc78edad1cc08af2de333e2c7ec4e05f7bb213a2`。论文明确说明：Ouster OS0-128 扫描、单独用 Leica BLK360 构建的地图、地图对齐 SLAM 生成的六自由度位姿，以及 Leica TS15 跟踪机器人棱镜得到的位置。后者不含独立姿态；姿态来自 LiDAR 对该地图的配准。[官方论文 IV-B](https://arxiv.org/pdf/2303.01123)、[官方仓库](https://github.com/ctu-vras/depth_correction)。

官方数据目录 `http://ptak.felk.cvut.cz/vras/data/fee_corridor/` 可按文件访问。原始采集包 `bags/seq1.bag` 的 HEAD 返回 HTTP 200、17,922,456,346 B，`bags/seq2.bag` 为 21,404,429,889 B；另有较小的序列派生包 `sequences/seq1/data.bag`，返回 228,005,307 B。`maps/pcd/map_5cm_alligned.pcd` 返回 HTTP 200、7,582,206 B。ZIP 返回 HTTP 200、39,247,580,582 B，支持 Range。通过有界读取 ZIP64 尾部和中央目录，发现 727 项、成员解压大小合计 51,320,290,669 B。**完整 ZIP 加解压至少约 90.57 GB，不适合本轮空间与 20 GiB 安全余量；官方直链允许未来按需获取单一原始 bag 或小资产。** 实际未下载 bag 或地图。[官方 README](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/README.md)。

有界读取还确认 `seq1/poses/poses.csv` 为 119 条地图配准位姿，`seq1/poses/leica_poses_raw.txt` 为 119 行纯 XYZ；`seq1/calibration/map2subt.txt` 为一个 4×4 数值矩阵。中央目录显示预提取 `seq1/ouster_points/` 和 `static_ouster_points/` 均有单帧文件。单独取官方直链 `1669300145_103033600.npz`，4,570,825 B，SHA256 `395a6a80428231e9ae24e54cd1407e0f5d4dcf3cd2260c4eeb816607a841631b`；其 `cloud` 为 131,072 个结构化点，含 float32 `x/y/z` 和 uint32 `t`。`t` 的单位没有获得认证。该直链文件与 ZIP 内同路径条目的解压大小 1,606,105 B **不一致**；两种官方交付的 payload 身份不可混用，原因未查明。该小样本只用于格式证明，未参与定位。

对 `/home/jian/rosbag` 与 `/home/jian/livox_ws` 的限深定向搜索未发现既有 FEE Corridor 数据；这不等于全盘无文件。检查时文件系统可用 `80,539,389,952` B，内存可用约 `7,415,955,456` B。上限允许按文件获取小规模样本，但实际 PCL 内存、运行时间和地图解码尚未实测。

## G1—G6 判定

| 门 | 判定 | 实际证据与边界 |
| --- | --- | --- |
| G1 原始 LiDAR | `PASS_AVAILABLE_NOT_ACQUIRED` | 官方 `bags/seq1.bag` 原始包直链返回 200/17,922,456,346 B；序列派生 `data.bag` 为 228,005,307 B；单帧 Ouster NPZ 已作格式核查。未下载或解析原始 bag，完整载荷身份尚未冻结。 |
| G2 独立先验地图 | `PASS_PROVENANCE_NOT_ACQUIRED` | 论文说明 BLK360 是独立三脚架扫描；官方 5 cm PCD 直链可访问。未下载、哈希或验证其坐标内容。 |
| G3 参考来源与自由度 | `BLOCKED_FOR_INDEPENDENT_LIDAR_POSE` | TS15 独立测得棱镜 **位置**，未测姿态；发布六自由度位姿由测试 LiDAR 对 BLK360 地图配准得到，不能作为独立六自由度真值。棱镜位置转 LiDAR 原点还缺数值杆臂。 |
| G4 地图与参考坐标 | `BLOCKED` | `map2subt.txt` 提供数值候选，但 seq1 launch 同一变换处有未解决的 `TODO`；缺少其独立测定/验证来源及棱镜到 LiDAR 的转换链。不能据此认证地图系独立位置误差。 |
| G5 时间与外参 | `BLOCKED` | 原始 Leica XYZ 表不含时间；导出程序用 bag 记录时间最近邻匹配，允许至 1.5 s，并丢弃被选记录的时间/frame_id。点内 `t` 单位及棱镜外参未认证。 |
| G6 资源 | `NOT_EVALUABLE_RUNTIME; SELECTIVE_STORAGE_FEASIBLE` | 单独获取 17.92 GB 的 seq1 原始 bag 与约 7.6 MB 地图在当前空间下有望保留 20 GiB 余量；完整 ZIP 加解压不满足该余量。尚未实测地图加载与 NDT 内存/时间，不能写成 G6 整体通过。 |

**总状态：BLOCKED。** G3—G5 未闭合，故不能把该数据直接用于独立 NDT 失败精度实验。没有以同地图 LiDAR 配准姿态代替独立参考，也没有用测试帧拟合变换。

## 直接证据

- [官方论文：数据采集与参考资格](https://arxiv.org/pdf/2303.01123)
- [官方 dataset reader：`poses.csv`、Tracker XYZ 与 `map2subt`](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/src/depth_correction/datasets/fee_corridor.py)
- [官方提取程序：PointCloud2 与 Tracker 的时间选择](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/scripts/create_fee_corridor)
- [官方 seq1 launch：`map subt` 与待确认注释](https://github.com/ctu-vras/depth_correction/blob/dc78edad1cc08af2de333e2c7ec4e05f7bb213a2/launch/slam_fee_corridor.launch)

**唯一 NEXT：** 向数据作者或其正式资料取得棱镜到 LiDAR 的数值杆臂、`map2subt` 的来源/方向与有效性，以及原始 Tracker 每条记录的时间基准与对应容差；随后由科研总控决定是否开展有界准入复核。当前停止。
