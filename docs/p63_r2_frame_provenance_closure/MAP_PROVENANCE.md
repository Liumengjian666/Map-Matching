# 指定simtime-rebuild PCD生产身份

## 已有原件

目标：`/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd`

7346648 bytes、459154点、XYZRGB binary，SHA256 `016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca`。**CONFIRMED**，与R1冻结相同，原件未改。PCD本身没有FAST-LIVO2代码版本、有效参数、地图frame名或世界重力认证信息；VIEWPOINT单位姿态不能弥补缺口。

同目录状态：

| 文件 | 实际证据 | 等级 |
|---|---|---|
| `pcd/lidar_poses.txt` |6615行；首1786179479.946594，首姿态近I；与源码写IMU状态相符 |SUPPORTED：邻接同生产系列；不是外部GT |
| `logs/mat_pre.txt`, `mat_out.txt` |与pose首位置、早期姿态对应，原mtime同为8月12日15:14:49附近 |SUPPORTED：同运行关联；未含gravity分量 |
| `pcd/rebuild_loop2_metrics.json` |明确dataset名、6615poses、duration1089.7847829s |SUPPORTED：同名派生总结；不是生产配置 |
| 同名前缀raw PCD及红色轨迹PCD |存在且mtime相近；只做目录元数据检查 |SUPPORTED：邻接派生产物，未重新生成／全量hash |

mtime、路径、命名只能支持关联，不能证明没有命令行覆盖或保存后坐标变换。轨迹文件的名称也不能证明其参考点是LiDAR；核查源码按_state.rot_end/pos_end输出，是IMU状态。

## 较早配置能证明什么？

用户提供的8月8日目录中，`config/loop2_config.yaml`确实存在，SHA `1cde331d3b092629e1c2b06ebf218a9c51122fe4f8feeef3ea6e9ef2fafae76c`，与R25快照一致。它写E=I4、gravity_align_en=false，**CONFIRMED仅限文件内容**。launch指向该系列配置并启动fastlivo_mapping；仍没有指定simtime-rebuild的有效参数快照。

该目录`dateset/mapping-bag/logs/fastlivo_loop2.log`包含：

- 第43行初始化重力[3.9234,-0.2524,-8.9878]。
- 第303598行保存`.../outputs/loop2/pcd/loop2_all_raw_points.pcd`，45295762点。
- 第303599行保存`.../outputs/loop2/pcd/loop2_all_downsampled_points.pcd`，468495点。

故“这就是指定459154点、simtime前缀PCD的直接生产回执”是**CONTRADICTED**。这不证明两次参数一定不同，也不否认它们可能使用同一原始bag；只能拒绝将旧回执直接替代新回执。

## 有界历史追查

本地FAST-LIVO2源码链接解析到`/home/jian/FAST-LIVO2_project/FAST-LIVO2`，与旧launch源码路径相符。当前HEAD `0d2c0346107b75b59934975adec9a6eeeb913c64`，IMU处理及状态头文件与HEAD相同；LIVMapper和其他文件已有用户未提交改动。保留原状态；不能以当前源码HEAD假冒8月12日实际执行二进制。

当前可查Git里较早定位报告引用了同一PCD，但其性质是消费者记录，不是地图构建合同。ROS log树仅用精确map前缀查询一次，5个命中文件均是9月NDT/旧定位/DCReg加载或启动记录；shell历史命中为pcl_viewer查看命令。原先`/home/jian/rosbag/loop2_simtime_rebuild_2026_08_12_001`目录现不存在。具体范围、命中摘录和缺失项见`PROVENANCE_SEARCH.json`，没有复制环境转储或扫描全部个人历史。

**UNVERIFIED：指定PCD生产身份未完整闭合。** 缺少：该次启动命令＋全部override、有效参数dump、确切代码／二进制身份、IMU初始化及保存重力状态、若存在则保存后变换／搬运记录与hash链。未找到不等于断言记录在所有位置都不存在；本轮有界核查到此停止。

为避免继续无关搜索，下一次只请求上述生产记录。不能因旧配置名字相似便宣布同一，也不能用NDT/GT误差反求一个“应有”地图变换。
