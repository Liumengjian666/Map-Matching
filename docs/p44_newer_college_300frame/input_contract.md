# P44 输入合同

## 数据和选择

- 数据集：Newer College 2020，`01_short_experiment`。
- 数据根目录：`/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment`。
- LiDAR：官方 Ouster 单帧 PCD；按文件时间戳解析并升序选择，不读 bag record time。
- IMU：官方 Ouster `raw_format/ouster_imu/data.csv`；真实 sec/nsec、角速度和加速度。
- 先验地图：官方 5 cm PLY 无损 XYZ 转换的 PCD；运行时坐标未变。
- GT：`ground_truth/registered_poses.csv`，仅第一条位姿预先固化到初始化 YAML，全部轨迹在运行结束并冻结输出哈希后由离线脚本读取。

精确选帧合同：起始扫描为 `1583836591182590976 ns`，第 300 个扫描为 `1583836621082438144 ns`。两者之间为 300 个唯一、递增的规范时间戳；时间差范围为 99.722240–100.219904 ms，没有大于 101 ms 的间隔。目录共有 15,301 个规范时间戳；另有一个带 `(1)` 后缀的非规范重复文件被忽略。完整逐帧路径、点数和运行后 SHA256 见 [input_scan_sha256.csv](input_scan_sha256.csv)。

每帧含 65,536 点，合计 19,660,800 点。原始 PCD 字段包括 XYZ、intensity、`t:uint32`、reflectivity、ring、noise、range。适配器保留 XYZ、强度、ring 和原始 `t`，仅新增现有 ROS Deskew 接口要求的 `time:float32 = t × 1e-9` 秒字段；原始文件不修改。本次实测每帧最大点时间偏移范围为 99.626496–100.122368 ms。

IMU 边界样本为 `1583836591178606952 ns`，比首扫描早 3.984 ms，与 P41 冻结初始化一致。发布器逐扫描发送真实、严格递增 IMU，验证相邻输入最大间隔不超过 20 ms，并等待 EKF 和 NDT 的消费水位达到扫描末点后的括号样本，再发布点云。300 帧的 Deskew 均为 `PUBLISHED`；`post_scan_end_imu_used=0`、`current_scan_ndt_leakage=0`。Deskew 每帧使用 10–11 个历史状态样本，共 3,001 个。

## 冻结输入文件

| 资产 | 路径 | SHA256 | 说明 |
|---|---|---|---|
| 官方 5 cm 地图 PLY | `prior_map/new-college-29-01-2020-5cm-resolution.ply` | `e8bf6c7f2ddacde4f6f65ba7759b00bdca5c6bf5e706344ce8c3592529f04ab0` | 10,277,007 点 |
| 无损 XYZ 运行 PCD | `/media/jian/HIKVISION/paper rosbag/NewerCollege/.p41_first_ndt/prior_map_xyz_binary.pcd` | `57da358a7c3f52a132fd06afba69f2c3eff2e980b7dbad5b8751548665104e25` | 10,277,007 点，369,976,348 bytes；转换前后 XYZ 不变 |
| GT CSV | `ground_truth/registered_poses.csv` | `b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e` | 运行后评价；P43 清单与当前文件哈希一致 |
| Ouster IMU CSV | `raw_format/ouster_imu/data.csv` | `b733ce23544ed23a3ab7a627acfd61ea329f882b12ef0cbcb501b6b13e5828be` | 原始传感器测量 |
| Camera–IMU 标定 | `calibration/kalibr_output/cam-ouster-imu/camchain-ouster_imu-cam-rooster_2020-03-11-10-05-35_0.yaml` | `3bc13fa24af9ef8ca888c5523b1df57d1c53d0ca4052b818b7e02310fa781feb` | P43 条件性 Base→LiDAR 链中的 `T_C_I` |
| LiDAR–IMU 标定 | `calibration/kalibr_output/ouster_imu_lidar_transforms.yaml` | `8fe56ff808c6c900db52ac1545819ef3da979818bfa392ba00aef43daf59a037` | P43 条件性 Base→LiDAR 链中的 `T_I_L` |

外盘为 HIKVISION（UUID `F627-4133`），本次验证时挂载正常、可写，剩余约 432 GiB。运行输入与输出全在外盘；仓库仅保存小型复现文件、CSV 结果和图表，不保存原始 PCD、地图或 GT。
