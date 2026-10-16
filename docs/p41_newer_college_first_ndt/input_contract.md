# 输入合同

## 冻结文件

数据集根目录：`/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment`

| 内容 | 路径 | SHA256 |
|---|---|---|
| 单帧 Ouster PCD | `raw_format/ouster_scan/cloud_1583836591_182590976.pcd` | `7c2fc3a20ef266c2ac3196173c534a1acdb4436224c264bc8f2dc1b1909803a2` |
| Ouster IMU | `raw_format/ouster_imu/data.csv` | `b733ce23544ed23a3ab7a627acfd61ea329f882b12ef0cbcb501b6b13e5828be` |
| 官方 GT CSV | `ground_truth/registered_poses.csv` | `b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e` |
| 官方 5 cm prior PLY | `prior_map/new-college-29-01-2020-5cm-resolution.ply` | `e8bf6c7f2ddacde4f6f65ba7759b00bdca5c6bf5e706344ce8c3592529f04ab0` |
| PLY→PCD 无损 XYZ 转换 | `/media/jian/HIKVISION/paper rosbag/NewerCollege/.p41_first_ndt/prior_map_xyz_binary.pcd` | `57da358a7c3f52a132fd06afba69f2c3eff2e980b7dbad5b8751548665104e25` |
| Ouster LiDAR–IMU 标定 | `calibration/kalibr_output/ouster_imu_lidar_transforms.yaml` | `8fe56ff808c6c900db52ac1545819ef3da979818bfa392ba00aef43daf59a037` |
| Ouster IMU–RS_C1 标定 | `calibration/kalibr_output/cam-ouster-imu/camchain-ouster_imu-cam-rooster_2020-03-11-10-05-35_0.yaml` | `3bc13fa24af9ef8ca888c5523b1df57d1c53d0ca4052b818b7e02310fa781feb` |

来源和文件完整性以 P40 下载清单为准；本轮重新计算了所用扫描、IMU、GT、PLY、PCD 的 SHA256。没有重新下载数据，没有读取未完成的双目图像。

## 点云与时间

输入 PCD 为 1024×64（65,536 点），字段含 XYZ、intensity、`t:uint32`、reflectivity、ring、noise、range。适配器不改 XYZ 或原始 `t` 字段，新增既有 Deskew 订阅接口要求的 `time:float32`，其值为 `t × 1e-9` 秒。扫描内 `t` 范围为 0 至 99,904,768 ns。原文件不被修改。

IMU CSV 的时间由 `sec,nsec` 组成，数据是实际 Ouster IMU 角速度和加速度，单位分别为 rad/s 与 m/s²。发布器核验时间顺序并选择真实扫描前边界样本、扫描内 IMU 和覆盖最后点时间的后续边界样本；该帧实际发出 11 个样本，起始边界早 3.984 ms，最后用于区间覆盖的样本在扫描起始后 106.005 ms。Deskew 日志记 `post_scan_end_imu_used=0`：没有把扫描点结束之后的 IMU 用作点时刻状态。

本轮只发布一张真实扫描及其所需 IMU 子集，不构造完整 rosbag，不代表 15,301 帧已运行。
