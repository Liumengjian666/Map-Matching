# SubT-MRS 论文与 Corridor02 实际发布链核查

## 结论

`WHOLE_SEQUENCE_ALIGNMENT_UNRESOLVED`。本次没有获得可据以替换旋转/平移的
官方坐标定义。只修正显示配置中的结论标记，保留现有数值供检查；未修复完整
轨迹对齐，未启动定位、ICP、NDT 或 GT 拟合，未改原始数据和受保护工作区。

## 论文实际提供什么

- [SubT-MRS §3.1.1](https://arxiv.org/html/2307.07607v5#S3.SS1.SSS1)：传感器标定与同步方法。
- [§3.2](https://arxiv.org/html/2307.07607v5#S3.SS2)：参考地图来自 FARO，GT 轨迹由
  地图几何与多传感器约束生成；作者用 GT 变换扫描、重建点云来核验。
- [补充 B.3](https://arxiv.org/html/2307.07607v5#A2.SS3)：参考地图模型介绍，
  没有本地 YAML/PCD 导出坐标关系。
- [SuperLoc 官方表](https://superodometry.com/superloc.html)：Corridor01 为
  SubT-MRS/RC2，Corridor02 为 SuperLoc/RC1；官方发布 TUM 轨迹及初始化配置，
  但页面没有定义 `world_darpa` 为 GT-world 到原始 PCD XYZ 的固定外参。
- [官方 Issue #50](https://github.com/superxslam/SuperOdom/issues/50)：作者说明
  SuperOdom 的 world/sensor 帧约定。它不是 Corridor02 GT/PCD 导出变换的定义。

论文没有给出 `world_darpa`、PCD `VIEWPOINT` 或 Corridor02 文件的坐标链。
这不证明它们错误，只说明不能声称这些数值的含义已被该论文认证。

## 正确的坐标链与目前的假设

记 `T_target_source` 将 source 坐标转换到 target。
P 是 PCD 实际存储 XYZ；W 是 GT 世界；I 是假设的 GT/IMU 参考系；L 是 LiDAR。
若 GT 为 IMU 位姿，扫描应按下式变换：

```text
T_P_L(t) = T_P_W * T_W_I(t) * T_I_L
```

当前显示数值假设 `T_P_W = inverse(V) * Y`，其中 V 来自 VIEWPOINT，Y 来自生效
YAML。PCD 格式定义 V 为 acquisition viewpoint；格式本身不证明其是此文件的
地图导出位移。当前组合的首段几何支持不能代替完整导出语义或全程核验。

已排查的发布实现：TUM 四元数按 xyzw；GT 左乘上述固定显示变换；只发布
Path/Marker/info，不创建 TF 或估计器观测；地图发布器保留原始 XYZ。
发布器的矩阵逆变换与直接 `inverse(V)*Y` 计算一致，最大元素差 5.69e-14。

RC1 `laser_to_imu` 为 `T_I_L`：零平移、约 0.5443° 绕 Y 旋转。
因此，切换“GT=IMU/GT=LiDAR”会改变扫描朝向，但不会改变轨迹原点位置。
不能靠交换参考点解决路径位置下沉。VIEWPOINT 的旋转恰为单位阵，其平移
处理也不能自行补偿剩余俯仰误差。world 与首帧 LiDAR 重合仅是参考系定义，
不表示运动中每个 LiDAR 帧都与世界帧相同。

## 按论文思路执行的有限几何核验

未重建整图；将四个既有诊断时间的真实 raw 扫描逐帧变换后，与完整原始 PCD
做最近邻检查，未优化任何位姿。时间身份均为原始整数 header ns；GT 使用
既有线性平移/Slerp 插值，无外推。读取已准备 bag 的相同序列化点云，点集仅去除
非有限 XYZ。没有使用 GT 逐点去畸变；这不是历史运行的 NDT Source。
条件为 GT=IMU，外参使用 RC1，不把该条件当成官方证明。

| 相对 GT 首时刻/s | NN median/m | NN P90/m | ≤0.5m覆盖率 | 点Z−最近地图点Z中位数/m |
| ---: | ---: | ---: | ---: | ---: |
| 0.000000031 | 0.116021 | 0.243448 | 96.7143% | -0.003781 |
| 180.026506932 | 0.141405 | 0.309891 | 96.4106% | +0.002668 |
| 539.978649886 | 2.198896 | 2.546281 | 4.5438% | -2.182028 |
| 630.042285950 | 2.632660 | 3.049000 | 1.6599% | -2.407209 |

该诊断复现了后段几何不吻合，不是只观察 RViz。最近邻距离不是轨迹 ATE，
最近地图点也未必是同一物理表面；不能把表中 Z 差直接称为机器人高度误差。
raw 扫描有运动畸变，不能单凭这四帧确定剩余固定旋转、GT 导出问题、局部
地图差异或时变误差中的哪一个是唯一原因。未搜索或估计新的坐标变换。

## 我们此前哪里有问题

1. 最初未区分 PCD 原始 XYZ 与读取软件可能应用的 acquisition pose。
2. 用起点附近五帧的吻合，给出了过强的全程“修复”表述。
3. `T_world_gt` 是程序对数值的解释，并非官方字段含义得到证明。

本轮把这些解释明确降级为假设，而不是再把 GT 人为调平。没有证据证明原始
官方 R/t 错误，也没有证据证明完整 GT 正确。唯一下一步是取得 **Corridor02
GT 导出世界系到 PCD 存储系的确切定义或导出代码**（同时确认 GT 的传感器帧），
再核对本数值链。若只能从数据估计，应另行授权、标记为估计对齐，不能称官方外参。

## 修改与验证

仅更改 C02 显示 YAML 的注释/合同文本、README 和一项合同断言，新增本说明与
JSON 诊断；矩阵、话题、点云、原始 GT、定位算法均未变。13 项 GT 显示测试和
5 项 Corridor01 launch 测试通过，JSON 解析及 `git diff --check` 通过。
未重新编译 C++，未启动新的 ROS 回放或 GUI；之前的全程对齐声明已收回。
