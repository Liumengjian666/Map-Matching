# P32-R5：Corridor02 GT机体／IMU坐标诊断

## 正式结果

`GT_ASSISTED_FRAME_HYPOTHESIS_NOT_SUPPORTED`。
固定5帧 × 2参考点 × 3世界转换的30组NN诊断已完成，所有组合的
0.5m、2m、8m覆盖率均为0，合理几何对应为0/5帧。
数值最好的H1/C2仍为54.551966m的五帧平均中位距离，不支持该假设。

`GT_REFERENCE_AMBIGUOUS`，并记录
`GT_ORIGIN_NOT_IDENTIFIABLE_FROM_GEOMETRY`：Body=IMU未获证明，
GT=LiDAR也未获证明。没有建立非GT初始化，没有启动正式定位。
这不证明数据、地图或官方YAML错误，也不排除未被本合同覆盖的合法坐标关系。

## Git与运行范围

Repository：https://github.com/Liumengjian666/Map-Matching

Branch：`research/p9-r4-heldout-visual-evidence`。
START：`a5798d3398503dd8ee7d6bcaf698735d1e1e8fa5`。
CODE：`c36f6aa0790f11bf6a8caf08321ec3751b4be7a8`。
END是包含本归档的提交，使用`git log -1 --format=%H -- docs/p32_r5_corridor02_gt_frame_test`
解析，并在聊天交接中提供完整SHA；不在提交内容中构造循环引用的自身SHA。
本轮没有执行push。远端最终查询结果在聊天回执中提供。

仅研究工作区新增离线工具和本报告；成熟定位源码、原始文件、标定与历史缓存未修改。
未写受保护的稳定机器狗和主论文工作区。不执行reset、clean、强推或历史覆盖。

## 实际输入

数据根：`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/`。

| 输入 | SHA256 |
|---|---|
| map/corridor02.pcd | 6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228 |
| gt/corridor02_gt.txt | cddb6739230ded86c57412e768e071e5cd6d62a5de5ff310618032ca2e38be0a |
| calibration/corridor02_extrinsics.yaml | 892bf743dc0f51eabfe9d8ae71dc98663753f174910ae9cc40ee244c77a9e217 |
| initial_pose/corridor02.yaml | fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1 |

raw/corridor02.bag实际读取，24494797838 bytes。本轮未重新计算整包SHA，
而是冻结实际消费的5条消息data SHA、有序XYZ SHA、header身份与提取源码SHA。
此为有界输入追溯，不冒充整包的密码学认证。详见`input_freeze.json`。

GT实际5522条，TUM xyz/qxyzw；支持区间
1645999726.984117–1646000619.653331秒。
[官方SuperLoc页面](https://superodometry.com/superloc.html)支持格式和RC1场景信息，
但未提供足以认证GT参考点的exporter证据；证据等级见`GT_SOURCE.md`。

## 扫描与时间

| 原始帧号 | header uint64 ns | RAW有限XYZ点数 |
|---|---|---:|
| 8 | 1645999726984117031 | 28152 |
| 9 | 1645999727084924936 | 28128 |
| 10 | 1645999727185785055 | 28150 |
| 11 | 1645999727286644936 | 28142 |
| 12 | 1645999727387505054 | 28137 |

原1–7帧早于GT，scan3没有被外推。scan8比GT首时刻晚31ns。
五帧完整scan end也在GT支持内；Decimal秒转整数ns，不用bag record时刻匹配。
位置线性插值，归一化xyzw quaternion做shortest-arc Slerp。
这是获准的离线GT插值，不声称GT在历史实时决策时刻可用。

本轮使用RAW，有限XYZ剔除数为0；所有组合共享相同有序点集。
GT只在header时刻刚性变换整帧，不使用GT逐点deskew，不称为在线NDT Source。
raw扫描未消除的运动畸变是本实验限制，不追加运动补偿以争取通过。

## 矩阵与诊断

记T_target_source。H1：T_ML=C*T_WGT*T_I_L；H2：T_ML=C*T_WGT。
C0=Identity，C1=active T_yaml，C2=实际4×4 inverse(T_yaml)。
没有读取注释候选、额外Yaw或平移；未修改原舍入矩阵。
RC1 laser_to_imu的原矩阵作为T_I_L，方向有schema及既有工程使用支持；
此依据不等同于GT exporter参考点认证。

T_I_L的平移为0，旋转约+Y 0.0095rad，即0.544309906°。
H1/H2同C下原点差严格0。静态矩阵见`world_matrices.json`；
全部30个实际T_ML、变换后bbox和逐帧统计见`geometry_comparison.csv`。
不把GT世界W、YAML所名darpa世界D、PCD世界M和运动机体B混用。

## 六组合结果

下表是各组合五帧的逐帧中位距离/P90的算术平均，单位m，
不是合并所有点的总体分位数。完整逐帧结果不筛除，均已归档。

| 假设 | C | mean NN median | mean NN P90 | 0.5/2/8m覆盖率 | 合理帧数 |
|---|---|---:|---:|---|---|
| H1 GT=IMU/Body | Identity | 194.770120 | 197.192053 | 0/0/0% | 0/5 |
| H1 GT=IMU/Body | T_yaml | 295.643768 | 298.543762 | 0/0/0% | 0/5 |
| H1 GT=IMU/Body | inverse(T_yaml) | 54.551966 | 56.685265 | 0/0/0% | 0/5 |
| H2 GT=LiDAR | Identity | 194.770096 | 197.191162 | 0/0/0% | 0/5 |
| H2 GT=LiDAR | T_yaml | 295.643823 | 298.542188 | 0/0/0% | 0/5 |
| H2 GT=LiDAR | inverse(T_yaml) | 54.552505 | 56.686883 | 0/0/0% | 0/5 |

各组合逐帧NN median范围依次为：H1 C0 194.764801–194.773865；
H1 C1 295.638428–295.646759；H1 C2 54.549908–54.554996；
H2 C0 194.764999–194.773407；H2 C1 295.638428–295.646790；
H2 C2 54.550163–54.555870。
这是错误几何对应在五帧中的稳定性，不是定位稳定性PASS。

配对H1/H2中位距离差最大0.000873566m，P90差最大0.003997803m。
它们的零杆臂和小旋转，以及目前所有世界假设均不形成对应，
使本次几何不足以辨识GT参考点，不能挑选细微较好分数作为Body=IMU证据。

NN沿用PCL float变换、0.20m target voxel和KdTree。
代码的median取排序数组`count/2`（偶数点数的上中位数），P90取
`floor(0.9*(count-1))`，未插值；这与本轮冻结的既有几何实现一致，
不是声称采用所有统计库默认的分位数定义。

## 验证、预算与成本

独立CMake Release编译通过；GNU9.4/PCL1.10/Eigen3.3.7。
时间、q/-q、无外推、零杆臂、外参方向和真逆的定向数学测试通过。
数学测试调用2次，其中一次随正式runner输出完整log；不是2次几何实验。
实际执行1次输入准备、1次NN程序；5帧、30组，ICP=0，NDT=0，ROS定位=0。
退出码0，target voxel后2378870点，无非有限结果。

NN程序（包括加载地图、voxel、建树、30组NN）的CPU 2.81s（user2.40/system0.41），
wall2.82s，peak RSS334776KiB=326.929688MiB。
30组NN行内计时合计1523.083559ms，计时覆盖循环、bbox和排序。
输入准备的完整GT/Bag读取及哈希成本未单独计时，不是免费输入。
本轮不测在线相机或机器狗部署，不宣称10Hz/100MiB。

运行离线二进制SHA256：
`69a1fb667c969f8e4fdfa9e0e3f85d844884843ae279922b5db532cf3490212e`。
源码、实际输入、输出和二进制已绑定；未对全体共享库做密码学认证。
逐帧统计、原NN执行记录和成本记录均保留，未伪造未执行分支。

## 复核与唯一NEXT

已完成一次当前独立只读复核；外部CLI经本轮限定输入和精确命令单独确认，
只允许调用一次。实际返回与逐项处理记录见`INDEPENDENT_REVIEW.md`及
`external_review/`；复核不增加实验预算或改变已冻结诊断。

唯一NEXT：取得带来源的Corridor02 GT导出参考点与发布PCD世界坐标连接说明。
不自动启动新的变换搜索、GT辅助运行初始化或正式定位。完成后停止。
