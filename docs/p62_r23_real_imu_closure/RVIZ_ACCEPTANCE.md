# R23 一键RViz真实显示验收

真实可视化目录：
`/home/jian/livox_ws/p62_r23_real_imu_data/rviz_baseline_20_01/`。

本次GUI采用W1/C0、BASELINE、20个评价扫描；运行命令及实际二进制身份在该目录`VISUAL_COMMAND.json`。这是独立计费的显示验收回放，不并入正式10条C/D对照精度。实际新增20次名义Full NDT、0次额外搜索；20次观测发布、20次EKF校正ack、20条OOSM `APPLIED`均由真实CSV记录确认。

## 一键入口

默认参数为`MULTISTART_C`、W1/C0、60扫描、`rviz:=true`，D关闭，匿名新输出目录。桌面终端可复制以下单行：

```bash
source /home/jian/livox_ws/p62_r23_real_imu_closure_ws/devel/setup.bash && ROS_MASTER_URI=http://127.0.0.1:11383 ROS_IP=127.0.0.1 roslaunch dog_prior_map_localization p62_r23_integrated_localization.launch strategy:=MULTISTART_C rviz:=true
```

该命令复用已构建的研究NDT/EKF节点、真实PCD/IMU发布器和已有P45地图管理路径。它不修改稳定机器狗工程。切换为BASELINE或风险已知的D，必须显式设置`strategy:=BASELINE`或`strategy:=RELATIVE_D_EXPERIMENTAL`并从独立新状态运行。窗口、时间和初始配置成套参数见[README.md](README.md)。文档交付后执行端不自动重复运行。

## 预配置显示与实际订阅

配置文件：`src/dog_prior_map_localization/rviz/p62_r23_integrated_localization.rviz`；Fixed Frame=`newer_college_map`，视角目标=`os1_lidar`。配置包括8个数据Display和TF，无需用户手动添加：

|显示|实际话题|颜色/含义|
|---|---|---|
|Prior Map|`/newer_college/prior_map`|灰色官方Leica地图，仅可视化|
|Local NDT Target|`/newer_college/local_ndt_target`|蓝色当前实际局部匹配目标|
|Deskewed Current Scan|`/newer_college/deskewed_points`|黄色真实Deskew扫描|
|NDT Aligned Scan|`/newer_college/points_aligned`|红色匹配后点云|
|NDT Pose|`/dog_livo/ndt_pose`|地图观测坐标轴|
|NDT Path|`/dog_livo/ndt_path`|绿色NDT地图观测轨迹|
|EKF High-Rate Path|`/dog_livo/path_high_rate`|蓝色EKF高频轨迹的采样Path显示|
|EKF Corrected Path|`/dog_livo/path_corrected`|橙色EKF校正轨迹|
|TF|`/tf`、`/tf_static`|实际地图/LiDAR坐标关系|

该目录`RVIZ_NODE_INFO.txt`记录RViz节点PID、8个数据话题及TF的真实TCPROS连接。截图`RVIZ_ACCEPTANCE.png`显示`Global Status: Ok`、先验地图与当前扫描，不是只启动一个空RViz窗口。可查看[实际截图](visual_acceptance/RVIZ_ACCEPTANCE.png)（完整绝对路径如上）。截图不能替代时序、OOSM或精度验收；对应证据在发布器、NDT/EKF和Deskew日志。

首次GUI启动出现Noetic EOL提示对话框；执行端向该对话框发送`WM_DELETE`后获得截图，保留原始`RVIZ_INITIAL.png`与操作事实。当前launch为RViz节点设置`DISABLE_ROS1_EOL_WARNINGS=1`；该环境变量名称已从本机`librviz.so`字符串核实，用于避免用户再次手动关闭同类对话框。此次环境设置只影响启动提示，不改变算法、地图、传感器数据或显示项目。首次实际显示验收包含这一次UI操作，不能倒写成首次全程零交互。

## 计费和适用范围

|部分|实际Full align|extra|用途|
|---|---:|---:|---|
|原系统20帧Smoke|20|0|G1基础闭环|
|10条正式轨迹|432|12|420名义+12额外；600输入扫描中部分被原有优化前几何门拒绝|
|本次20帧GUI回放|20|0|仅可视化验收|
|合计|472|12|显示成本未隐藏|

GUI资源与墙钟不能替代正式headless策略对照成本。数据发布完成后保留RViz供查看，按Ctrl-C关闭本次launch；headless监督器另有完成回执后自动关闭自身ROS树的逻辑。

可视化已验证Newer College本地PCD/IMU输入；没有将其伪称rosbag。机器狗rosbag模式仅在README给出话题、点时间、外参、初始化与地图合同，未实际运行或验收。条件性Base和地图配准参考限制继续保留，GUI显示一致不能认证独立物理真值或论文创新。

最终补充：独立RViz环境变量检查没有生成完成回执，不能声明再次启动后的提示抑制已经实测通过。变量仅经本机库核实；实际截图是在首次手动关闭EOL提示后取得。原私有11383运行树已不在最终进程表中，未保留正常退出状态回执。实际图像、订阅、20次融合数据均已归档到`visual_acceptance/`。
