# P63-R3：官方 HDL Localization 持续 NDT 候选

**A：HDL_CANDIDATE_PASS。** 唯一一次正式运行完成前400扫描、400次原生NDT，400次通过预注册的外部有效更新筛选；目标地图始终冻结，连续失配0，未触发预注册异常跳变。此PASS仅表示当前无IMU的候选基线链路可用，不证明真实定位精度、退化鲁棒性、完整序列稳定性或论文创新。

**资源另判：**定位manager峰值约150.32MiB，通过200MiB筛选；实测完整回调P95 **105.994585ms**，没有达到低于100ms的目标。不能据平均值或纯NDT时间宣称10Hz通过。没有合法独立GT，不报告正式ATE/RPE。

|身份|实际值|
|---|---|
|START_SHA|`837721616755bc74c6bbec8896dec5f90c508a7c`|
|实际正式运行CODE_SHA|`eb6ff37ab791193ff1a803952d79e148b9ecea6c`|
|END_SHA|包含本归档的最终commit；完整SHA见最终交接／git rev-parse HEAD|
|分支／工作区|`research/p63-r3-hdl-ndt-prior-map` / `/home/jian/livox_ws/p63_r3_hdl_prior_map_ws`|
|官方HDL SHA|`de2dc7769aa2412876d05ae6161408d3295b6a3c`，BSD-2-Clause|
|作者核心／依赖补丁|均0，独立干净副本，Release构建；没有使用旧依赖目录的链接补丁|
|正式运行|`/home/jian/livox_ws/p63_r3_hdl_data/trial_01`；只有1次真实传感器试验；另有零传感器加载检查与封存结果显示，均0 align|

## 完整结果

|量|实测|
|---|---:|
|发布／桥接／完整回调／原生NDT|400 / 400 / 400 / 400|
|收敛／有效更新筛选|400 / 400|
|最长连续无效更新|0|
|滤波位置最大单帧增量|0.094926m|
|滤波旋转最大单帧增量|5.347057°|
|原始PCD点数／原生0.1m处理后target|459154 / 424323|
|target几何SHA|`d4ae17e75e22b2c660df1ca08140ad3675479c79f214da6253706c239ca4325e`，400次逐点检查未变|
|原生inlier fraction Mean／Min|0.906791 / 0.842277|
|NDT compute Mean/P95/Max|33.827908 / 90.752905 / 166.655089ms|
|完整回调 Mean/P95/Max|50.659627 / 105.994585 / 181.915786ms|
|定位manager保守峰值RSS|153928KiB =150.320313MiB|
|完整自有实验进程树同时峰值|865516KiB =845.230469MiB，含输入/记录器和RViz；不称整机内存|
|总墙钟／源扫描时间区间|88.318192s / 40.000008499s|

初始人工T_map_lidar=I4，来源为历史稳定机器狗NDT的零位置／单位旋转起步定义；首个滤波位置[-0.0171542,-0.0068812,-0.0078001]m，最后[3.0908148,0.4689392,2.3606553]m。完整xyzw及每帧增量见TRAJECTORY.csv，原生NDT终值和迭代／fitness见NDT_RESULTS.csv。这里的Z是PCD数字坐标，不是已认证竖直高度。

## 为什么本轮没有使用未验证物理外参

use_imu=false，use_global_localization=false，robot odometry prediction=false；状态直接表示原始LiDAR frame在PCD数字地图中的姿态，odom_child_frame_id等于输入livox_frame，不经过IMU。没有将I=L作为物理标定，没有重力对齐或UTM平移；convert_utm_to_local明确false。R2的物理／生产缺口仍保留，只是不参与当前无控制输入的UKF传播。

作者连续使用NDT_OMP＋UKF，原生参数完全保留：resolution1m、DIRECT7、epsilon.01、构造器max35/step.1/outlier.55、scan/map voxel.1m、cool_time2s。本轮没有C+G、额外搜索、Split、EKF替换或新恢复模块。原UKF每帧correct并无研究质量拒绝门；“有效更新筛选”只用于外部统计及停止整次试验，不修改其反馈。

## 可视化与限制

原正式运行启动了RViz，但截到的是Noetic提示对话框，不能当成地图显示验收。随后仅增加RViz进程级提示关闭参数，并用400帧封存轨迹＋第400扫描／记录终值做只读显示核验；只加载地图server，没有localizer，新增NDT/UKF更新0。`FROZEN_OUTPUT_PREVIEW.png`是封存输出预览，不是第二次在线定位。最终一键live入口已提供，GUI修正后未重跑400帧；详见RVIZ_ACCEPTANCE.md。

本轮不对旧算法做精度排名，不把可视一致性、converged或inlier当真值。目标PCD文件身份可靠，但生产及物理标定依旧未全认证。输入未Deskew，每个点只是保留原坐标；header用原scan-start，作者忽略点内offset。回放采用上一回调完成才发下一扫描的离线握手，不能据此证明实时调度。

唯一NEXT：请总控根据候选PASS、周期筛选失败和参考缺口，决定是否批准同参数更长／完整序列的基线验收。本轮不自动开始新算法、IMU接入或其他框架。
