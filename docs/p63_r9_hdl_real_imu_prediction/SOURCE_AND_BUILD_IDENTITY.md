# R9源码、编译、运行与封存身份

START `b2e2470c4cc9e89df70db73803a8df2ad994ced6`。
实际在线CODE `c03c58fc2bc2a1bf6fe4b802b76fe3b6aa673dd2`。
实际主要参考评价CODE `a8bf141f5c9dc07b7ce2605f56f9e1dce31dbd83`。
最终END为收尾归档commit，在最终交接给出完整SHA，不把它伪写成在线CODE。

原HDL de2dc7769aa2412876d05ae6161408d3295b6a3c；其余三上游SHA与六份原ELF SHA见BUILD.json，均复用R8。核心修改/重编译0。
只读trace路径`/home/jian/livox_ws/p63_r9_real_imu_prediction_data/libp63_r9_trace.so`，SHA `e87b8e26cc2d8eb1390291b603a81a7e015c47fc784a523cc8b1b0ee7f7a4fe6`；源码SHA `6e1a24a62e04a5ecd9ddb3d7a367036206672beda5bdba2b3eaafb1731d6b1cb`，完整O3/DNDEBUG编译与链接argv见BUILD.json。

本轮源码：tools/imu_geometry.py仅刚体输入转换；preflight.py校验真实CDR/预检原时钟；read_only_trace.cpp原R8测量+原生IMU入队和predict只读观测；run_imu_once.py独占目录、升序IMU缓存回执后发扫描、原生结果身份检查与封存；evaluate.py仅封存后参考评价。后续给evaluate.py增加局部恶化统计的可复现代码，未修改任何主要指标/阈值或重跑定位；该附加统计由封存CSV生成。plot_comparison.py只画实际连续输出。

新增独立ROS包research/p63_hdl_real_imu，复用R8的显示Path/RViz，不改旧包。正式launch的autoplay=false，扫描和IMU由专用run_imu_once.py一起监督；**不得用继承的单点云autoplay播放器冒充本IMU实验**。

## 实际命令（事实记录，不授权再次运行）

```bash
cd /home/jian/livox_ws/p63_r9_real_imu_prediction_ws
/usr/bin/python3 docs/p63_r9_hdl_real_imu_prediction/tools/test_imu_geometry.py
/usr/bin/python3 docs/p63_r9_hdl_real_imu_prediction/tools/preflight.py
/usr/bin/python3 docs/p63_r9_hdl_real_imu_prediction/tools/build_trace.py
# 提交冻结为上述CODE之后，仅执行一次：
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}
/usr/bin/python3 docs/p63_r9_hdl_real_imu_prediction/tools/run_imu_once.py > docs/p63_r9_hdl_real_imu_prediction/logs/imu_on_01.log 2>&1
# 原输出SHA封存之后：
/usr/bin/python3 docs/p63_r9_hdl_real_imu_prediction/tools/evaluate.py > docs/p63_r9_hdl_real_imu_prediction/logs/evaluation.log 2>&1
/usr/bin/python3 docs/p63_r9_hdl_real_imu_prediction/tools/archive_run.py
/usr/bin/python3 docs/p63_r9_hdl_real_imu_prediction/tools/plot_comparison.py
```

实际ROS argv在raw/imu_on_01/COMMAND.json，private master11649。无需rosbag或数据下载。原独占运行目录`/home/jian/livox_ws/p63_r9_real_imu_prediction_data/imu_on_01`保持不动；15个原始工件按RAW_MANIFEST逐字节gzip归档。ONLINE_SEAL内包含全部候选（只有原生）、IMU调用、缓存、输入转换、UKF输出和内存原始记录；先封存再读取测试GT。主要CSV和图片可直接离线审阅。

LIVE_raw164.png为本次真实RViz截图，显示地图、扫描和原UKF轨迹；不是离线候选轨迹。截图不用于认证精度。没有第二次手动回放。

## 必要测试/审阅

旋转方向、无符号翻转、恒角速度向心项/恒角加速度切向项、无状态修改和非法dt拒绝的ROS-free测试通过。Release只读trace编译、动态加载（零ROS/零NDT/零UKF）、Python语法、launch文件解析通过。首次git sparse-checkout时因旧Git缺少info目录/add子命令，显式创建元数据目录并使用set；没有覆盖主工作区。预检临时读取anchor字段名曾报KeyError，改用真实T_map_lidar_anchor字段后正式preflight通过；没有因此运行NDT。

最终核验另见logs/final_checks.log：真实原生调用而非模拟消费、Source/Target、时序、封存、旧代码/ELF和历史保护。时间模型预检只是预检；正式结果全部从原生trace/回执得出。预算160/160结束、无候选/重跑/自动下一轮。
