# R5运行、显示和结果复现

原版算法固定为HDL `de2dc7769aa2412876d05ae6161408d3295b6a3c`。只编译研究只读trace ELF，原算法库复用`/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/lib/`。编译命令/二进制SHA在BUILD.json。不要在稳定机器狗工作区执行构建或启动这些研究配置。

## 本轮实际执行

已完成的唯一正式运行，以下是命令记录，**formal_01已存在，不能再次执行覆盖或当作新的授权**：

```bash
cd /home/jian/livox_ws/p63_r5_hdl_public_baseline_ws
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}
/usr/bin/python3 docs/p63_r5_hdl_public_baseline/tools/run_once.py > docs/p63_r5_hdl_public_baseline/logs/formal_01.log 2>&1
```

该监督器自动启动私有master11645、真实HDL nodelet/map server、原生TF、只读Path和RViz，随后发送160份PCD转换的真实PointCloud2；逐帧握手。没有rosbag、IMU输入或旧历史Predictor。所有输出关闭并写ONLINE_SEAL后才执行`tools/evaluate.py`，其复用Git中原R29 Reference实现。输出先封存，评价过程再次核对封存SHA。

`raw/LIVE_raw131.png`及`raw/LIVE_raw164.png`均为**实际在线定位截图**，不是封存轨迹播放器。显示固定map、当前aligned scan、原UKF轨迹/odom与TF。截图本身不证明精度，正式结果为BASELINE_LOCALIZATION_FAIL。

## 一键roslaunch显示入口

以下配置已在正式运行中实际加载；`autoplay=true`的手动组合仅做XML和脚本检查，**没有第二次执行**。只有用户/总控批准额外回放后才手动使用；它不等于本轮正式监督评价。无需另开rosbag或手动添加RViz显示项：

```bash
source /opt/ros/noetic/setup.bash && source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && export ROS_PACKAGE_PATH=/home/jian/livox_ws/p63_r5_hdl_public_baseline_ws/research:${ROS_PACKAGE_PATH:-} && export ROS_MASTER_URI=http://127.0.0.1:11645 ROS_IP=127.0.0.1 && r5_demo_dir=$(mktemp -d /home/jian/livox_ws/p63_r5_hdl_public_baseline_data/manual_view_XXXXXX) && roslaunch p63_hdl_public_baseline p63_r5_indoor_easy.launch output_dir:="$r5_demo_dir" autoplay:=true rviz:=true
```

手动播放器固定raw5–164，等待每帧odom后再发下一帧，不读参考、不重定位，完毕后定位/GUI保留供查看。退出使用Ctrl-C。不要往已有formal_01或旧试验目录输出；不要开启全局定位、IMU、修改initialpose或参数后把它称为R5原版结果。

## 检查范围和交付索引

- `FRAME_AND_INPUT_CONTRACT.md`：预注册、真实坐标/时间及成功/失败界限。
- `INPUT_FREEZE.json`：160扫描身份、raw4初始化、8 Tile及合并图身份。
- `RESULTS.csv`：160行实际初值、原生终值、UKF、原生质量、耗时、累计RSS高水位及封存后误差。
- `RESULTS.json`：正式结论、全部统计、P62同窗比较及原始来源身份。
- `P62_MATCHED_WINDOW.csv`、`CRITICAL_FRAMES.csv`：旧输出同窗重算及三个关键帧。
- `raw/`、`RAW_MANIFEST.json`：完整NDT/callback/status/odom/RSS/启动日志，封存SHA及实际截图。
- `P62_VS_HDL_COMPARISON.md`：系统差异与失败链，禁止一因素或独立动捕精度表述。

160输入SHA/转换XYZ核对、3个真实样本序列化检查、只读wrapper一次转发、Release编译与launch解析通过。没有为测试调用额外NDT。四上游源码pristine，六份旧R3库保持原SHA；SOURCE_AFTER.json仅记录这些必要身份，不扩展审计旧P62。

两套系统的误差均用相同Reference实现，在各自**真实输出时间**评价；P62重算与原封存对应样本误差最大差0。没有事后对齐、删高误差帧、GT在线预测或失锁后重置。参考是地图锚定GICP+IMU，不是独立动捕。

本轮在线CODE为661cc97784edb3e387afe75f6e4f6c3e6e8179fc。后置评价/文档改动不冒充参与过原生运行。
