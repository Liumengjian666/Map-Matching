# R8实际源码、编译、执行及RViz入口

START `cf5a45e47819c3bbbc0834bcfc2ab132aeac6818`；基线CODE `4cdfa6bd52ad954a684e32fc13309b59040b96c9`；候选/排序CODE `e1712fea88ede397f7c9b628c0277dbc064329a1`；封存后评价CODE `03203e45b00873ea346f7247d988e0701b24a9bd`。END为最终归档commit，完整SHA见交接。分支`research/p63-r8-hdl-indoor-hard-holdout`，工作区`/home/jian/livox_ws/p63_r8_indoor_hard_holdout_ws`，仅本地提交、未push。

## 冻结上游与源码范围

原HDL `de2dc7769aa2412876d05ae6161408d3295b6a3c`；NDT_OMP `5495fd9214945afcb4b35d5a1da385e405c52bf9`；原`libndt_omp.so` SHA `e274416abe31236910b5aa588e095cd3100dd77f348163ffddd29618bb8b82ff`。复用R3/R5原算法Release库，不重编译或修改NDT/UKF核心。

本目录`tools/`：

- `prepare_data.py`：核对三官方载荷MD5及历史SHA，只重新解码raw4–164实际CDR核对XYZ/header；复用Hard原Tile，不拟合变换；仅解析raw4参考插值，输出冻结输入。
- `kernel_preflight.cpp`：复用R6/R7消息及VoxelGrid路径，零align实测Hard新目标与160个Source SHA。
- `read_only_trace.cpp`：R5只读wrapper只增加Source XYZ SHA；每次仍原样转发一次原生computeTransformation，UKF/优化参数不变。额外哈希记入diagnostic时间。
- `run_baseline_once.py`：R5逐帧握手监督，160帧，发布前新增严格Hard target身份核对；原生低质量不门控、不提前重定位；写完整回执和ONLINE_SEAL。
- `prepare_candidates.py`：必须验证基线已封存，固定六帧actual guess，输出48个非Oracle种子；不读测试参考。
- `candidate_ndt.cpp`：R7原路径，只将Hard地图点数/SHA、六个预定raw及Source身份改为实际Hard基线；八方向/.75m、原NDT与质量公式不改。
- `select_candidates.py`：逐字节复用R7原生Gaussian排序及同分规则；没有NN/GT选择。
- `run_candidates_once.py`：再次核对基线seal，最多48次，然后排序并封存，无UKF或ROS。
- `evaluate.py`：两阶段封存后才打开测试参考，完整160帧评价及六帧分组/安全/恢复判定，不回写选择或调用NDT。
- `plot_baseline.py`：只画原基线误差，未将候选连接为在线轨迹。

独立ROS配置在`research/p63_hdl_indoor_hard/`，原稳定工作区和R5–R7未修改。封存后发现未来手动autoplay脚本继承的工具导入路径需改为R8实际目录，已修正并静态检查；正式基线由监督器发送扫描、autoplay=false，该未使用路径修正不改变实际运行。没有因此重放。

## 实际构建身份

只构建独立预检/只读trace/离线驱动，原算法库保持SHA。

- trace：`/home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/libp63_r8_trace.so`，SHA `9154b601162c5cdaf3bb1c40f467feedcfc71d327bdd31aadeefdbe0eb5528c6`。
- 候选程序：同目录`candidate_ndt`，SHA `4a51212c87fa991e99e41549fd6f9d399a8ed1c60f1cc40434adedf132a465f8`。
- 完整编译/链接argv、源码SHA及原库SHA：BASELINE_BUILD.json、CANDIDATE_BUILD.json。

Release O3/DNDEBUG，沿用原默认16个OpenMP线程，不调线程/算法参数。系统库优先`/usr/lib/x86_64-linux-gnu`以继承R6已解决的SDK libusb环境约定，无依赖安装。预检、构建、launch解析、排序同分/有限性零align测试通过。

## 真实执行顺序

```bash
cd /home/jian/livox_ws/p63_r8_indoor_hard_holdout_ws
/usr/bin/python3 docs/p63_r8_hdl_indoor_hard_holdout/tools/prepare_data.py
/usr/bin/python3 docs/p63_r8_hdl_indoor_hard_holdout/tools/build_components.py
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} /home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/kernel_preflight docs/p63_r8_hdl_indoor_hard_holdout/INPUT_FREEZE.json /home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/preflight
# 160个Source与Hard目标身份写入冻结INPUT、提交后：
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}
/usr/bin/python3 docs/p63_r8_hdl_indoor_hard_holdout/tools/run_baseline_once.py > docs/p63_r8_hdl_indoor_hard_holdout/logs/baseline_01.log 2>&1
# 160帧完成并封存后：
/usr/bin/python3 docs/p63_r8_hdl_indoor_hard_holdout/tools/prepare_candidates.py
/usr/bin/python3 docs/p63_r8_hdl_indoor_hard_holdout/tools/build_candidate.py
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} /home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/candidate_ndt --prepare-only docs/p63_r8_hdl_indoor_hard_holdout/CANDIDATE_INPUT.json /home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/candidate_preflight
/usr/bin/python3 docs/p63_r8_hdl_indoor_hard_holdout/tools/test_selection.py
# 48实际种子和源代码另行提交冻结后：
/usr/bin/python3 docs/p63_r8_hdl_indoor_hard_holdout/tools/run_candidates_once.py > docs/p63_r8_hdl_indoor_hard_holdout/logs/candidate_01.log 2>&1
# 候选与选择均封存后：
/usr/bin/python3 docs/p63_r8_hdl_indoor_hard_holdout/tools/evaluate.py > docs/p63_r8_hdl_indoor_hard_holdout/logs/evaluation.log 2>&1
```

实际基线ROS argv和原生候选argv分别保存在`raw/baseline_01/COMMAND.json`、`raw/candidate_01/COMMAND.json`。原始外部目录分别为数据目录的baseline_01和candidate_01。两阶段独占，正式预算160+48已用完，以上命令只是事实记录，不授权重跑。

## 一键ROS launch及RViz

新launch已在实际基线中加载，自动启动真实HDL nodelet、固定地图、Path显示和RViz，原生发布当前扫描、位姿与TF。两张`raw/baseline_01/LIVE_raw155.png`、`LIVE_raw164.png`为真正在线基线现场图，非候选恢复或封存轨迹假播放；截图不认证定位精度。

以下手动组合仅做XML/工具路径检查，**没有作为第二次定位执行**。仅用于总控另行批准的复现，自动发布固定160帧PCD，无需rosbag、手动加载地图或添加显示项：

```bash
source /opt/ros/noetic/setup.bash && source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && export ROS_PACKAGE_PATH=/home/jian/livox_ws/p63_r8_indoor_hard_holdout_ws/research:${ROS_PACKAGE_PATH:-} && export ROS_MASTER_URI=http://127.0.0.1:11648 ROS_IP=127.0.0.1 && r8_view_dir=$(mktemp -d /home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/manual_view_XXXXXX) && roslaunch p63_hdl_indoor_hard p63_r8_indoor_hard.launch output_dir:="$r8_view_dir" autoplay:=true rviz:=true
```

手动player等待原生odom后发下一帧，不读GT，不反馈离线候选；未包含正式监督器全部验收回执，不冒充新正式结果。本轮不再执行此命令，完成后停止。
