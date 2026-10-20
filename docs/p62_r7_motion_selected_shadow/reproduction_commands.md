# 复现与只读核查

工作区`/home/jian/livox_ws/p62_r7_motion_shadow_ws`。以下实际执行已结束；不能删除token或原始输出以重新启动本轮实验。

实际纯数据选择（已执行，一次；输出exclusive，不能覆盖）：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r7_motion_selected_shadow/select_inputs.py
```

实际构建及零align单元测试：

```bash
cmake -S docs/p62_r7_motion_selected_shadow/tools -B /home/jian/livox_ws/p62_r7_motion_shadow_runs/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p62_r7_motion_shadow_runs/build -j2
/home/jian/livox_ws/p62_r7_motion_shadow_runs/build/test_recording
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s docs/p62_r7_motion_selected_shadow -p 'test_motion_selection.py' -v
```

实际受控运行命令（已失败并保留，一次；真实align=0）：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r7_motion_selected_shadow/run_shadow.py
```

只运行输入预检即因验证块点数停止。没有Smoke、NDT、诊断、EKF或轨迹。该命令不会被本轮再次执行；RUN_STARTED/原formal目录存在会阻止复用。

日志：`/home/jian/livox_ws/p62_r7_motion_shadow_runs/formal.log`；实际输入预检：同目录formal/input_preflight.csv；完整原始运动/时间清单：all_imu_motion.csv、all_scan_times.csv。编译日志为`/home/jian/livox_ws/p62_r7_build.log`及p62_r7_build_review.log，configure为p62_r7_configure.log。large bag/map保持原路径，仅只读。

归档代码archive_blocked.py只读失败结果，独立核验ECDF和排名/Source/块资格后写新的派生报告；采用exclusive输出，不覆盖原始资产。它不执行选择器、G、NDT或EKF。合成unit测试不代表实际优化器运行。

完成后安全只读检查（不运行算法）：

```bash
cd /home/jian/livox_ws/p62_r7_motion_shadow_ws && sha256sum -c docs/p62_r7_motion_selected_shadow/checksums.sha256
cd /home/jian/livox_ws/p62_r7_motion_shadow_ws && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/p62_r7_motion_selected_shadow/verify_results.py
```

环境沿用R6现有GCC9.4、PCL1.10、Eigen/CMake3.16、Python3.8/Numpy，不安装依赖。仅子进程LD_PRELOAD已核验系统libusb，避免MVS旧库；loader无参检查0align。来源与二进制以run_freeze为准。
