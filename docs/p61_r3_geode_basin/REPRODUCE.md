# 实际命令与禁止重复执行的范围

输入来自/home/jian/livox_ws/p61_r2_geode_runs/screen_inputs/，原bag不读取。
全部大数据、原生二进制及完整日志位于系统内盘p61_r3_geode_runs。

已执行构建：

```bash
cd /home/jian/livox_ws/p61_r3_geode_basin_ws
cmake -S docs/p61_r3_geode_basin/tools -B /home/jian/livox_ws/p61_r3_geode_runs/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p61_r3_geode_runs/build -j2
```

已执行的唯一只读预检和正式实验（不要重复）：

```bash
python3 docs/p61_r3_geode_basin/run_frozen.py verify /home/jian/livox_ws/p61_r3_geode_runs/preregistered_v2 /home/jian/livox_ws/p61_r3_geode_runs
python3 docs/p61_r3_geode_basin/run_frozen.py align /home/jian/livox_ws/p61_r3_geode_runs/preregistered_v2 /home/jian/livox_ws/p61_r3_geode_runs
```

runner使用系统libusb preload（沿用已有环境库冲突处理），/usr/bin/time -v，
新建排他回执后才执行；已有回执拒绝重试。每次运行前核对SHA和本体源码不变。
预检20jet/0align，正式238jet/238align，另2个OOB只有记录。
唯一正式回执align_receipt.json将命令、二进制及输出SHA绑定实际CODE_SHA。

已执行冻结输出分析（无align/GT）：

```bash
python3 docs/p61_r3_geode_basin/analyze_basin.py /home/jian/livox_ws/p61_r3_geode_runs /home/jian/livox_ws/p61_r3_geode_runs/preregistered_v2 /home/jian/livox_ws/p61_r3_geode_runs/analysis
```

可复跑的单元测试（不调用align或读取原始传感器）：

```bash
python3 -m unittest discover -s docs/p61_r3_geode_basin -p 'test_*.py' -v
```

不启动ROS、EKF、Coupled、Recovery、在线回放或更多扰动搜索。
