# 实际运行与复核命令

所有计算以系统内盘输入进行；本轮已完成外盘复制与独立SHA，不自动再读外盘。
不要重复以下20/100次align或删除旧输出目录。

## 已执行的转存

```text
rsync --partial --append-verify --info=progress2 --bwlimit=16384 '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/Shield_tunnel1_gamma.bag' /home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag.part
```

通过storage_transfer.py包装监测USB4-2/sda/host0等实际关联错误，每阶段新证据目录，最长20分钟，
无进度90秒停止，不按单次D-state误判。源／目标哈希相同后排他发布，不覆盖现有正式bag。
若未来明确批准恢复，可使用新尝试标签保留旧证据，例如copy resume1；原.part由rsync append-verify验证。
该接口不是自动重试授权。当前正式bag已经存在，工具会拒绝覆盖或重复制。

## 构建

GCC9.4.0、CMake3.16.3、PCL1.10.0、C++14 Release。
独立docs/CMake，不修改生产catkin配置。

```bash
cd /home/jian/livox_ws/p61_r2_geode_real_data_ws
cmake -S docs/p61_r2_geode_real_data_run/tools -B /home/jian/livox_ws/p61_r2_geode_runs/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p61_r2_geode_runs/build -j2
```

实际运行二进制SHA256：
`2b56a6a231443b7b5318d955ba1d5be1ed0ca7b0ecb118b10ee73f594f47ec02`。
沿用本机原有系统libusb preload解决MVS共享库冲突，不改NDT数学。

## 已执行的真实匹配

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o /home/jian/livox_ws/p61_r2_geode_runs/smoke_resource.txt /home/jian/livox_ws/p61_r2_geode_runs/build/native_geometry /home/jian/livox_ws/p61_r2_geode_runs/smoke_inputs/sample_manifest.csv 20 /home/jian/livox_ws/p61_r2_geode_runs/smoke
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o /home/jian/livox_ws/p61_r2_geode_runs/screen_resource.txt /home/jian/livox_ws/p61_r2_geode_runs/build/native_geometry /home/jian/livox_ws/p61_r2_geode_runs/screen_inputs/sample_manifest.csv 100 /home/jian/livox_ws/p61_r2_geode_runs/screen
```

原始stdout及冻结输入/输出都在p61_r2_geode_runs。每个输出目录必须不存在，防止无意重跑覆盖。
sample_manifest冻结真实索引和时间。首组Source2..21，后组由全5427个原始header时间均匀选100对。
所有完整点字段NPY和PCD只留内盘，Git只保存清单和结果。

## 可安全重复的无align测试

```bash
python3 -m unittest discover -s docs/p61_r2_geode_real_data_run -p 'test_*.py' -v
cd /home/jian/livox_ws/p61_r2_geode_runs/build
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ctest --output-on-failure
```

无融合准入，因此不提供冒充正式融合定位的roslaunch入口或虚假轨迹。
