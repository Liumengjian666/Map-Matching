# 真实复现命令

现有输出目录是单次保护，不覆盖、不删除后重跑，不绕过预算。

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
cmake -S src/dog_prior_map_localization/scripts/p32_world_frame -B .p32_r4_corridor02_world_frame/build -DCMAKE_BUILD_TYPE=Release
cmake --build .p32_r4_corridor02_world_frame/build -j2
/usr/bin/python3 src/dog_prior_map_localization/scripts/p32_world_frame/run_once.py --asset '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02' --binary .p32_r4_corridor02_world_frame/build/world_test --output .p32_r4_corridor02_world_frame/world_once
```

实际内部命令/环境/退出码见runs/world_once/execution.json，数学日志、
GNU time与四候选CSV均保存。进程绑定系统libusb，不修改系统或全局环境。
没有调用完整地图ICP、holdout、normalizer或F/G，因此不编造相应运行命令。
经本轮精确确认的外部CLI仅执行一次，完整命令与输入SHA见
external_review/execution.json；不将该命令当作重复调用授权。

只重算归档（无NN/ICP/ROS/GT）：

```bash
/usr/bin/python3 docs/p32_r4_corridor02_world_frame/finalize_evidence.py
```
