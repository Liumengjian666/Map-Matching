# 完整复现命令与单次保护

以下来自本轮实际执行。现有缓存带一次性目录/marker；不要重复执行
实验、覆盖原输出，亦不要删除 marker 来绕过预算。

真实实验脚本版本为CODE aaef3d58ded3561710a8148807c700d0c24e2289，
见runs/initializer_once内的prepare_input_executed.py/run_once_executed.py。
外部复核后的源码补了更严格来源守卫，旧extraction没有预冻结decoder
字段，当前工具会合法拒绝旧输入receipt；不应为复现而覆写旧冻结记录。
上面所述版本和原回执定义真实实验；下列命令不是授权再次执行预算。

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
cmake -S src/dog_prior_map_localization/scripts/p32_init_transfer -B .p32_r3_corridor02_init_transfer/build -DCMAKE_BUILD_TYPE=Release
cmake --build .p32_r3_corridor02_init_transfer/build -j2
/usr/bin/python3 src/dog_prior_map_localization/scripts/p32_init_transfer/prepare_input.py --asset '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02' --output .p32_r3_corridor02_init_transfer/input
/usr/bin/python3 src/dog_prior_map_localization/scripts/p32_init_transfer/run_once.py --asset '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02' --inputs .p32_r3_corridor02_init_transfer/input --build .p32_r3_corridor02_init_transfer/build --output .p32_r3_corridor02_init_transfer/initializer_once --protocol docs/p32_r3_corridor02_init_transfer/PROTOCOL.md
```

原初始化调用、每进程环境、实际退出码与GNU time回执见
`runs/initializer_once/*_execution.json` 及资源文件。
工具对其进程使用系统 libusb 的 LD_PRELOAD，不更改系统或全局环境。

只重算归档（不执行ICP/ROS，不读取GT）：

```bash
/usr/bin/python3 docs/p32_r3_corridor02_init_transfer/finalize_evidence.py --code-sha aaef3d58ded3561710a8148807c700d0c24e2289 --guard-sha d3b00ca27787fe0c9c946ecef8f1770bd2cdee8a
```

没有 holdout、map normalization 或 F/G 命令回执，原因是初始化未准入；
不列出未执行命令作为已完成实验。外部CLI精确命令单独记录实际回执。
