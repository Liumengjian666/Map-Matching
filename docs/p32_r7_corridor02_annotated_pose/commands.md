# 实际命令与复现边界

本轮只执行一次真实实验。下列命令记录用于经授权复现，不代表再执行一次。
原one-shot输出目录已存在，runner会拒绝覆盖，不要删除它以重新运行。

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
cmake -S src/dog_prior_map_localization/scripts/p32_annotated_pose -B .p32_r7_corridor02_annotated_pose/build -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build .p32_r7_corridor02_annotated_pose/build -j2
env PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 src/dog_prior_map_localization/scripts/p32_annotated_pose/run_once.py --self-test
env PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 src/dog_prior_map_localization/scripts/p32_annotated_pose/run_once.py --asset '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02' --binary .p32_r7_corridor02_annotated_pose/build/annotated_test --output .p32_r7_corridor02_annotated_pose/annotated_once
```

自测不调用NN/ICP。原实际自测在配置/编译前执行一次，以上命令作为复现步骤
排列不改变其含义，不声称两次自测或重新编译。
实际底层二进制命令及进程环境见`runs/annotated_once/execution.json`。
runner对子进程显式设置系统libusb的LD_PRELOAD，未修改全局环境或共享库。
map/local/YAML/原helper及执行源码SHA见`input_freeze.json`，运行二进制SHA见REPORT。

后处理仅运行`docs/p32_r7_corridor02_annotated_pose/finalize_evidence.py`，
读取已生成文件，核对SHA并生成轻量归档；不新增NN/ICP或独立验证。

用户确认的外部CLI精确命令、有限输入SHA和输出原文单独归档到
`external_review/`。该调用不是科研实验，不能触发另一次定位/配准。
按用户合同，未执行GT读取、其他候选、5–10秒验证或F/G。
