# 复现命令与实际运行身份

以下用于另一次明确授权的独立复现，不代表本轮追加执行。
`mktemp`生成新缓存避免覆盖原结果，运行工具本身也拒绝已存在的输出目录。
GT本轮仅用于离线诊断；不得将生成的矩阵导入正式定位。

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
source /opt/ros/noetic/setup.bash
P32_R5_REPRO_DIR=$(mktemp -d /tmp/p32-r5-reproduce.XXXXXX)
env PYTHONPYCACHEPREFIX="$P32_R5_REPRO_DIR/pycache" /usr/bin/python3 src/dog_prior_map_localization/scripts/p32_gt_frame/prepare.py --asset '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02' --output "$P32_R5_REPRO_DIR/input"
cmake -S src/dog_prior_map_localization/scripts/p32_gt_frame -B "$P32_R5_REPRO_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$P32_R5_REPRO_DIR/build" -j2
/usr/bin/python3 src/dog_prior_map_localization/scripts/p32_gt_frame/run_once.py --inputs "$P32_R5_REPRO_DIR/input" --binary "$P32_R5_REPRO_DIR/build/gt_geometry" --output "$P32_R5_REPRO_DIR/diagnostic_once" --protocol docs/p32_r5_corridor02_gt_frame_test/PROTOCOL.md
```

本轮实际缓存为`.p32_r5_corridor02_gt_frame_test/`，其中原始XYZ/poses不提交。
实际NN可执行文件、参数、退出码：`runs/diagnostic_once/execution.json`；
完整源与输入绑定：`runs/diagnostic_once/input_freeze.json`。
该程序为独立编译，原定位二进制未重编译。
MVS环境下只在本NN子进程使用`LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0`，
没有修改全局环境或系统共享库。

实际源码CODE SHA、NN二进制SHA、运行环境版本见REPORT和freeze。
二进制SHA不是全体系统共享库一致性的证明。

外部CLI不是科研实验：单独确认的精确命令、有限输入SHA与实际输出
仅保存在`external_review/`。CLI不获准重复运行或触发额外实验。

Git上传由用户手动执行；本轮不自动push，不向受保护工作区复制成果。
