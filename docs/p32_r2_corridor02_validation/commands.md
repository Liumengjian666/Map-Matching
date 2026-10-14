# 实际命令与复现边界

以下属于数据/初始化检查，不是F/G定位回放。禁止据此追加逆矩阵或
其他种子。缓存有exist_ok=False与O_EXCL保护，不得清理原缓存来重试。
独立复现应采用新的、明确授权的独立目录，保持原证据。

```bash
git -C /home/jian/livox_ws/p10_corridor01_oosm_r1_ws status --short --branch
git -C /home/jian/livox_ws/p10_corridor01_oosm_r1_ws rev-parse HEAD
git -C /home/jian/livox_ws/p10_corridor01_oosm_r1_ws ls-remote origin refs/heads/research/p9-r4-heldout-visual-evidence
```

以下实际从P10工作区执行：

```bash
source /opt/ros/noetic/setup.bash
python3 src/dog_prior_map_localization/scripts/p32_corridor02/prepare_probe.py --bag '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02.bag' --asset '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02' --output .p32_r2_corridor02_validation/probe
cmake -S src/dog_prior_map_localization/scripts/p32_corridor02 -B .p32_r2_corridor02_validation/build -DCMAKE_BUILD_TYPE=Release
cmake --build .p32_r2_corridor02_validation/build -j2
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/dog_prior_map_localization/scripts/p32_corridor02 -p 'test_adapter.py' -v
PYTHONDONTWRITEBYTECODE=1 python3 src/dog_prior_map_localization/scripts/p32_corridor02/run_geometry.py --probe .p32_r2_corridor02_validation/probe --binary .p32_r2_corridor02_validation/build/first_geometry --map '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/map/corridor02.pcd' --protocol docs/p32_r2_corridor02_validation/PROTOCOL.md
```

最初prepare_probe版本在加入遍历硬上限前已经完成有限50/968/15条探针，
其实际源码快照保留；没有重读或多次运行首扫描配准。
编译有一次libusb链接修复，以及align前输入守卫修改后的正常增量重编译。
实际完整align只有1次。未执行rosbag play/roslaunch/GT评价/视觉提取命令。

官方页面本轮有限查阅：https://superodometry.com/superloc.html。
它的资产链接与TUM格式不构成具体GT参考点或初始矩阵方向证明。

用户单独确认后执行一次外部CLI只读复核，退出码0。
精确命令与原始输入/输出身份见external_review/execution.json。
该归档记录不构成再次调用授权。
