# 一次固定探针的真实命令

环境：Ubuntu GCC9.4.0，CMake3.16.3，PCL1.10.0，C++14 Release（-O3 -DNDEBUG）。
没有修改生产构建或启动链。实际运行二进制SHA256：
`de9d4f79676db5c43a66e3e0b74a27c861b3f7d7afd101c9d97588fb78aea318`。
所有继承代码／输入和二进制身份见run_code_freeze.json、input_freeze.json。

```bash
cd /home/jian/livox_ws/p60_r5_ndt_resolution_ws
cmake -S docs/p60_r5_ndt_resolution_correction/tools -B /home/jian/livox_ws/p60_r5_ndt_resolution_runs/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p60_r5_ndt_resolution_runs/build -j2
```

网格测试（无align）：

```bash
cd /home/jian/livox_ws/p60_r5_ndt_resolution_runs/build
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ctest --output-on-failure
```

本机已有MVS共享库的libusb冲突，沿用R4已记录的系统libusb preload，不改优化器。
preload文件SHA256：`451c0a2b8806b71ddab9046470badc7665a467d511e87c1d5e1da7a343456586`。

正式命令已经执行一次，退出0；不是后续自动重跑授权。
输出目录必须不存在；不要删除已有formal目录来重跑。

```bash
cd /home/jian/livox_ws/p60_r5_ndt_resolution_ws
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o /home/jian/livox_ws/p60_r5_ndt_resolution_runs/resource.txt /home/jian/livox_ws/p60_r5_ndt_resolution_runs/build/corrected_screen docs/p60_r4_ground_challenge_ndt_degeneracy/sample_manifest.csv docs/p60_r4_ground_challenge_ndt_degeneracy/alignments.csv /home/jian/livox_ws/p60_r5_ndt_resolution_runs/formal > /home/jian/livox_ws/p60_r5_ndt_resolution_runs/formal.log 2>&1
```

冻结输出后才执行的纯离线分析（不运行align、无GT）：

```bash
python3 docs/p60_r5_ndt_resolution_correction/analyze_results.py /home/jian/livox_ws/p60_r5_ndt_resolution_runs /home/jian/livox_ws/p60_r5_ndt_resolution_runs/analysis
```

analysis目录及归档同样拒绝覆盖。冻结核验测试可以重复执行，不会运行NDT：

```bash
python3 -m unittest discover -s docs/p60_r5_ndt_resolution_correction -p 'test_*.py' -v
python3 -m unittest discover -s docs/p60_r4_ground_challenge_ndt_degeneracy -p 'test_*.py' -v
```

提交后自行上传（本轮没有执行）：

```bash
git -C /home/jian/livox_ws/p60_r5_ndt_resolution_ws push -u origin research/p60-r5-ndt-target-resolution-correction
```

分支只存在于隔离worktree的Git仓库，不能照搬受保护dog_loc_paper_ws路径，否则可能源引用不存在。
