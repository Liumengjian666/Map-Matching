# 实际构建与最小测试回执

Ubuntu20.04.6、GCC9.4.0、CMake3.16.3、PCL1.10，原ROS环境不变。
只构建独立离线程序。实际执行的核心命令：

```bash
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu cmake -S /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/src/dog_prior_map_localization/scripts/p14_inlier_baseline -B /home/jian/rosbag/p14_inlier_r1_20261011/build -DINLIER_SOURCE=/home/jian/rosbag/p14_inlier_r1_20261011/upstream_inlier -DCMAKE_BUILD_TYPE=Release
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu cmake --build /home/jian/rosbag/p14_inlier_r1_20261011/build -j2
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu OMP_NUM_THREADS=1 OMP_DYNAMIC=FALSE /home/jian/rosbag/p14_inlier_r1_20261011/build/inlier_baseline --smoke /home/jian/rosbag/p14_inlier_r1_20261011/experiment/author_synthetic_f64.bin /home/jian/rosbag/p14_inlier_r1_20261011/experiment/smoke_release.csv
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu python3 src/dog_prior_map_localization/scripts/p14_inlier_baseline/test_input.py --author /home/jian/rosbag/p14_inlier_r1_20261011/upstream_inlier --binary /home/jian/rosbag/p14_inlier_r1_20261011/build/inlier_baseline --out /home/jian/rosbag/p14_inlier_r1_20261011/experiment
```

以上测试已执行且通过，不要求用户重复运行。真实单次实验命令、执行环境、
源文件及binary SHA见execution_receipt.json。源码工作树先提交，才运行真实数据。
seed0重复Verify精确一致。边界预处理9→4点，真实reference0的17699→559点
与作者Python预处理逐点/顺序完全一致；Python仅用于输入测试与统计，算法
实际执行是直接C++，未尝试fallback。作者合成fixture函数通过AST直接调用，
仅移除pytest装饰器，不重新编写合成数据公式。

兼容依赖nanoflann v1.6.2 commit26071402db0e690f6e009e64080fda653fac2ae6，
由作者CMake获取。inlier_core源文件未修改，LICENSE为MIT。
原libusb链接失败和最初不合理的烟测断言也保留日志。无真实数据选择性重跑。

CSV/JSON/SE3审计、Python语法检查及git diff --check通过。只核验本轮新离线
接口，未重跑P10/P11/P12、ROS一致性、滤波器或完整作者pytest套件。
独立只读审查修正了输入零点保护和分阶段计时快照，不修改科学参数。
未启动外部跨模型CLI。

Git日志副本机械删除行尾空格并使用LF；外部原始日志未修改，原字节SHA及
路径见raw_log_provenance.json，归档字节SHA见artifact_hashes.json。
