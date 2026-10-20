# 复现命令（本轮结束后不自动运行）

所有命令在独立R6工作区，禁止在dog_visual_loc_ws构建。原始大bag、PCD、二进制留内盘。

实际构建：

```bash
cmake -S /home/jian/livox_ws/p62_r6_dog_shadow_ws/docs/p62_r6_dog_shadow_prototype/tools -B /home/jian/livox_ws/p62_r6_dog_shadow_runs/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p62_r6_dog_shadow_runs/build -j2
```

GCC9.4.0、CMake3.16.3、C++14 -O3 -DNDEBUG、本机PCL1.10、Python3.8.10。测试用require不受NDEBUG取消。没有安装新软件。完整编译日志与CMakeCache在runs内；头文件与binary身份在run_freeze。

实际测试（已运行；包含2次合成align，**不是只读日志检查，不要在本轮再次运行**）：

```bash
/home/jian/livox_ws/p62_r6_dog_shadow_runs/build/test_contracts
```

实际成功执行的一条受控命令（已执行完毕；重用同目录会拒绝，不删除token/旧输出）：

```bash
cd /home/jian/livox_ws/p62_r6_dog_shadow_ws && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/p62_r6_dog_shadow_prototype/run_shadow.py --attempt-dir /home/jian/livox_ws/p62_r6_dog_shadow_runs/attempt02
```

该脚本只启动离线C++原型，不是rosbag play/ROS节点/EKF。以冻结的100份XYZ+原始offset字段和记录共同初值为输入。严格独占新输出、前后完整输入SHA与生产源码/工作树守卫、真实PCL叶尺寸检查、最多20额外align。仅子进程LD_PRELOAD已核验系统libusb，环境不影响生产。第一次旧loader失败保留在runs根目录，成功运行在attempt02。

若未来总控批准复现实验，必须创建**新的**输出目录并记录新执行身份，不能删除旧token使原目录重新通过。当前合同不授权再次运行。

只读核查已冻结结果（不运行任何align）：

```bash
cd /home/jian/livox_ws/p62_r6_dog_shadow_ws && sha256sum -c docs/p62_r6_dog_shadow_prototype/checksums.sha256
cd /home/jian/livox_ws/p62_r6_dog_shadow_ws && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/p62_r6_dog_shadow_prototype/analyze_results.py --verify-only
```

离线分析脚本`analyze_results.py`提供独立70顶点LP oracle及矩阵检查，读取冻结CSV、不启动优化器。本次单候选窗口实际oracle调用0次，真实成对比较0；只复核单候选选择、输入和预算。`--verify-only`安全重算、对比原始输出与归档，不写文件。非verify模式的已有归档输出采用exclusive写入，重用同目录会拒绝，不能覆盖原归档。分位数为numpy线性type7；单候选优势列的原始1为未定义哨兵，解释为NA。

`prepare_inputs.py`为实际一次性提取代码，依赖已有ROS Python读取API，仅提取首100帧；不播放bag。其输出已冻结，不能再次运行到原路径。字段raw_serialized_sha256的实际方法明确为XYZ float32+offset uint32，不是完整ROS序列化摘要；整个原始bag另有完整SHA。
