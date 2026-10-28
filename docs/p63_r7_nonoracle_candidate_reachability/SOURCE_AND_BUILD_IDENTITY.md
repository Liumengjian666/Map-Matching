# P63-R7源码与运行身份

START `79657c8c4cba6985af7a51ee759fe38746c24acc`；匹配/排序CODE `88daa7203b30b081d1ef49eee2284272649d4869`；封存后评价CODE `99973ff0d9a38947ca806e0770cc0f93d9f82298`。END为最终归档commit，完整值见交接。分支`research/p63-r7-nonoracle-candidate-reachability`，独立工作区`/home/jian/livox_ws/p63_r7_nonoracle_reachability_ws`。本地提交，不push。

## 复用和修改范围

R6 `tools/oracle_ndt.cpp`中的原PCD读取、PointCloud2序列化、同帧复制、PCL VoxelGrid、XYZ SHA及原生NDT/NN/inlier计算直接复用到新`tools/reachability_ndt.cpp`。R5/R6旧文件不改。

必要改变仅为：Oracle字段替换成R5 prediction，增加R6 Source SHA核对，读取48个预注册地图XY平移矩阵，6×8固定循环、有限性停止、事件计时。NDT工厂、分辨率、Gaussian实现、梯度/Hessian、停止参数均未修改；不调用UKF或ROS发布。

- `tools/prepare_inputs.py`：只从R5原始ONLINE_RESULTS取guess/Candidate 0，按字段允许表生成新INPUT，绝不读R6 INPUT/POSE/Oracle输出或参考文件。
- `tools/reachability_ndt.cpp:52`：复用R6准备路径；`:102`：完整六帧/48种子预检；`:143`：目标网格只初始化一次；`:158`：唯一align调用语句，固定48次。
- `tools/select_candidates.py`：收敛/有限条件下只按`(-score, id)`排序，包括0，无NN/GT决策。
- `tools/run_once.py`：独占formal_01，源/库/资产SHA检查、180s超时；先调用C++，再原生排序，随后封存，不能覆盖重试。
- `tools/evaluate.py`：只在候选和选择封存通过后，读取官方参考作误差/标签、描述性去重与成本统计，不调用NDT或重新选择。
- `tools/test_selection.py`：零align验证精确同分0优先、分数排序和收敛/有限条件，不作大量合成实验。

## 上游与实际二进制

HDL `de2dc7769aa2412876d05ae6161408d3295b6a3c`；NDT_OMP `5495fd9214945afcb4b35d5a1da385e405c52bf9`。原R5/R6共享库不重编译，Release仅构建新离线驱动。

实际程序：`/home/jian/livox_ws/p63_r7_nonoracle_reachability_data/reachability_ndt`。

SHA256：`fc28ef297d6c030c3a52bcfeedbf0bac0516ace37888a86f5a60f5ea727231ff`。

原`libndt_omp.so` SHA：`e274416abe31236910b5aa588e095cd3100dd77f348163ffddd29618bb8b82ff`。原参数resolution1m/DIRECT7/.01/max35/.1/.55、Source/Map Voxel .1m。源代码/编译/链接argv、各旧库SHA完整记录在`BUILD.json`。Release O3/DNDEBUG/SSE4.2/OpenMP，原默认16线程，不调线程或系统依赖。

沿用R6已验证的系统库优先路径以避免相机SDK libusb遮蔽。构建一次通过，没有原生参数或核心源码补丁。新二进制、object和原始点云/地图仅保存在独立外部数据目录，不提交Git。

## 实际执行记录

```bash
cd /home/jian/livox_ws/p63_r7_nonoracle_reachability_ws
/usr/bin/python3 docs/p63_r7_nonoracle_candidate_reachability/tools/prepare_inputs.py
/usr/bin/python3 docs/p63_r7_nonoracle_candidate_reachability/tools/build.py
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} /home/jian/livox_ws/p63_r7_nonoracle_reachability_data/reachability_ndt --prepare-only docs/p63_r7_nonoracle_candidate_reachability/INPUT.json /home/jian/livox_ws/p63_r7_nonoracle_reachability_data/preflight
/usr/bin/python3 docs/p63_r7_nonoracle_candidate_reachability/tools/test_selection.py
# 上述align=0；随后提交48种子/排序/源码/合同，才执行以下唯一正式试验
/usr/bin/python3 docs/p63_r7_nonoracle_candidate_reachability/tools/run_once.py > docs/p63_r7_nonoracle_candidate_reachability/logs/formal_01.log 2>&1
# OUTPUT_SEAL含SELECTION_NATIVE.json之后才执行
/usr/bin/python3 docs/p63_r7_nonoracle_candidate_reachability/tools/evaluate.py > docs/p63_r7_nonoracle_candidate_reachability/logs/evaluation.log 2>&1
```

原生命令见`raw/COMMAND.json`。48次BEFORE/AFTER见`raw/RUN.log`，全部原生候选/事件时间/不含GT的选择/封存在`raw/`；`RAW_MANIFEST.json`核对原运行副本。所有54个候选（0为历史）和六个实际选择的评价分别在CANDIDATES.csv/SELECTION_RESULTS.csv，GT最佳列不属于实际输出。

**48次预算已用完，命令为事实记录，不授权再跑或追加。** formal_01独占且C++有ALIGN_STARTED保护。不下载、不删除、不改写历史、不写稳定机器狗工程，不启动在线定位/UKF。
