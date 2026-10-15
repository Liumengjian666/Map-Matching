# P32-R7源码修改范围

CODE：e10e133c12c73d75655b2a5c0e360adf7eac12c0。

新增`src/dog_prior_map_localization/scripts/p32_annotated_pose/`：

- `run_once.py`：只提取第一组完整注释4×4，矩阵审查、输入/源码/二进制冻结、一次性输出目录及条件准入。
- `annotated_test.cpp`：R4同约定的唯一初始NN，增加同一次点查询中的2m覆盖统计；通过预筛才调用一次原地图ICP，并记录最终矩阵和准入。
- `CMakeLists.txt`：独立构建离线程序，不重新编译NDT/EKF等运行节点。

直接包含R3保留的原`superloc_first_segment_initializer.cpp`，main重命名且不调用；
只复用类型、`voxel()`、`registerCandidate()`和旋转角度统计。
因此没有调用其局部子图构建、properAxes或24候选main，也没有改原文件。

本归档的`finalize_evidence.py`仅检验实际冻结文件及汇总已存在CSV，
`prepare_external_review.py`仅组装有限复核输入，
`archive_external_review.py`仅归档已执行的一次授权复核。
这些归档工具不调用NN/ICP/NDT、ROS或GT。

未修改P31成熟算法、NDT参数、DCReg、多候选、MSCKF、SKF、平方根EKF、
OOSM、Deskew、velocity_blend、Step Limit、噪声或标定。
未写受保护工作区，未删历史缓存，未修改/提交地图、局部PCD、Bag或编译文件。
原始第一组完整矩阵以小型JSON、原文与CSV记录；本地f64不提交。
