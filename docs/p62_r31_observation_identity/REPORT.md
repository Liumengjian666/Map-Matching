# P62-R31-OBSERVATION-IDENTITY-CLOSURE

本轮完成单项观测身份修复及全部批准回放。工程语义PASS，完整Indoor定位性能FAIL。没有自动继续研究或Outdoor实验。

START：`edb4e913bdbaec2a493252cd97b7c842f9e41109`（远端计划分支已核实）。实际生产源码CODE：`bab26e991ef7ac871057ee4f9a94c92831619a62`。执行分支`research/p62-r31-observation-identity-closure`，工作区`/home/jian/livox_ws/p62_r31_observation_identity_ws`。B运行调度HEAD为`b8e0bc9a3105028732ab4d8eaaafd42032a931e9`；后两条调度HEAD为`739b4873dc3f90d924595b9b6dad3657fec89836`，生产二进制相同。END为包含本报告的最终归档commit，完整SHA在最终交接给出（避免将commit哈希写入自身循环）。只本地提交，未push。

## 完整结果

|数据/方法|身份策略/来源|平移RMSE m|旋转RMSE °|地图发布/EKF更新|额外NDT|最长无地图观测帧数|
|---|---|---:|---:|---:|---:|---:|
|EASY / G-only|STRICT_FAIL_CLOSED|5334.892892|26.538893|4/4|0|1978|
|EASY / C+G|STRICT_FAIL_CLOSED|5334.892892|26.538893|4/4|96|1978|
|NC_W1 / C+G|STRICT_FAIL_CLOSED|0.209943|2.984149|57/57|8|2|
|EASY / G-only|LEGACY_HISTORICAL_R29|630.506992|16.232952|139/137|0|1644|
|EASY / C+G|LEGACY_HISTORICAL_R29|630.506992|16.232952|139/137|132|1644|
|NC_W1 / C+G|LEGACY_HISTORICAL_R24_G_F_OFF|0.209943|2.984149|57/57|8|2|

完整P95/Max、首个持续失锁帧、调用次数见R31_CLOSED_LOOP_COMPARE.csv；完整逐帧轨迹见TRAJECTORIES/，未删失败区间。所有轨迹实际输入和输出覆盖100%，Indoor有效参考2022/2023（raw1336参考间隙不满足原合同）；NC60/60。误差直接在既定地图系计算，不做轨迹对齐。

## 工程证据与性能边界

实际处理4106帧，新增原生NDT932次=828名义+104额外；地图发布65次，实际EKF更新65次。两条2023帧Indoor各4次融合、2次身份拒绝；60帧NC57次融合、0次身份拒绝。8个单元案例及定向CTest通过。65份实际ROS NDT消息均匹配原认证终值，拒绝时导航历史/发布计数未变；所有4106次预测查询遵循已有因果时间条件，65次实际OOSM符合原回滚/重放时间约束。没有更改EKF/IMU/OOSM/Deskew数学或现有阈值。

raw62的原生终值、地图哈希和前置输入与R29一致。旧限步器将该合格终值移动0.446249m；新STRICT正确不发布。随后长期地图观测饥饿和IMU传播漂移仍然发生且明显加重。这提供了修复身份语义与提升定位性能分离的直接反例，不能声称R29失锁已修复。后续map/质量失效是实际因果轨迹的一部分，未靠阈值或重置掩盖。完整拒绝原因分布见results.json。

NC60帧回归保留原有C+G结果，但不能借此宣称通用安全性；该窗口是已知片段，参考为MAP_ANCHORED_BENCHMARK_REFERENCE，Base轴解释仍有条件。Indoor为Azure Kinect深度快照，不代表MID360验证已完成。

## 成本

|轨迹|输入到最终确认 Mean/P95/Max ms|NDT回调 Mean/P95/Max ms|墙钟 s|NDT/EKF/同时峰值 KiB|
|---|---|---|---|---|
|EASY_C0_B_strict_01|23.902/53.521/94.857|18.182/49.288/90.661|142.529|93104/17124/109016|
|EASY_C0_C_strict_02|19.232/47.332/103.678|13.937/42.954/95.620|138.471|95232/17116/112168|
|NC_W1_C3_C_strict_01|83.395/135.035/1267.465|59.445/108.819/1240.149|10.372|369144/21460/390220|

输入到确认时间包括第一次扫描的地图初始化代价；相邻完成间隔另列R31_RUNTIME_MEMORY.csv，不把剔除首帧后的间隔冒充全部成本。RSS为50ms采样的实际进程RSS峰值（同时峰值非两峰相加）。启动/清理、阻塞式输入水位、操作系统调度均影响墙钟，不能据本轮成本宣称10Hz。C回调均值低于B不是算法加速证据。

## 实际命令及运行异常

```bash
# 本轮执行工作区，不能在稳定机器狗工程执行
cd /home/jian/livox_ws/p62_r31_observation_identity_ws
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="dog_prior_map_interfaces;dog_prior_map_localization" --make-args -j2 dog_prior_map_ndt_node_cpp dog_prior_map_ekf_node_cpp newer_college_sequence_publisher observation_identity_contract_test
(cd build && ctest -R '^observation_identity_contract_test$' --output-on-failure)
# 以下是已执行顺序的记录，不能再次执行并覆盖原结果
python3 docs/p62_r31_observation_identity/tools/run_all.py
python3 docs/p62_r31_observation_identity/tools/resume_unstarted.py
python3 docs/p62_r31_observation_identity/tools/evaluate_runs.py
python3 docs/p62_r31_observation_identity/tools/finalize_archive.py
```

第一批B完成；C第一次启动时，监督器在EKF尚未出现时误判退出，零PCD、零IMU处理、零NDT。只修正新监督器的进程出现判定，再执行尚未输入的C和NC；未重跑B或任何已处理的正式轨迹。原零输入目录、计划和回执完整保留，见ZERO_INPUT_STARTUP_ABORT.json。实际完成轨迹3条、启动尝试4次，932次NDT包括所有正式调用；零输入启动无NDT。每事件≤2、每60输入帧≤12，最大实测12，无预算超限。

每条确切roslaunch参数、时间边界、初始化与二进制SHA见raw/*/RUN_COMMAND.json.gz及RUN_RECEIPT.json.gz。实际调用逐条记录在r23_candidates.csv.gz。运行原始日志位于/home/jian/livox_ws/p62_r31_observation_identity_data/；归档保留全部根级结构化日志压缩副本和安全的ROSLAUNCH stdout，逐文件SHA见evidence/PORTABLE_RAW_MANIFEST.json。ROS内部环境转储不放入Git。

## 版本及资产保护

现有LEGACY默认行为、稳定工程与旧归档未改。R30网格初始化顺序修复已继承；R29使用的Tile路径此前已正确，前raw62原生矩阵最大差0，所有Indoor输入cloud_hash一致，见HISTORICAL_RUNTIME_CORRESPONDENCE.json。没有新增LEGACY完整运行，没有冒称其他工作区未提交内容已处理。STABLE_BEFORE/AFTER相同，四个冻结二进制SHA在运行前后相同。磁盘占用与可再生文件仅列清单，不删除历史资产。

正式判定及唯一NEXT见R31_SCIENTIFIC_DECISION.md。禁止将ENGINEERING_CONTRACT_PASS写成LOCALIZATION_PERFORMANCE_PASS或SCI创新成立。

ENGINEERING_CONTRACT_PASS限定为本轮观测身份、拒绝状态和已有因果时间不变量闭合，不代表R30已知IMU大间隔未积分问题或所有系统基础问题已经解决。R31_STORAGE_AUDIT.md及evidence/STORAGE_AUDIT.json给出实际空间；未批准删除，删除0。
