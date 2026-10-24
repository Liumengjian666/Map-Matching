# P62-R25 — Results-to-Paper Closure

**完成：G-only真实因果消融、资源/方法整理、完整英文初稿及图表。未完成：机器狗新对照，因真实LiDAR–IMU外参来源无法闭合而按合同停止。**

## Git与执行身份

- 仓库：https://github.com/Liumengjian666/Map-Matching
- 分支：`research/p62-r25-results-to-paper-closure`
- 工作区：`/home/jian/livox_ws/p62_r25_results_paper_ws`
- START：`e2673afc388e56cf7ed2bbc1997391740fe23b33`
- 实际NDT CODE：`131a900842e965305465220e777c8d6949c5f17a`
- 运行前冻结：`d86831e6fe608a9b2b7c83189c3db6f90258cdab`及CODE_FREEZE.json。
- 原生输出封存：`0d4a56f55b6550a61d5c97fa6945d853cc8e49a5`，随后才执行参考评价。
- END为包含本归档的最终本地提交，完整SHA见最终交接和外置FINAL_GIT_RECEIPT.json；一个提交不在自身内容中伪造自身SHA。
- 本轮不push，不修改稳定机器狗工程，不改写R19–R24。

## G-only消融

A直接引用R23 W1/C3 MULTISTART_C在原前置门下的轨迹；C直接引用R24的G实验，即C+G。未重跑A/C。B为本轮真实独立状态的60扫描运行，G开启、额外搜索关闭、F关闭，原基本质量及Coverage Consistency Mean<=.30完全不变。

|方法|EKF平移RMSE/P95/Max m|EKF旋转RMSE/P95/Max °|名义/extra Full|地图接纳/拒绝|高误差帧|最长无地图|首次持续低误差raw|
|---|---|---|---:|---:|---:|---:|---|
|A R23原前置门|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|0/0|0/60|60|60|NONE|
|B R25 G-only|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|60/0|0/60|60|60|NONE|
|C R24 C+G|0.209943/0.300124/0.444282|2.984149/3.723287/11.245245|60/8|57/3|2|2|2603|

B60次名义均原生收敛，60次都未被原后质量规则接纳；真实IMU继续传播，不是假地图观测，也不是旧位姿冻结。B的完整EKF及TP矩阵序列与A逐帧一致。原始输入、IMU、初始化、Deskew与地图规则不变；实际目标指纹仍为11658115094751181825、2297615点。各策略因果历史改变后，Deskew结果允许反映自己的状态，不能伪称它们的处理后点坐标仍必须逐点相同。

C的-5°种子在raw2603给出第一条合格地图观测，Coverage=.296356。raw2604仍拒绝；raw2605之后名义观测持续接纳。首次地图更新和首个在余下批准窗口持续满足既有.5m/5°参考误差界限的帧均为raw2603；后者是封存后评价，不是在线准入条件。

**结论：当前开发实例中G提供优化可达性，但G-only不足；固定多初值具有额外恢复贡献。** 不将这一实例写成一般充分性定理、新颖多初值方法或跨数据集验证。C+G仍有2帧旋转误差>5°、最大11.245245°。方法完成消融后未再修改。

成本：B callback Mean/P95/Max=110.496/179.406/977.302ms，NDT/EKF/同时两节点采样峰值369136/21352/389380KiB，进程墙钟13.308355s。C历史callback=55.304/111.302/1006.031ms、extra8，进程墙钟9.655876s；不能将不同因果分支的单次墙钟差包装成普遍加速。完整三组指标在ALGORITHM_ABLATION.md和ABLATION_TABLE.csv；未宣称10Hz。

## 机器狗结果

DOG_BASELINE / DOG_CG_EXPERIMENTAL均NOT_RUN，新增DOG align=0。

完整loop2 bag实际存在并可读取索引：19677401800bytes、1090.511421s、10905条Livox点云、218101条真实IMU。地图及历史initial_guess记录SHA分别重新核验为016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca、e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a。19GB bag的完整SHA取历史冻结期望，本轮停止于坐标门后未再次全量读取，不伪称当前完整payload新验真。

实际loop2 FAST-LIVO2建图配置明确将extrinsic_R=I、extrinsic_T=0作为没有单独标定时的假设；own_loop2.yaml也明确UNAVAILABLE_NOT_PHYSICALLY_CALIBRATED。相同livox_frame名称不是物理外参。不得借用Avia或Ouster外参，不通过参考拟合，因此坐标链不满足合法运行前提，立即停止新的机器狗对照。

原稳定配置是CustomMsg、scan mid、Deskew OFF、200样本重力初始化/OOSM，与Newer College不同。lidar_poses.txt仅为同系FAST-LIVO2建图参考，历史对比还包含首共同位置相减，不能移作R25独立精度结果。没有确认.30门在Mid-360中的完整适用性。详见DOG_VALIDATION.md、DOG_ASSET_CONTRACT.json及只读原配置快照。

新增p62_r25_dog_asset_view.launch及RViz为**未执行的可视化配置**：地图、实时估计器点云、NDT位姿/轨迹、EKF轨迹显示项已配置；可选参考项默认关闭。它不启动估计器，不能仅通过播放原始bag就宣称DOG_CG验收；本轮不提供已认证的完整CG定位启动命令。R23实际GUI验收只属于Newer College，不移用于机器狗。

## 论文交付

- **[PAPER_DRAFT.md](PAPER_DRAFT.md)**：完整英文第一稿，包含Abstract、Introduction、Related Work、Methodology、Experimental Setup、Results、Discussion、Conclusion及引用。
- [PAPER_METHOD_SUMMARY.md](PAPER_METHOD_SUMMARY.md)：冻结C+G的实现准确公式、质量定义和可用主张。
- [ALGORITHM_ABLATION.md](ALGORITHM_ABLATION.md)：A/B/C完整消融与恢复链。
- [RESOURCE_COMPARISON.md](RESOURCE_COMPARISON.md)：P45与当前实际资源，明确测量范围不同。
- [EVIDENCE_LEDGER.csv](EVIDENCE_LEDGER.csv)：P45、R19–R25各证据源与SHA，避免混用协议。
- figures/：误差曲线、实际Coverage准入曲线、P45历史内存图，均提供PNG/PDF/SVG及误差图源CSV。

论文保留P45局部地图内存收益及时间/尾误差代价、R19稳定不同终值、R20简单质量/多初值成功和失败、R21开发正例/R22安全反例、R23真实融合、R24责任边界及F未被查询消费，以及R25消融和机器狗阻塞。没有将D/E/F/Coupled写入保留方法。

写作以源码为准：R20 C的r17::quality/r20::accepted没有调用r17::base中的TP .8m/5°硬界；.8m/5°在排序距离中是归一化尺度。另有真实相邻NDT .5m/5°发布限步及EKF更新界限。论文明确区分，未为契合旧叙述而改冻结源码，也未虚构独立留出或D>.02门在当前C里执行。

数据为条件性Base下的MAP_ANCHORED_BENCHMARK_REFERENCE，Full Coverage点参与优化，不能称独立动捕或独立留出。已知开发窗口不称新盲测确认。论文第一稿文字完整，但并不宣称原创数学、独立物理精度、跨场景泛化、机器狗部署通过或可直接投稿。

## 编译和实际命令

NDT实际Release构建：
```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="dog_prior_map_interfaces;dog_prior_map_localization" --make-args -j2 dog_prior_map_ndt_node_cpp
```
未修改的EKF和真实序列发布器直接复用已保留的R24二进制并核验SHA，避免无意义重编译；本轮NDT及三份实际二进制身份见CODE_FREEZE.json，外置保存在/home/jian/livox_ws/p62_r25_results_paper_data/binaries/。

实际唯一因果实验：
```bash
python3 docs/p62_r25_results_to_paper_closure/tools/run_ros.py --mode G_ONLY --window W1 --arm C3 --run-dir /home/jian/livox_ws/p62_r25_results_paper_data/g_only_w1_c3_01
```
实际ROS完整argv及独立11385 master环境见formal_runs/g_only_w1_c3_01/RUN_COMMAND.json。输出目录排他，不覆盖重跑。source/launch检查与原生预算/因果检查通过；封存后评价和绘图源码均保存。

## 正式收束

本轮新增Full align总数60，extra0，DOG0；A/C未重跑。机器狗坐标阻塞不延误论文初稿。稳定工程与历史归档保持不变。方法不再修改，不进入新算法阶段。

**唯一NEXT：科研总控审阅完整英文初稿、消融和证据表，决定最终论文主张及投稿定位；机器狗部署必须先补齐可信外参合同后另行安排，不再继续离线恢复算法探索。**
