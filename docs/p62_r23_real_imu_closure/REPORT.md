# P62-R23 REAL-IMU LOCALIZATION CLOSURE

本轮完成真实Ouster IMU → 原有传播/Deskew → 原生PCL NDT → C/D观测选择 → 原有EKF/OOSM的研究版闭环，并完成10条独立因果轨迹及一键RViz实际显示。正式结论为 **REAL_IMU_EKF_CLOSURE_ESTABLISHED_WITH_LIMITATIONS**。

**D没有获得独立收益：四组C/D输出与参考相对误差完全相同，仅相对准入均为0。依照任务预设决策，停止将D作为论文核心候选，C保留为简单强基线。** 工程接入完成不能转述为论文创新或定位可靠性通过。W1/C3是完整保留的失败：60/60帧在原有优化前几何门被拒绝，真实IMU继续传播但EKF显著漂移。

## Git、代码与运行顺序

- 仓库：https://github.com/Liumengjian666/Map-Matching
- 工作区：`/home/jian/livox_ws/p62_r23_real_imu_closure_ws`
- 分支：`research/p62-r23-real-imu-localization-closure`
- START：`5b3a18405e6168befbe55a8158f282f09607d684`
- Smoke CODE：`01af9138d418f7d90390195fa88dfe975ac29d36`
- 正式算法 CODE：`6da62ba328bbaecae259d729166533b5a383739d`
- 全10条原生输出封存提交：`970ae495eec18c629403cbf40cc2d6bbd4166489`
- RViz显示入口CODE：`028bdf40e19b0c0ed95b37ec4a28b7b5c6e132cc`；后续只补EOL显示环境变量及归档，不改变正式C++或实际运行二进制。
- END为包含本报告的最终本地归档提交，完整SHA由最终交接及外置`FINAL_GIT_RECEIPT.json`提供；不将commit自身SHA伪造写入其自身内容。未push。

先完成20帧BASELINE Smoke并冻结G1回执，再集成C/D、冻结源码及二进制，随后分别执行10条60扫描轨迹；封存全部原生输出后才执行参考误差评价。正式冻结在`FORMAL_CODE_FREEZE.json`，170份原生结果在`NATIVE_OUTPUT_FREEZE.json`逐项带SHA；该封存文件SHA为`a17ec6d1c0a902a1e3771e2cdabccdc23c266261dff6d1608a3763230ec77c70`。GUI另运行20帧BASELINE，仅用于显示验收。

## 输入、坐标、时间和保护

真实输入为本地Newer College `01_short_experiment`原始PCD及Ouster IMU CSV；122帧原始输入身份、点格式和边界IMU记录于`input/INITIALIZATION_AND_TIME_CONTRACT.json`。IMU 153014行，SHA `b733ce23544ed23a3ab7a627acfd61ea329f882b12ef0cbcb501b6b13e5828be`。后续测试参考从不进入在线进程。

使用原P45 tiles而非R22的0.20m静态ROI：manifest SHA `ebd7089f20026a991af1a88679a133bce020ee7bc9a9a5439665cc3a413cbb04`，74个20m tile、全局3142751点、既有0.15m目标。原地图SHA `57da358a7c3f52a132fd06afba69f2c3eff2e980b7dbad5b8751548665104e25`来源于已有manifest，本轮没有伪称重新读取全大地图校验。实际C/D匹配目标集合在相同窗口/初始化下相同：W1/C0 2380375点、W1/C3 2297615点、W3/C0 2439500点、W3/C3 2383209点；每帧FNV、tile IDs和点数均记录，不能宣称所有初始化使用完全相同的动态目标。

`T_A_B`将B坐标转换到A。`T_imu_lidar`旋转diag(-1,-1,1)，平移[-0.006253,0.011775,-0.028535]m。EKF内部IMU状态，NDT/高频接口为LiDAR位姿：`T_map_imu=T_map_lidar*inverse(T_imu_lidar)`。C3在锚点LiDAR姿态左乘地图Z轴+8°，保持LiDAR平移后重算IMU姿态和杆臂位置，两个模块初始化一致。

W1 anchor2600、评价2601–2660；W3 anchor6500、评价6501–6560。仅锚点参考初始化，v/bg/ba=0是既有初态先验，不是实测静止/bias估计。没有额外预热，逐条重放anchor边界之后的真实IMU。scan header为start，点t为相对start纳秒，Deskew仍参考start；NDT观测时间也是start，计算完成墙钟不代替采样时间。

真实TP取已收到的EKF历史样本（sample≤scan start，age≤20ms），经现有有界body-twist接口查询；没有第二传播器或R14B常速度历史预测替身。原OOSM在允许20ms误差范围内回滚历史状态并重放真实IMU；日志区分measurement、rollback、current三时刻，不能声称在测量时间精确插值更新。正式418次均`APPLIED`，没有OOSM拒绝。

**已知预测新鲜度限制：** Smoke后续19/19帧扫描起点历史高频样本落后最近完成地图校正一个revision。前帧结束括号IMU已推进过下一扫描起点，而OOSM不重新发布旧时间高频样本；记录仍然因果，地图校正会进入后续IMU，但不能宣称最新后验历史同步无延迟。本轮没有修改传播/历史发布模型。

官方参考仍是条件性Base解释的`MAP_ANCHORED_BENCHMARK_REFERENCE`，不是独立动捕真值。Full优化使用了相关点，历史`heldout_mean`字段在本轮称Coverage Consistency Mean，不是独立留出证据。参考评价不做事后SE3对齐。

稳定工程前后HEAD `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`、分支、status、index和保护文件完全一致；R6–R22归档不改。只修改研究工作区的NDT接口、发布器、研究launch/RViz及本轮归档。现有IMU/EKF/OOSM/Deskew/地图管理数学源码不变。

## 实现与阶段验收

`dog_prior_map_ndt_node.cpp`增加默认关闭的`research/r23_enable`和三个显式策略；真实EKF来源/样本时间门；当前实际目标地图身份；原始、选中及现有限步后发布位姿独立日志。C/D直接调用冻结R20/R21头文件，不重写决策语义。保留原生NDT .8/.08/.001/40/.55、Full Source、原始预处理及Coverage V2；额外地图yaw±5°各一次、每轨迹最多12次。

`newer_college_first_scan_publisher.cpp`增加anchor起始IMU重放及显式measurement-published融合ack，拒绝时不等待虚假NDT校正；原生收敛与实际发布分开。现有相邻观测0.5m/5°限步与EKF观测限制保留，raw候选、选中候选、实际发布观测不能混淆。

|门|事实及判定|
|---|---|
|G0|真实IMU/PCD、合法anchor边界、外参/地图合同通过；保留初态与Base限制|
|G1|20扫描Deskew、20原生NDT、210次实际高频传播、20观测和20 OOSM更新；PASS_WITH_DELAYED_PREDICTION_LIMITATION|
|G2|C接入实际测量链，W3/C3额外候选真实进入EKF；拒绝时没有发布Prediction冒充NDT|
|G3|10条独立真实因果实验完成；C/D没有独立差异，W1/C3实际定位失败，不能称全部初始化可靠通过|
|RViz|实际20扫描显示，Global Status Ok、真实地图/点云/轨迹订阅及截图；首次EOL弹窗有一次手动关闭；最终抑制变量重启未完成验证|

## 全部正式轨迹与参考相对误差

以下均为完整60扫描，EKF按真实输出自己的时间匹配参考：accepted帧使用该次真实校正后current状态，rejected帧使用该次真实IMU watermark高频状态。其时间晚于scan start约99.500–110.224ms；不能用scan-start参考给current状态算误差。NDT原始输出则在scan start评价。600/600 EKF样本参考有效，插值缺口最大100.720ms；固定缺口上限150ms、近端距离≤50ms。拒绝地图测量不写成零误差，原生未尝试记NA。

|轨迹|EKF平移RMSE/P95/Max m|EKF旋转RMSE/P95/Max °|原始名义NDT RMSE m/°|
|---|---|---|---|
|W1/C0/BASELINE|0.191979/0.258302/0.324871|2.115287/2.829333/3.690048|0.174244/2.014626|
|W1/C0/MULTISTART_C|0.191979/0.258302/0.324871|2.115287/2.829333/3.690048|0.174244/2.014626|
|W1/C0/RELATIVE_D_EXPERIMENTAL|0.191979/0.258302/0.324871|2.115287/2.829333/3.690048|0.174244/2.014626|
|W1/C3/BASELINE|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|NA/NA|
|W1/C3/MULTISTART_C|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|NA/NA|
|W1/C3/RELATIVE_D_EXPERIMENTAL|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|NA/NA|
|W3/C0/MULTISTART_C|0.175791/0.222859/0.361382|2.009151/2.720610/2.880309|0.164635/1.942995|
|W3/C0/RELATIVE_D_EXPERIMENTAL|0.175791/0.222859/0.361382|2.009151/2.720610/2.880309|0.164635/1.942995|
|W3/C3/MULTISTART_C|0.174696/0.230222/0.260149|2.025760/2.814523/2.931799|0.177435/2.254654|
|W3/C3/RELATIVE_D_EXPERIMENTAL|0.174696/0.230222/0.260149|2.025760/2.814523/2.931799|0.177435/2.254654|

|轨迹|名义尝试/收敛|发布/拒绝|优化前拒绝|质量事件|extra|选中extra|relative-only|高频IMU传播|实际融合/OOSM APPLIED|最长无地图扫描|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|W1/C0/BASELINE|60/60|60/0|0|0|0|0|0|611|60|0|
|W1/C0/MULTISTART_C|60/60|60/0|0|0|0|0|0|611|60|0|
|W1/C0/RELATIVE_D_EXPERIMENTAL|60/60|60/0|0|0|0|0|0|611|60|0|
|W1/C3/BASELINE|0/0|0/60|60|0|0|0|0|611|0|60|
|W1/C3/MULTISTART_C|0/0|0/60|60|0|0|0|0|611|0|60|
|W1/C3/RELATIVE_D_EXPERIMENTAL|0/0|0/60|60|0|0|0|0|611|0|60|
|W3/C0/MULTISTART_C|60/60|60/0|0|0|0|0|0|611|60|0|
|W3/C0/RELATIVE_D_EXPERIMENTAL|60/60|60/0|0|0|0|0|0|611|60|0|
|W3/C3/MULTISTART_C|60/60|59/1|0|3|6|2|0|611|59|1|
|W3/C3/RELATIVE_D_EXPERIMENTAL|60/60|59/1|0|3|6|2|0|611|59|1|

实际正式调用420名义+12extra=432；600扫描中180帧在优化前被拒绝，不能虚报600次align。418地图观测确实接收并完成EKF历史更新，6110次原始IMU高频传播（OOSM重放另计）。所有420已尝试名义均收敛，W1/C3的0尝试收敛率应为NA。正式2次优化后拒绝发生于C/D各自W3/C3最后一帧。

## 成功事件与完整失败链

W3/C3，C与D完全相同：raw6501、6502均触发质量搜索，选择-5°初值的id2原生终值并通过绝对门进入EKF。封存后的参考评价显示，raw6501选中候选误差0.191925m/1.836112°，较同帧名义改善0.150961m/3.856724°；raw6502为0.089110m/1.593754°，改善0.267400m/4.371692°。这是同帧参考相对改善，不是独立真值证明，也没有W3 BASELINE完整闭环对照可据以归因整段RMSE收益。

raw6560（末帧）第三次质量事件再用2次搜索，无候选通过绝对门，真实拒绝、不发布观测；IMU watermark继续推进约100ms。该事件之后没有批准的后续扫描，不能宣称末次拒绝后长期恢复。各轨迹extra6≤12，所有D relative-only=0。

W1/C3，三方法60扫描全部被原P41优化前几何支持门拦截，align0、观测0、extra0，无法触达C/D搜索。真实IMU传播611次，首末状态位置相差16.935m，未锁死上一地图观测，但参考相对平移RMSE7.986422m、最大18.755587m；姿态RMSE13.052649°。这是预优化门与受扰初始化下的系统失败，不可混成多初值优化器失败、额外预算耗尽或数据缺失。没有改门、换窗口或追加搜索。

四个匹配C/D条件的完整姿态输出、RMSE、高误差帧及回退长度均完全相同；每组actual target集合也相同。W1/C0相对BASELINE没有变化。D特殊机制未被执行，既无新的独立收益，也不据此消除R22已知风险。

## 成本与真实调用预算

下表NDT callback包括预处理、当前局部地图和决策路径；input-to-ack完整传感器到融合延迟逐帧另在`RUNTIME_MEMORY.csv`，包括外置Deskew/传输等待，不能只用align耗时描述整链。进程墙钟包含启动与关闭；逐帧地图构建、tile IO、align等分别保存。

|轨迹|callback Mean/P95/Max ms|除首帧Mean/P95/Max ms|进程墙钟s|NDT/EKF/同时合计峰值KiB|
|---|---|---|---:|---|
|W1/C0/BASELINE|50.030/46.479/1004.566|33.852/45.223/51.565|9.249|381148/21968/403044|
|W1/C0/MULTISTART_C|49.609/43.306/1045.361|32.732/40.935/51.273|9.407|380184/22752/402328|
|W1/C0/RELATIVE_D_EXPERIMENTAL|50.218/43.902/1044.504|33.365/41.193/48.047|9.315|380452/21200/401580|
|W1/C3/BASELINE|30.467/39.016/601.664|20.785/38.040/58.257|8.100|256644/23544/280188|
|W1/C3/MULTISTART_C|30.352/39.254/602.224|20.660/38.179/43.350|8.051|259060/23208/282196|
|W1/C3/RELATIVE_D_EXPERIMENTAL|32.642/43.989/714.881|21.079/43.493/49.235|8.308|256976/23508/280320|
|W3/C0/MULTISTART_C|50.991/53.630/1037.352|34.273/47.864/58.643|9.610|385700/22752/407080|
|W3/C0/RELATIVE_D_EXPERIMENTAL|50.273/44.807/1034.950|33.584/44.082/55.350|9.409|385652/23480/409060|
|W3/C3/MULTISTART_C|53.764/54.100/1045.225|36.959/51.131/142.298|9.556|378520/21920/400276|
|W3/C3/RELATIVE_D_EXPERIMENTAL|54.626/57.102/1066.492|37.475/55.019/144.695|9.701|378184/23304/399404|

首次callback601.664–1066.492ms，包含一次地图目标准备；各次first target build14.69–25.53ms，target rebuild513.33–631.22ms，tile IO53.38–73.34ms（各测量边界不同、不可无条件求和）。W3/C3事件帧最坏C1045.225ms、D1066.492ms，含首帧初始化；不能用平均约50ms宣布10Hz。资源峰值来自监督器50ms采样，可能漏过短尖峰；NDT独立采样峰值385700KiB、EKF23544KiB；同时两节点峰值409060KiB，后者不含publisher/Deskew/RViz等辅助进程，不能冒称整机RSS。

调用总账：基础Smoke20 + 正式432 + GUI显示20 = **472次Full**；其中extra总共12，全部属于正式C/D，各条≤12。没有Split/Coupled/E/真实EKF模型改动。GUI成本不混入正式精度或headless资源指标。正式二进制外置保存在`/home/jian/livox_ws/p62_r23_real_imu_data/binaries/`，路径、SHA和大小见`BINARY_IDENTITY.json`。

## 构建、运行、必要验证

Release构建使用本研究catkin工作区、ROS Noetic及只读根工作区livox_ros_driver2 underlay。命令/原始输出见logs，正式每条ROS完整argv见`formal_runs/*/RUN_COMMAND.json`。主要命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="dog_prior_map_interfaces;dog_prior_map_localization" --make-args -j2 dog_prior_map_ndt_node_cpp dog_prior_map_ekf_node_cpp newer_college_sequence_publisher
```

已有causal_pose_prediction_test、frame_conversion_contract_test通过，launch XML/解析检查通过。形式接口测试不冒充闭环证据；实际闭环证据为收据/IMU/Deskew/NDT/OOSM逐帧关联。结束前再次验证170份封存原生文件、原策略头、实际C++、模型与二进制SHA和稳定工程快照；显示launch后续变化单独记身份，不冒充旧freeze文件未变。

真实一键RViz命令见README/RVIZ_ACCEPTANCE；机器狗rosbag输入合同也在README，但本轮未运行机器狗数据，不能声称实物或bag模式验收。首次GUI弹窗手动关闭事实及后续环境变量检查未完成回执均保留。

## 科研边界和唯一NEXT

本轮得到真实IMU传播与真实地图观测历史融合的工程证据，C能把两次通过绝对质量门的替代候选送入EKF；D没有独立收益。不能据此宣称独立物理精度改善、跨场景泛化、10Hz、论文创新成立或初始化可靠性通过。与R22相比，Deskew、目标地图和预测/滤波链均变化，不能将差异单独归因于IMU。

**唯一NEXT：保留C为强基线、停止D核心创新，下一轮仅处理现有系统优化前几何门的责任边界及OOSM历史预测新鲜度，再决定机器狗实测闭环合同；不再开发新的离线候选搜索。** R23到此停止，等待科研总控核查真实代码和原始归档后决策。

关键源码点位：`src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`中R23开关约483行、真实历史EKF查询约1237行、冻结策略接入1283行、优化前几何门2420行、实际地图观测发布2732行及lineage约2987行；`newer_college_first_scan_publisher.cpp`显式观测回执约275行、anchor IMU起点约447行。新launch及RViz位于该ROS包对应目录。
