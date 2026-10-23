# P62-R21 — Relative Admission and Temporal Confirmation

**结论：单帧相对准入D在当前开发案例达到强窗恢复工程目标；两帧确认E没有独立收益，优先保留简单D。** Strong/C3的D为0.158692m/2.071567°，E为0.160088m/2.443303°，两者都满足RMSE<=0.35m且<=3°。E比D多1次原生确认、延迟100.502016ms，旋转RMSE增加0.371736°。该结论仅涉及已暴露参考的开发窗口，不证明泛化、独立正确性或论文创新。

## Git、实现和输入

START_SHA：`9ffb2349ebe83022f3f03e49bd618e00ed267fb9`，R20本地与远端归档起点一致。分支 `research/p62-r21-relative-temporal-admission`；独立工作区 `/home/jian/livox_ws/p62_r21_relative_temporal_ws`。实际运行 CODE_SHA：`199a98e1a4f98cc4a23b44516290d6666edf29fa`。END_SHA为包含本报告的最终Git提交，完整值在收尾交接提供，文件不构造自引用哈希。仅本地提交，不push。

源码：`tools/policy.hpp`定义可比较名义指标、严格Pareto、确定性排序、Pending隔离及预算；`tools/run_relative.cpp`基于R20 runner最小扩展D/E在线状态机；`tools/seal.py`封存输入和原生输出；`tools/evaluate.py`封存后计算参考误差并直接引用三条历史基线。R6 NativeNdt、R9 Coverage V2、R14B Predictor与R20质量/种子/排序函数均直接复用，未修改历史源码。实际Release二进制SHA256：`6f991e6d2af53b1c66a298d1f814bf580bc1ab8c79b99005a6f8dc80602536ff`。

固定两个R19窗口和地图：WEAK anchor14542、评价14543–14602；STRONG anchor10972、评价10973–11032。每窗C0/C3，各D/E独立因果状态，共120个唯一测试扫描、8条新60帧轨迹。R19 A、R20 B/C共12条历史轨迹只引用，不重跑。

- WEAK map SHA256：`974056be8b2146b89fe6e8ec03326000498c5afe12d6aed65893e563d7fe95e1`。
- STRONG map SHA256：`4cd10f10160eaf8f2570f7627e78a22491dea5f7a267a7dceaa56c0b68dcf3e7`。
- 全部122帧PCD SHA、Source点数/FNV、锚点、地图与复用源码身份在`INPUT_IDENTITY.json`，运行前后匹配。输入冻结SHA `d6da0e16cf64a7a865747893eb54f8572920758504217ae43eaa697d50f27159`。
- 所有原生输出先封存，SHA `d5874f3214652d744ff3e472283a84a6cc69ad6aac846662d8934feeb70e35b0`，提交`ac27755`后才评价参考。生产工作区HEAD/branch/index/status前后一致；历史归档未修改。

## 方法和确认合同

名义及所有extra都是真实Full PCL NDT，固定参数resolution=.8、step=.08、epsilon=.001、max iterations=40、outlier=.55。基本质量门与0.30绝对门不变；R20的map-yaw ±5°两个种子不变。绝对通过优先，使用Native Score最大及原有距离/ID同分规则。

只有所有候选都没过绝对门时，检查额外候选对有效可比较名义结果的三项严格优势：Score↑、Overlap↑、Coverage mean↓，并存在>.10m或>1°的物理差异。D直接接纳并标为RELATIVE_ONLY_ADMISSION。E保存pending，当帧仍输出预测回退；下一帧只从该pending的一历史位姿保持初值进行一次真实Full确认。确认通过绝对门或再次严格相对支配后，只写入当前帧位姿；原pending不提前写入或事后回填。

用户明确确认：若下一帧名义结果已过绝对门，直接接纳名义、撤销pending、零extra。该取消分支本次真实数据未触发，只能报告实现合同，不能声称获得其真实效果证据。确认失败则删除pending并回退；确认帧不再开启新yaw搜索。每轨迹12次预算同时覆盖搜索和确认，正常帧0extra。边界和取消理由详见METHOD.md。

Full优化已使用奇数扇区，因此继承字段heldout_mean只是Coverage几何质量统计，不是未参与优化的独立留出验证。在线决策没有读取测试参考。参考是MAP_ANCHORED_BENCHMARK_REFERENCE，Base轴解释有条件，无事后SE3对齐；误差不属于独立动捕真值精度。

## 完整轨迹误差（每行60帧）

| 窗口/初始 | 策略 | 平移 RMSE / P95 / Max (m) | 旋转 RMSE / P95 / Max (°) |
|---|---|---|---|
| WEAK/C0 | R19 A | 0.260090 / 0.347973 / 0.391645 | 1.827785 / 2.449893 / 3.634624 |
| WEAK/C0 | R20 B Gate | 0.265708 / 0.353501 / 0.394637 | 1.858396 / 2.588184 / 4.156104 |
| WEAK/C0 | R20 C Multi | 0.263902 / 0.359318 / 0.391822 | 1.858832 / 2.538737 / 3.701357 |
| WEAK/C0 | R21 D Relative | 0.263902 / 0.359318 / 0.391822 | 1.858832 / 2.538737 / 3.701357 |
| WEAK/C0 | R21 E Confirm | 0.263902 / 0.359318 / 0.391822 | 1.858832 / 2.538737 / 3.701357 |
| WEAK/C3 | R19 A | 0.283121 / 0.485931 / 0.622737 | 2.808664 / 7.522614 / 9.018234 |
| WEAK/C3 | R20 B Gate | 4.164507 / 6.701954 / 7.000428 | 7.016976 / 10.693101 / 11.649997 |
| WEAK/C3 | R20 C Multi | 0.266655 / 0.356058 / 0.393714 | 1.862490 / 2.486138 / 4.308431 |
| WEAK/C3 | R21 D Relative | 0.266655 / 0.356058 / 0.393714 | 1.862490 / 2.486138 / 4.308431 |
| WEAK/C3 | R21 E Confirm | 0.266655 / 0.356058 / 0.393714 | 1.862490 / 2.486138 / 4.308431 |
| STRONG/C0 | R19 A | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C0 | R20 B Gate | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C0 | R20 C Multi | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C0 | R21 D Relative | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C0 | R21 E Confirm | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C3 | R19 A | 0.947759 / 2.145176 / 2.309913 | 7.788894 / 15.936936 / 22.616286 |
| STRONG/C3 | R20 B Gate | 0.351154 / 0.877361 / 1.177489 | 4.143002 / 9.983213 / 12.735693 |
| STRONG/C3 | R20 C Multi | 0.351154 / 0.877361 / 1.177489 | 4.143002 / 9.983213 / 12.735693 |
| STRONG/C3 | R21 D Relative | 0.158692 / 0.187519 / 0.211521 | 2.071567 / 2.913251 / 3.979997 |
| STRONG/C3 | R21 E Confirm | 0.160088 / 0.189015 / 0.258568 | 2.443303 / 3.670174 / 9.076500 |

这里评价实际因果输出，包括合法预测回退。REFERENCE_ERROR.csv同时保存TP、Raw、正式Selected和Output；拒绝时Selected为NA，未记零误差。历史基线记录原样引用。C0两窗均未超过0.03m或0.3°明显恶化界限：弱C0的D/E相对R19增加0.003811m/0.031046°；强C0无变化。D/E的WEAK C0/C3和STRONG C0输出与R20 Multistart逐矩阵一致。

## 相对准入、确认及调用

| 窗口/初始 | 策略 | 异常帧 | Pareto候选 / 仅相对候选 | 直接相对接纳 | 建立pending | 确认成功/尝试/失败 | 回退/最长 | 额外Full |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| WEAK/C0 | R21 D Relative | 2 | 1 / 0 | 0 | 0 | 0/0/0 | 1/1 | 4 |
| WEAK/C0 | R21 E Confirm | 2 | 1 / 0 | 0 | 0 | 0/0/0 | 1/1 | 4 |
| WEAK/C3 | R21 D Relative | 4 | 2 / 0 | 0 | 0 | 0/0/0 | 2/1 | 8 |
| WEAK/C3 | R21 E Confirm | 4 | 2 / 0 | 0 | 0 | 0/0/0 | 2/1 | 8 |
| STRONG/C0 | R21 D Relative | 0 | 0 / 0 | 0 | 0 | 0/0/0 | 0/0 | 0 |
| STRONG/C0 | R21 E Confirm | 0 | 0 / 0 | 0 | 0 | 0/0/0 | 0/0 | 0 |
| STRONG/C3 | R21 D Relative | 2 | 2 / 1 | 1 | 0 | 0/0/0 | 0/0 | 4 |
| STRONG/C3 | R21 E Confirm | 3 | 3 / 2 | 0 | 1 | 1/1/0 | 1/1 | 5 |

Pareto候选数含同时过绝对门者；“仅相对候选”是未过绝对门的Pareto候选。D的直接相对接纳只有1次；E建立pending 1次、真实确认1次、相对确认成功1次、确认失败0次；没有正常名义取消、预算到期或窗口到期实例。原生调用总计 **480名义+32搜索+1确认=513/576**，D共16额外、E共17额外，单轨迹最高8次。所有候选和调用均有原始CSV，Split/EKF/Coupled及历史基线重跑为0。

## Strong/C3完整决策链与反例

`STRONG_C3_COMPLETE_CHAIN.csv`保存从raw10973到11032的D/E各60帧完整链，包括每帧所有候选指标、决定、预算、pending状态和输出误差；不是只挑选成功帧。完整seed、终值和8块损失见RELATIVE_ADMISSION_LOG.csv，确认来源与官方历史时间见SHADOW_CONFIRMATION_LOG.csv。

1. raw10973，两策略同一名义：Score1.359323、Overlap0.607042、mean0.543936。−5°候选id2为Score1.958248、Overlap0.849296、mean0.361016；距名义0.069486m/5.453391°，故严格Pareto且物理不同，但没过0.30。D立即相对接纳；参考误差由0.167883m/8.868964°变为0.145811m/3.488735°。E不接纳，保留pending，当帧输出原TP，参考旋转误差9.076500°。
2. raw10974，D的新因果名义已过绝对门，误差0.176541m/2.630294°。E从上一帧pending终值（16个矩阵元素完全相同）对真实当前扫描重匹配。当前E名义为Score1.223323、Overlap0.580381、mean0.559279；确认id3为1.706428、0.731608、0.434215，距名义0.073168m/5.607854°，再次严格Pareto，按CONFIRMED_RELATIVE_ONLY_ADMISSION接纳。
3. **确认成功不等于正确恢复：** E的该确认结果仍有0.258568m/6.959934°误差。相对当前名义，旋转改善5.540322°，平移却恶化0.037885m。它同时改善三个在线质量指标，仍不能保证参考误差全面改善。E的正式历史在raw10973始终停留在anchor，raw10974成功后只追加当前确认位姿；没有提前泄露pending。
4. D在raw10975再使用一次+5°绝对通过候选，此后均名义接纳；E在raw10977使用+5°绝对候选，此后均名义接纳。D全60帧无>.5m或>5°输出；E前2帧高误差，raw10975开始连续3帧低误差，之后58帧未再次高误差。D首次连续低误差起于raw10974；D没有坏输出进入正式轨迹，不能将坏后恢复延迟伪报为0。

强窗工程目标两项同时通过，但E比D的平移RMSE增加0.001396m、旋转RMSE增加0.371736°，最大旋转误差9.076500°对比D的3.979997°。一次确认增加了100.502016ms扫描延迟和一次真实align，未展示减少错误接纳的作用。

WEAK/C3强对照完整保留：D/E均与R20 Multi逐帧相同，RMSE0.266655m/1.862490°，没有新增相对准入或pending机制事件。R20 Gate-only锁死反例依然报告为4.164507m/7.016976°，不作为本轮创新依据；本轮没有真实独立IMU预测。

## 运行成本

| 窗口/初始 | 策略 | 每帧Mean / P95 / Max (ms) | 事件帧Max (ms) |
|---|---|---|---:|
| WEAK/C0 | R21 D Relative | 21.352 / 27.466 / 52.268 | 50.025 |
| WEAK/C0 | R21 E Confirm | 20.524 / 24.685 / 50.853 | 50.853 |
| WEAK/C3 | R21 D Relative | 24.091 / 48.520 / 119.417 | 79.989 |
| WEAK/C3 | R21 E Confirm | 23.898 / 44.278 / 119.523 | 78.322 |
| STRONG/C0 | R21 D Relative | 23.290 / 28.467 / 118.236 | NA |
| STRONG/C0 | R21 E Confirm | 21.477 / 27.782 / 29.214 | NA |
| STRONG/C3 | R21 D Relative | 23.374 / 31.557 / 69.230 | 69.230 |
| STRONG/C3 | R21 E Confirm | 22.658 / 29.225 / 66.840 | 66.840 |

全480新帧Mean/P95/Max为22.583/29.600/119.523ms；17个事件帧Mean/P95/Max为54.600/79.331/79.989ms。33次额外align累计410.380ms，其中唯一确认align为6.012ms，其所在整帧29.702ms；确认的100.502016ms是等待下一扫描的时延，不是CPU耗时。两窗口进程墙钟5.476s/5.632s，地图读取/Gaussian/KDTree一次性初始化57.298/181.190ms；峰值RSS WEAK70080KiB、STRONG134852KiB，为窗口进程累计高水位。

E虽多1次调用，但两条已分歧轨迹的实际Mean/P95受路径和运行波动影响，不能仅以平均时间声称哪个实现更快。R19还含G计算，历史总成本不能直接作为R21门控效率消融。最大帧墙钟超过100ms，不宣称完整10Hz系统通过。

## 编译、测试与执行命令

```bash
cmake -S docs/p62_r21_relative_temporal_admission/tools -B /home/jian/livox_ws/p62_r21_relative_temporal_build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p62_r21_relative_temporal_build -j2
# 在构建目录执行：env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 ctest --output-on-failure
python3 docs/p62_r21_relative_temporal_admission/tools/seal.py before
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r21_relative_temporal_build/run_relative WEAK docs/p62_r19_native_ndt_basin_stress_test/weak/SCANS.csv docs/p62_r19_native_ndt_basin_stress_test/weak/ANCHOR_MATRIX.txt /home/jian/livox_ws/p62_r19_ndt_basin_data/weak_roi.pcd docs/p62_r19_native_ndt_basin_stress_test/weak/map_preflight_01/MAP_BUILD.csv docs/p62_r21_relative_temporal_admission/weak/formal_run_01
env LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_BIND_NOW=1 /home/jian/livox_ws/p62_r21_relative_temporal_build/run_relative STRONG docs/p62_r19_native_ndt_basin_stress_test/strong/SCANS.csv docs/p62_r19_native_ndt_basin_stress_test/strong/ANCHOR_MATRIX.txt /home/jian/livox_ws/p62_r19_ndt_basin_data/strong_roi.pcd docs/p62_r19_native_ndt_basin_stress_test/strong/map_preflight_01/MAP_BUILD.csv docs/p62_r21_relative_temporal_admission/strong/formal_run_01
python3 docs/p62_r21_relative_temporal_admission/tools/seal.py after
python3 docs/p62_r21_relative_temporal_admission/tools/evaluate.py
```

真实运行增加1200s timeout、pipefail、tee封存日志。运行目录有排他创建守卫，以上仅为运行记录，不应覆盖既有目录再执行。CMake Release和必要C++测试通过；Python评价脚本语法检查及封存/时序/预算/状态守恒检查通过。精简只读复核重新计算了Pareto与参考统计、确认次序和历史隔离，未发现结果性错误。详见TESTS_AND_REVIEW.md。源码保留一个空白行尾格式警告，不影响编译、算法与哈希身份；未在实验后改写运行源码。

## 科研判定与唯一NEXT

H1：在这个已观察案例，允许严格相对支配候选跨过0.30绝对门，改变了后续真实因果轨迹，D达到强窗两项工程目标。H2：未建立；E没有优于D，且通过两帧规则的候选仍存在较高参考旋转误差。H3：真实观察到额外一次调用、单帧等待和首帧回退代价。只有一个pending实例，无确认失败样本，不能从1/1确认推出可靠性。

**优先保留D，停止E作为新增复杂机制。** Pareto排序和跨帧确认不是已证明的新颖方法。本轮数据已暴露过参考，Full的Coverage也不是独立证据。未证明跨场景泛化、严格正确盆地或真实LiDAR–IMU闭环收益。

**唯一NEXT：冻结D作为简单准入基线，交科研总控决定是否批准参数冻结后的独立确认与真实LiDAR–IMU因果系统验证。** 不继续当前窗口调阈值或扩展E，不自动执行下一阶段。
