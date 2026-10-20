# P62-R6 — 真实机器狗 Shadow 原型与 NCLT 预筛

日期：2026-10-20。正式结果：**REAL_SHADOW_COMPLETED_WITHOUT_DISTINCT_CANDIDATES**；并存 **HOLDOUT_DISCRIMINATION_NOT_DEMONSTRATED**。B线：**NCLT_PREFLIGHT_BLOCKED**。创新性仍为 **NOT_ESTABLISHED**。

真实数据完成100帧，前20帧Smoke通过后同一实例继续80帧。200次真实原生PCL NDT，100次FULL与100次SPLIT；恢复触发0帧、额外align 0/20、替代候选0。没有为了制造事件而重新挑选窗口。它证明名义路径与离线记录可运行，**没有证明真实候选恢复或结构留出判别有效**。

## 1. Git 与实际运行身份

- 工作区：`/home/jian/livox_ws/p62_r6_dog_shadow_ws`。
- 分支：`research/p62-r6-dog-shadow-prototype`。
- START_SHA：`c1769471f291932a749fc44705d8ddc30108cf31`，R5本地和实际远端查询相同，未修改历史。
- C++原型冻结提交：`4592128cf21c0c22d51c6a88fdb9a10ec0d64497`。
- 实际成功运行CODE_SHA：`90954199d1f2e06128c6eaafca91bbc6a7c98979`。后者只增加子进程动态库守卫及失败记录，没有改变优化数学。
- 实际二进制SHA256：`6044fc23f931430245526da606c762ebb939e4501f3436235b9692dc962f3e85`。
- END_SHA以最终归档提交的`git log -1 --format=%H`及交接回执为准；一个提交不能在自身内容中嵌入自身摘要。
- 运行前实际查询R6远端分支不存在；结束再查询，见`GIT_HANDOFF.json`。PUSH_EXECUTED=false。

原`dog_loc_paper_ws`与本研究工作树不是同一Git公共目录；不要在不含本分支的仓库盲目push。本轮不复制分支或修改该工作区。

## 2. 真实资产和初值

完整路径、每帧整数时间及SHA见`REAL_INPUT_MANIFEST.json`。

| 资产 | 字节数 | SHA256 |
|---|---:|---|
| loop2_raw.bag | 19677401800 | d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae |
| loop2 FAST-LIVO2 PCD | 7346648 | 016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca |
| 已验收运行的因果initial_guess记录 | 20638557 | e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a |

固定选择bag最前100个原始LiDAR消息：序号4736–4835；原始点时间从1786179479409757376至1786179489409696951ns，跨度9.999939575s。首20帧不是从其他窗口挑选。所有Source均1400点，与历史生产预处理的点数及FNV逐帧100/100一致，且在任何真实align之前全部核验。

CustomMsg是`/livox/lidar`，IMU是`/livox/imu`，两者frame_id=livox_frame。读取1985条真实IMU，仅保存时间/输入身份，不运行新积分或EKF。真实点时间为header_ns+原始uint32 offset_time_ns；不以timebase替换ROS时间。稳定算法原配置Deskew关闭、参考时刻为扫描中点，本轮如实沿用raw Source；不是scan-end Deskew验收。

共同初值取已验收记录的当前帧匹配前initial_guess，而不是旧Raw/Used结果或GT；98帧记录local_imu_rotation_prior，前两帧采用其原有无IMU启动/历史预测状态。第一帧无IMU起点包围，不宣称IMU成功积分。所有研究候选不反馈下一帧，共同初值保持冻结；这不是重新运行闭环定位。

地图459154点，研究程序两次0.15m Target体素后仍459154点。该地图与loop2同记录关联，是工程自地图依赖，不是独立跨会话地图证据。全部新输出在系统内盘，不访问HIKVISION。

## 3. 实现与真实优化器

`tools/native_kernel.hpp`直接调用本机PCL 1.10 `NormalDistributionsTransform::align`，不重定义优化器。原生头文件/实现SHA随`run_freeze.json`冻结，合成封装与原生PCL位姿和得分逐数值一致。

参数：resolution=.8m，step=.08，epsilon=.001，max_iterations=40，outlier=.55；Source .25m、.5–80m、cap1400；Target两次.15m。先resolution后Target，逐次align检查实际target_cells_叶尺寸为.8f。

生产稳定源码loadMap为Target先于resolution的顺序，可能保留默认1m实际网格；本研究按R1显式使用真实.8m网格。**没有修改生产代码，也不宣称CONTROL_FULL与历史生产运行数值等价**。所有新候选和对照共享同一个冻结研究Target。

固定16扇区在优化前拆分，Training 670–728点，8个验证块各12–231点，100/100均可评价。普通帧PROPOSED只运行一次SPLIT名义NDT。FULL是另一次工程基线，成本没有藏入免费对照。训练/验证空间相关，不能称IID。

G是训练点的归一化PSD方向矩阵，不是原生Hessian；本次无触发，真实G未计算，谱、弱方向和真实种子行为不可评价。合成有限差分检查不能替代这些真实结果。实现保留固定±种子、2次/触发帧、20次/窗口及物理去重/弃权合同，不启用Coupled。

## 4. 实际100帧结果

| 项目 | CONTROL_FULL | CONTROL_SPLIT / PROPOSED名义 |
|---|---:|---:|
| 原生align | 100 | 100 |
| finite / converged / 接纳门通过 | 100 / 100 / 100 | 100 / 100 / 100 |
| 迭代次数 | 全部1 | 94帧1，6帧2 |
| 验证点Overlap@.5m均值 | .741479720239 | .741295345245 |
| 8块平均损失的均值 | .167166351499 | .168581955759 |
| 相对共同初值最大平移修正 | .027178245m | .046488324m |
| 相对共同初值最大旋转修正 | .561467995° | 2.496712474° |

FULL训练使用了验证点，其验证块指标只是工程诊断，不是held-out证据。FULL与SPLIT优化Source不同；原生平均得分/fitness不能跨Source直接作为方法优劣。两者最大位姿差.022180848m和2.220284513°，不能算同Source候选分支。

SPLIT每帧8块损失中位数最大.047315783576，低于冻结触发.25；原生均收敛且未达40次。因而真实触发率0/100，没有计算G/额外种子，未用预算20/20。每帧候选池仅T0，物理不同替代分支0。

## 5. 同候选池三种选择器

100帧均核验三种选择器使用同一个池`{T0}`：

- Native score：100次保留T0。
- Held-out mean：100次保留T0。
- Robust paired：100次弃权**恢复**，不虚构替代结果。

这里弃权意味着没有替代候选可被证据支持；普通帧名义结果仍已计算，不代表100次定位丢帧或融合拒绝。本轮根本不反馈EKF。无真实多候选，无法比较排序器的实际分支区分能力，也无法证明稳健门比均值或原生分数更好。

8块共同最坏权重D使用4个最小成对差的均值；C++合成测试以70个LP顶点穷举独立验证。Python也提供该oracle，但本窗口只有单候选，其真实成对比较和LP顶点计算均为0，未实际调用该oracle。缓存单候选选择与预算复核通过，不等于真实多候选检验通过。合成正/负/并列/无资格对手反例已测试，真实区分证据仍缺失。

原始`SELECTOR_COMPARISON.csv`中Native/Mean的单候选`min_pairwise_advantage=1`是C++初始化哨兵，不是测得的优势；没有对手时该指标无定义，应读作NOT_AVAILABLE。原始CSV保持不变，不能将这个1用于判别强度统计。Robust弃权行的nan同样不是零残差。

## 6. 成本与资源

单位ms；P95为线性type-7经验分位数，n=100。

| 测量 | Mean | P95 | Max |
|---|---:|---:|---:|
| Source预处理 | 1.401208 | 2.956765 | 4.192536 |
| FULL原生align | 6.271914 | 6.240390 | 72.028039 |
| SPLIT原生align | 3.082740 | 4.409668 | 7.156484 |
| 全部候选验证（含FULL诊断） | 1.849438 | 1.970207 | 3.952140 |
| 三选择器 | .001384 | .001488 | .002863 |
| 每帧计算（两基线及评分，不含预加载预处理） | 13.896561 | 16.387137 | 79.440285 |

额外align及真实G耗时均为0（未执行，不是它们没有计算成本）。地图加载/Grid/KDtree148.769356ms。实际原生计算子进程墙钟1.768814685s，不含Python前后完整SHA检查。Peak RSS=109048KiB=106.4921875MiB，来源Linux getrusage(RUSAGE_SELF).ru_maxrss，包含地图和预加载100帧，不是纯PROPOSED独立进程，也不是总系统物理内存。没有监控器RSS简单相加。

`align_ms`只测原生align，不包括之后getFitnessScore等调用；其开销包含在每帧总计算中。预加载预处理另外计时。不能用此次批处理、近静止小窗口、0触发结果宣称持续10Hz或100MiB目标通过，更没有EKF/Deskew耗时结果。

## 7. 真实失败、测试与保护

第一次启动在C++ main前因/opt/MVS旧libusb缺符号失败，exit127、真实align=0。回执与日志完整保留，见`LOADER_FAILURE.md`，没有覆盖。一次有根因的工程修正仅在子进程预加载已核验系统libusb；无参数argc测试先验证动态加载，0align。新attempt02执行上述100帧。

C++程序18类合同断言通过，包含2次合成原生align（独立于200次真实预算）。CRLF、预算/弃权、非有限、共同权重、G有限差分、真实PCL封装一致性均在真实执行前检查；非NDEBUG assert，使用运行时require。独立Python从冻结CSV复算100帧输入、预算、触发及同池选择，PASS。

全部大输入、100帧原始字段SHA、17个受保护生产源码/配置文件、稳定HEAD和原有脏工作树状态，计算前后均相同。R1三份冻结合同SHA也通过。稳定工作区未编译、未启动、未修改。GT reads=0，ROS节点=0，rosbag play=0，EKF=0，反馈=0，外部CLI=0。未读取参考误差挑帧。

## 8. B线及唯一NEXT

NCLT官方元数据预筛少于30分钟后已停止。固定两会话空间重叠与独立RTK质量/时间/杆臂评价未认证，约11GB压缩包加20GiB预留已超过当时可用空间；没有下载大型文件或建图。详见`NCLT_PREFLIGHT.md`。这是预筛阻塞，不是证明NCLT无法使用。

**唯一NEXT（仅建议，不执行）**：请总控批准一个按原始运动/采集记录预先选择、独立于本原型候选结果和GT的真实移动/风险窗口；保持此代码、扇区、阈值和20次额外预算不变，检验是否自然出现触发、物理不同候选及稳健证据的真实区分。没有这些事件仍应报告未建立，不放宽门限、不搜索其他方向。不进入正式EKF反馈或精度宣称。

复现、字段语义和限制分别见`reproduction_commands.md`、`IMPLEMENTATION_CONTRACT.md`和`EXPERIMENT_LIMITATIONS.md`。本轮到此停止，不启动下一窗口。
