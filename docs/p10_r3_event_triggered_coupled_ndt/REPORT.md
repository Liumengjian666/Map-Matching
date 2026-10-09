# P10-R3: Event-triggered coupled NDT with temporal shadow confirmation

FINAL_RESULT = EVENT_TRIGGERED_SHADOW_ENGINEERING_PASS_ACCURACY_NOT_ESTABLISHED

NEXT = P10_R4_NONORACLE_ADMISSION_VALIDATION

工程硬门槛和开发性能目标通过；不能据此声称定位精度提升或授权候选切换。固定创新仍为 Degeneracy-Aware Coupled Subspace NDT。本轮只改变调度、局部推荐准入和非局部候选的两帧 shadow 确认，不改 PCL NDT、R2 耦合数学、IKFoM 更新、deskew 或搜索范围。

## Git 与执行身份

Branch = research/p9-r4-heldout-visual-evidence。

START_SHA = 489ac45653796617477e156d5b9eadf415d8ccf0。

算法代码提交 = 1e43ce746ef4a525321448ec5a442e32079e4205。

实际执行 CODE_SHA = 7dc7dc3f98389b553a6b829433bf398a7a8797d3（仅追加二进制快照保留执行位的启动修复）。END_SHA 为包含最终归档的后续提交，完整 SHA 在聊天交接中给出，避免归档自引用。

原工作区 .git 为只读，实际提交工作区为 /tmp/dog_loc_paper_r4_ws.Fq21k2；不修改原 Git、历史 untracked 或 stable workspace。持久运行缓存位于 /home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_r3_event_triggered_coupled_ndt/。Git 只含源码、manifest、统计和 CSV 位姿/状态日志，无 raw 点云、地图或二进制。

`git ls-remote origin refs/heads/research/p9-r4-heldout-visual-evidence` 返回 Couldn't connect to server。REMOTE_HEAD = NOT_VERIFIED_THIS_RUN；用户已核实的起点仍为 START_SHA，不能将本地 HEAD 当成远端 HEAD。PUSH_EXECUTED = NO；通过 bundle 交接本地 ancestry，不主动 push。

## 实现

新增 coupled_ndt_temporal.cpp 中 runEventCoupledNdtShadow()，在 nominal 完成后先计算轻量创新，仅 dt>0.12m OR dr>3deg 且无 pending 时搜索。NORMAL 直接返回；无额外 Hessian、value、preview 或 align。保留完整 map-product 二阶 pullback：

```
eta = W*u + S*v
H = J^T H_native J + sum_a g_native[a] K_a
(Hvv + lambda I) delta_v = -(gv + Hvu delta_u)
```

coupled_ndt_shadow.cpp byte-identical 于 R2；8→16 覆盖搜索与最多2次精修不变。新增 local 准入要求不增加相对 IMU prediction 的旋转不一致性。非局部候选只保存为 pending，不直接推荐，score 低于 nominal 也可在预声明 near-quality 范围内保存。

```
T_alt_next_pred = T_alt_prev * inverse(T_pred_prev) * T_pred_next
```

全部是 map_T_lidar。Pending 优先，一帧仅1次 extra align，用当前 source/既有地图；无 jet/preview。value-only 同一 carrier 的 nominal 和 alternate score 都显式计数。最多后续两帧，残差<=0.2m AND<=2deg，成功二次记录 TEMPORALLY_SUPPORTED 后清除。失败、坍缩、超时都清除，禁止同帧重新搜索。任何时间支持仍不作为状态修正或非局部 shadow 推荐。

CurrentFrameNdtRegistration::Impl::shadowBackend() 从 R2 提取共享 callback；eventShadow() 复用同一 NDT/source/map。p10_r2_replay.cpp 增加 opt-in event mode，applyPoseMeasurement() 仍只读取原 nominal。EventLogger 保存调度/传播/确认/完整处理成本；run_event.py 冻结输入、完整范围、源码、binary 和协议后运行 Control/R3；evaluate_event.py 先验证 freeze 和硬门槛，再用旧固定 anchor/extrinsic 做 GT post-hoc。

## 完整真实回放

Floor01 原冻结 P7 raw-timed input 共4127帧，TX1–4127，初始化 stamp=0；IMU/scan/map/params 六个输入 SHA 全部 PASS。原 scan-end propagation、deskew、外参、地图和 NDT0.8/0.08/1e-5/80 不变。只运行一个 Control 与一个 R3，无 ABC 重跑，无 source recovery、oracle、B12、视觉或 Corridor 实验。

Nominal success = 4127/4127。Control/R3 的 registration 与 trajectory 全部 **8254/8254 rows exact parity**（只排除 alignment_ms）。原 R2 前200帧 nominal registration/trajectory **400/400 exact parity**。原始 nominal 输出和状态更新完全未变。

|类型|帧数|完整处理+日志 mean / P95 ms|额外 jet|额外 preview|额外完整 NDT|
|---|---:|---:|---:|---:|---:|
|Control全段|4127|25.9823 / 49.5958|0|0|0|
|R3全段|4127|33.0353 / 79.5996|7505|6960|1002|
|NORMAL|3318|26.8818 / 49.9049|0|0|0|
|SEARCH|545|72.8966 / 129.5874|7505|6960|738|
|PENDING|264|28.0838 / 52.8986|0|0|264|

Raw innovation trigger = 776/4127 =18.8030%；实际完整搜索 =545/4127 =13.2057%；231个 raw trigger 帧优先进行 Pending 确认。3318/4127 =80.3974% 跳过所有额外 backend 工作；轻量调度平均0.00409ms。invalid nominal 没在本次真实序列出现，拒绝分支由合成回归覆盖。

每帧完整 NDT最大3；Pending最大2（1nominal+1extra）；preview最大16。Control4127次，R3共5129次（4127nominal+738search+264confirmation），全实验9256次。terminal value-only score 共1266次，单独计数不伪装成完整 align。非有限推荐0，非局部推荐0，单地图实例，未改变正式 IKFoM。

R3全帧 CPU mean/P95 =33.0089/79.5538ms；shadow增量 CPU mean =7.19423ms。逐帧完整处理包括 cloud IO、传播/deskew、nominal source preprocessing/align、shadow、原 IKFoM update 和 CSV serialization，排除其自身最后一行 cost 的写入。日志 mean0.17887ms。最大单帧191.7939ms，不能把 P95通过解释成最坏实时性保证。

Control/R3进程 wall =107.61482/136.71927s；含输入、地图加载及收尾的进程摊销26.07580/33.12800ms/frame。一次性 IMU/索引读取、静态初始化和地图加载没有分别打点，只能得到混合剩余0.38591/0.38261s，不能声称这些组件各自被实测。这个计时子合同存在明确限制，未通过重新运行来掩盖。Peak RSS =111976/112184KiB，差208KiB只是两进程 high-water 差值代理，不是严格的增量分配测量。

## 同范围 R2 成本对照

TX1–200中199帧NORMAL、1帧SEARCH，无额外align，无推荐。R3 mean/P95 =27.12582/37.36326ms；原R2最终C =138.54728/197.02056ms，描述性下降80.4213%/81.0359%。R3计时还包含日志，而R2旧值不包含日志；相同 nominal和输入已复现，但不同运行时刻不是严格硬件隔离性能试验。R2该段296次完整align/3200preview；R3200次/16preview。未重新跑R2算法，且不能拿完整4127帧平均数与R2前200直接当成同范围对照。

## Pending 和开发诊断片段

搜索返回283个非局部 terminal，其中269个满足预声明基本质量/物理门槛；这是 terminal 数，不是独立 basin 数。保存180次Pending；264次确认 align；第一后续帧支持84次，最终两帧支持57次。180段结局：57支持，71坍缩回nominal，51连续性失败，1匹配质量失败。最大确认帧数2，末尾无未完成Pending。创建Pending的帧中另有7次合法局部推荐，因此 LOCAL_RECOMMENDED事件45与实际局部推荐52不矛盾。

|固定开发窗口|NORMAL / SEARCH / PENDING|R3 mean/P95 ms|Pending创建 / 最终支持|局部推荐|
|---|---|---:|---:|---:|
|TX616±10|3 / 7 / 11|40.2406 / 71.4798|6 / 4|0|
|TX2350±10|0 / 9 / 12|70.8583 / 133.3509|7 / 3|0|
|TX3341±10|0 / 11 / 10|54.1536 / 88.4417|8 / 3|2|

跨窗口Pending可在窗口外创建或完成；窗口支持数不是该窗口创建队列的成功率。TX616=TEMPORALLY_SUPPORTED（2 align），TX2350=PENDING_CREATED（3 align），TX3341=TEMPORALLY_SUPPORTED（2 align）。这三帧仍推荐nominal；均不是新的独立测试。

## 最后加载的 GT 反例

完整输出在 blind_outputs_freeze.json 冻结后，才读取 GT；固定历史 map/GT anchor和标定，不重新拟合。前200帧 nominal GT convention **200/200 parity**。GT仅覆盖4126/4127帧；TX4127超出GT时间范围，gt_coverage.csv 明确保留并标记不可用，绝不外推或删除实验帧。

|4126个 GT可用帧|Nominal raw|Shadow局部推荐|
|---|---:|---:|
|平移RMSE m|0.863610094|0.863611860|
|旋转RMSE deg|4.479268533|4.473453401|
|平移P95 / max m|1.533448187 / 1.784590472|相同|
|旋转P95 / max deg|11.013433986 / 20.800057753|相同|

局部推荐52次：平移25改善、27变差；其余4074相同。仅推荐子集的平移RMSE0.570236185→0.570448400，旋转7.544964435→7.266059982。总体平移RMSE略差；不能宣称定位精度改善。

57个TEMPORALLY_SUPPORTED非局部终端仅做GT诊断：22改善、35变差。**时间连续支持不等于GT正确**，也不等于有安全切换策略。这里已禁止其进入推荐/状态更新。Executed nominal corrected IMU轨迹平移/旋转RMSE =0.869398049m/2.603454144deg，与Control完全相同；这和 raw terminal 误差不是同一量。

前200帧平移/旋转RMSE维持0.043309548m/1.478105754deg。新规则节省正常帧成本，同时放弃了R2该段33次候选推荐；没有把这一取舍包装成精度收益。长段nominal误差风险真实保留，未删后段、未GT重置。

## 验证、失败回执与范围

Release build PASS。P7/P10测试4/4，P9当前研发源码回归41/41，原P10-R1测试2/2。覆盖NORMAL零调用、较低score非局部保留、非交换旋转传播、两帧确认、同帧不重入、过期/zero stamp、坍缩、非有限score、value异常旧score拒绝、非法SE3、local旋转护栏、无效nominal。代码审查按增量实现、Git和独立反证技能执行，审查修复了异常旧score与zero-stamp边界；用户选择仅当前单模型审查，无外部CLI。

保留启动失败 launch_failure_0：copyfile未保留执行位，PermissionError发生在创建子进程前，真实NDT calls=0；copy2修复，不是算法阈值改进。保留首次post-hoc evaluation边界失败的三份中间核查CSV：最后帧GT不覆盖，后续改为明确不可用，不改实验或score。旧P9测试build初跑受MVS/libusb搜索路径影响2项失败，显式系统LD_LIBRARY_PATH后25/25通过；当前研发build41/41通过。所有失败均未通过改数学门限消除。

运行前和输出freeze、CSV/JSON/hash、nominal/source/state parity及git diff检查见 validation_receipt.json / artifact_hashes.json。无需触发本轮最多一次性能改进预算：首次真实算法已达 mean/P95目标，所以没有追加实验。旧R1/R2和全部P9/R7结果保持，stable workspace不修改。

本轮直接创新贡献是：**nominal后真正的零昂贵调用调度 + 原残差耦合候选的预算搜索 + 一个候选、两帧、同图同source的因果时间支持诊断**。证明开发序列正常帧开销明显下降，而不是证明新的定位精度、非局部正确率或生产切换安全性。

唯一下一阶段建议：P10_R4_NONORACLE_ADMISSION_VALIDATION。在现有耦合主线内先验证非GT候选准入和局部推荐风险，维持shadow和原nominal更新，暂不接入非局部状态切换。停止本轮工作，等待科研总控下达合同。
