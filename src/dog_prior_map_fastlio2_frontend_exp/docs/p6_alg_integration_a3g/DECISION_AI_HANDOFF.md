【执行 AI → 科研总控 / 决策 AI】

PAPER-P6-ALG-INTEGRATION-A3G 执行回报

结论：完整工程 soak 未完成。唯一真实 run 首次失败于 tx366 的
pre-measurement square-root covariance rank gate。已立即停止，未重跑、未修复。

Git：
START_SHA=7bcbd5fb2c5c1822e5e0f161f5d59abc356632fd
END_SHA=包含本报告的最终 analysis commit（精确 SHA 及远程状态见执行 AI 最终回传）
branch=research/p6-i6d-full-algorithm
worktree=/home/jian/livox_ws/dog_loc_p6_i6b_ws

production algorithm changed=NO
diagnostic wiring changed=YES，只有轻量健康记录/首次失败保存接线和运行汇总工具。
阈值、noise、NDT、NIS、U_obs/U_nonlocal、rank定义、damping、Window大小、
deskew、初始化、Schur/QR公式和small-step合同均未改。

Input：
dataset=冻结manifest中的SuperLoc Corridor01（没有切换到M3DGR）
full frame count=2777
pre-handoff scans=51
expected post-handoff terminals=2726
7项冻结文件hash=全部与A3F P3-200一致，runner manifest gate保持并通过
mode=FULL_FIXED_LAG_V3_EXPERIMENTAL / ADAPTIVE_SELECTED_NIS
visual=NONE，两个backend=SQUARE_ROOT_QR
initialization_stamp_ns=1517157224188979000，官方外参保持
GT_USED=false，无ATE/RPE/终点误差/精度评分

Run：
REAL_REPLAY_COUNT=1
RETRY_COUNT=0
exit code=1，自然fail-closed退出，非监控强杀
completed=NO
attempted terminals=315
completed terminals=314（到tx365）
completed events=630（316 scan-start + 314 terminal）
window-owned deskews=314
NDT converged=314，总NDT calls=534
U_obs valid=124
covariance requests=315，available=314，unavailable=1
NIS valid=123
LiDAR committed=107
NIS rejected=16
map-support rejected=190（188无有效几何对应，2地图支持不足）
rank rejected=1，other rejected=0
U_nonlocal probes=110，额外正/负NDT calls=220
optimizer ACCEPTED_UPDATE=589
optimizer CONVERGED_WITHOUT_STEP=41
optimizer failures=0
QR removal attempts=592，failures=0，marginalized rank violations=0
post-handoff IKFoM calls=0，visual events/factors=0，legacy fallback=0
max completed nodes=40
max completed span=1.958914906 s
nonfinite states=0，prior A/b nonfinite=0
SO3最大正交缺陷=2.24265e-14，det缺陷=1.79856e-14
timestamp strictly monotonic=YES

First failure：
transaction_id=366
stamp_ns=1517157256000664307
event=LIDAR_SCAN_END
stage=PRE_MEASUREMENT_COVARIANCE_STAGE
reason=producer_square_root_covariance:SQUARE_ROOT_COVARIANCE_RANK_DEFICIENT
rows/columns=600/600
QR numerical rank=498
rank threshold=907.22622380221742
abs R diagonal min=0.028400000820474211
abs R diagonal max=6809639472427348
ratio=4.1705586522557578e-18
triangular solve=NOT_EXECUTED，日志residual=0为初始化值，不是成功解的证据
P15/Pmap=未生成，不存在借用legacy covariance或fallback
failure Window=40 states / recorded span 1.916278839 s
active factors=39 IMU / 0 LiDAR / 0 visual
prior=finite A/b，15×600
window revision=1331，optimized revision=1330
current optimizer=NOT_RUN，而不是optimizer failure
当前tx366的NDT/U_obs/probe/NIS/admission未执行，失败发生在它们之前
此前最后完成事件=tx367 LIDAR_SCAN_START，stamp1517157256000623941
距离失败terminal=40366 ns；真实时间顺序未修改，也没有stamp snapping
本轮只定位到rank gate，不把rank498直接等同真实不可观，不推断根因或修复方案。

Sanity限定：
有限状态不等于物理运动合理。313个完成pose增量的translation mean/max=
2.970121/10.221731 m；rotation mean/max=1.362809/9.102823 deg。
末端tx365 position=[-51.903319,-885.890686,-151.160057] m。
这些较大的有限数值是非GT sanity证据，不是精度误差或定位PASS；没有新增调参。

Freeze/旧结果保持：
与冻结A3F P3-200的298条events逐字段比较，除QR耗时外，差异=0。
旧CSV/hash原路径保持。tx90/tx115/tx155已通过，无已关闭机制复发。
本轮没有把历史短链路PASS提升成全程PASS。

Resources（失败前缀的ENGINEERING RESOURCE OBSERVATION）：
wall=47.98 s，user=41.76 s，system=5.99 s，max RSS=74448 KiB
event mean/P50/P95/max=132.550/70.638/371.835/470.080 ms
NDT per-terminal aggregate mean/P50/P95/max=50.408/1.161/262.179/362.011 ms
optimizer estimate mean/P50/P95/max=13.487/6.941/29.032/32.268 ms
QR removal mean/P50/P95/max=4.697/4.708/5.251/6.533 ms
QR covariance mean/P50/P95/max=38.114/39.248/44.583/48.370 ms
covariance stack max=729×615，temporary estimate max=13767072 bytes
completed-event prior max rows=15，bytes=72120（不含临时扩展）
重型legacy marginalization/covariance shadows=OFF
没有CPU affinity/性能调优，也不把这些前缀数据写成论文benchmark。

Build/test：
Release build=PASS
full Release CTest=39/39 PASS（原37项均保留）
Debug targeted=17/17 PASS
诊断OFF/ON exact states/prior/lifecycle/callback parity=PASS
git diff --check=PASS
编译资源问题通过降低构建并行度解决；启动器元数据preflight失败发生于Popen前。
汇总字段拼写修正只在run结束后离线处理，保存as-run脚本SHA，无第二次真实run。

Artifacts：
docs/p6_alg_integration_a3g/summary.md
FIRST_FAILURE.md / RUN_CONTRACT.md / DIAGNOSTIC_PARITY.md
RESOURCE_OBSERVATION.md / BUILD_AND_CTEST_RESULTS.txt
RUN_FULL_FIRST_FAILURE/含covariance/health CSV、592条QR历史、前20个完成事件、
最后5条preopt、部分轨迹、input/source identity、资源和外部文件SHA索引。
完整外部结果=/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3g_full_corridor
未提交大型点云、bag或重复历史轨迹。
限制：轻量observer没有导出失败A_full/R_full或完整state数组，因此不声称离线数值根因已隔离。

Final：
A3G_FULL_CORRIDOR_ENGINEERING_GATE=FAIL
LIDAR_IMU_CORE_ENGINEERING_FREEZE=NO
READY_FOR_FORMAL_EXPERIMENT=NO

执行AI已停止。没有修复tx366，没有下一阶段/第二次回放/视觉/GT实验。
以上为事实回传，算法与后续阶段裁定由科研总控负责。
