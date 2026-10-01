【执行 AI → 科研总控 / 决策 AI】

PAPER-P6-A3G-R4 DIRECT DEBUGGING RESULT — 请复核并裁定下一步授权边界。

START_SHA=45ceeabdc835ebae96ee98a033dd9575a29b94e9
CODE_SHA=d0a5307b6e0b1070a6742b06238c8e019342da86
END_SHA=本文件所在普通 analysis commit，交付时以 git rev-parse HEAD 为准。
Branch=research/p6-i6d-full-algorithm
Workspace=/home/jian/livox_ws/dog_loc_p6_i6b_ws

普通 push 已尝试一次，因 HTTPS Username 凭据缺失失败；没有重试或 force push。
远端仍为 START_SHA。本地提交全部保留，需用户提供正常 GitHub 认证后执行：
git push origin research/p6-i6d-full-algorithm

参考 hdl_localization、Autoware NDT 和 PCL NDT，已 clean-room 实现：

1. 可配置 tracking health，连续失败进入 recovery。
2. 最多两个外围 seed：last committed registration、最近两次可靠 registration
   的有限时间外推；不使用漂移中的 Window velocity 更新锚点。
3. recovery 与正常候选共用原 support/U_obs/U_nonlocal/adaptive R/NIS admission，
   preview 只读，最多提交一个因子，不重置 Window，不注入 prior。
4. 拒绝 zero-iteration/iteration-budget-exhausted 的伪成功；同样检查 probe。
5. 修正 recovery nominal/probe seed 混用。probe 使用自身 recovery seed，
   但创新、P15、NIS 仍来自当前 Window。核心数学定义没有改变。

Release build PASS；44/44 CTest PASS；Debug targeted 24/24 PASS。
原 A1-R1、frozen subspace、alias-safe、small-step、QR marginalization/covariance
回归均保留通过，未放宽 tolerance。src/ 和 include/ 核心源码 diff=0。

实际完成 4 次 R4 prefix 调试，没有完整 Corridor01 run：

- v1 prefix220：tx209 达到连续20次不提交而停止。
- v2 prefix220：169 terminals 完成，127 commits，最长断供6帧。
- v2 prefix400：tx366 covariance rank failure，虽然仍有9个 active LiDAR factors；
  内部最大增量10.14824m，不能称稳定 PASS。未修改 rank gate。
- 最终 v3 prefix220：tx176 达到20次连续不提交而停止；最后 commit=tx156。

最终版本直接证据：

- recovery 48 个额外 nominal NDT 候选，36 个有有效几何 support，35 个
  被 selected NIS 拒绝，只有1个提交。
- tx166 recovery：15 iterations，2742 correspondences，rank3，fitness
  0.6300550851，NIS60.41871214 >11.345。
- tx174 recovery：31 iterations，3009 correspondences，rank3，fitness
  0.06783386965，NIS62.02884909 >11.345。
- 125/125 P15 可用；212/212 QR removal 成功；optimizer failure=0；
  post-handoff IKFoM=0；visual=0；legacy fallback=0；状态 finite/SO3 合法。

这些数据只证明 recovery 能重新获得 support，但与当前 Window prediction 的
selected-NIS admission 无法闭合；没有 GT，因此不宣称 recovered pose 正确，
也不宣称 NIS 本身错误。更改 NIS、弱方向数学、Window 重置或 prior 注入都超出
本轮已授权的外围修复。执行 AI 已停止，没有把略过 gate 当作恢复成功。

请只裁定一个下一实施点：优先检查可复现的 upstream prediction/interface bug，
还是明确批准 tracking-lost 后的 relocalization / state handoff 合同。
若批准后者，请定义 Window 状态/先验/观测 ID 生命周期与 NIS 的语义；不能隐式
覆盖状态，不能调阈值掩盖冲突，也不要先修 tx366 rank gate。

完整简短说明与 SHA inventory：
src/dog_prior_map_fastlio2_frontend_exp/docs/p6_alg_integration_a3g_r4/

外部证据：
/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3g_r4_debug/

GT_USED=false
VISUAL=NONE
Core innovation math changed=NO
CORE_ENGINEERING_SOAK=NOT_RUN
READY_FOR_NEXT_INNOVATION_STAGE=NO
READY_FOR_FORMAL_EXPERIMENT=NO

请按用户安排在汇总后手动发外部 AI 复核，执行 AI 不自行启动下一阶段。
