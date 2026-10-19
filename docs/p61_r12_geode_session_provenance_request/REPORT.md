# P61-R12 会话溯源请求准备完成，未发送

## 结果

完成22项已有资产/材料身份盘点、18项问题清单和正式英文请求草稿。
**请求状态DRAFT_NOT_SENT**；没有联系作者、创建issue、发邮件、读取收件箱或催询。
尚无新会话级独立证据，科学结论保持：

- CLOCK_SYNC_NOT_CERTIFIED
- BIAS_STABILITY_NOT_EVALUABLE
- R10_CONTRADICTION_REMAINS_CONDITIONAL

“未找到充分材料”仅针对固定盘点范围，不代表作者未记录或数据已被证明错误。

## Git与隔离

Repository：https://github.com/Liumengjian666/Map-Matching。
START_SHA：75b88b88f5c1bb2f978b065bcf3ee1391ec07ce3。
Branch：research/p61-r12-geode-session-provenance-request。
Worktree：/home/jian/livox_ws/p61_r12_geode_session_provenance_ws。

开始时R11本地和ls-remote实查均为该75b88b88f提交，工作树干净；当前远端确有R11分支。
这与此前回执时点不同；未修改旧回执，未推断由谁何时push，也未重写或更新R11分支。
R10基线a0275decc7862030c3581038ab87d09efe5aaa4e保留。
本轮只新增R12文档及身份清单，本地归档，不push；最终SHA见Git最终回执。

## 会话绑定及盘点边界

原始文件4500527001B：
/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag
SHA256：162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091。

R8本地派生文件1416133574B：
/home/jian/livox_ws/p61_r8_geode_runs/input/full_return.bag
SHA256：a138b5ceb0e8ee4a2e5203e70b3943596811ff15d7a45495909934d3a1494331。

两份bag只只读计算SHA/大小，没有解析或重放消息。另20份材料均来自冻结Git基线。
existing_evidence.csv逐项列绝对路径、大小、SHA、来源、支持内容、绑定级别和局限。
asset_gate_before/after.json记录本轮22项前后核验；不冒称重跑R11全部328项审计。
原始文件摘要是本地文件指纹，R2明确没有官方checksum；请求不冒称它由作者签发。
R8派生文件作者未必拥有，请求明确它只是本地处理链，不要求作者对其字节负责。

## 已有与缺失

已有：gamma设备说明、归档参考Livox驱动、作者算法配置、本地数据适配源码、
冻结bias及R11时间/bias审计。一般说明不能认证采集时实际固件/驱动/同步配置。
本轮未搜索最新驱动，也未混用其他数据集同步/校准记录。

未闭合：发布文件到采集会话身份、采集驱动和时间转换链、实际物理同步架构/配置/
独立误差记录，以及会话设备独立bias校准/静止/温度/稳定性证据。
request_checklist.csv把S1、A1–A6、B1–B6、C1–C5共18项与已有证据ID对应。
missing_evidence.csv记录范围限定缺口；未知指标为NA_WITH_REASON_NOT_ZERO，
没有虚构固件、时间误差、bias不确定度、作者答复或静止证据。

## 用户可审核的请求

REQUEST_DRAFT.md是可发送英文正文；明确目标文件SHA、整数ROS时间窗口、外置Xsens
和本地派生文件区别，逐项询问已有材料。作者可以回答未记录、未知、不适用或受限。
不要求重采集、重新标定、拟合时差/bias或生成不存在的日志。
历史README Contact仅建议官方仓库反馈；没有验证当前收件人或虚构个人邮箱。
用户需先决定是否发送及选择渠道。

## 边界及验收

GT reads=0；ROS localization nodes=0；rosbag play=0；new NDT align=0；EKF runs=0；
new trajectory estimates=0；clock-offset search=0；gyro-bias fit=0；production code changes=0。
未访问HIKVISION，没有修改地图、初始化、外参、配置及R10/R11结果。
没有网络数据搜索、外部CLI或新依赖；网络仅查询科研仓库远端ref。
前后SHA、清单/草稿问题ID一致性及范围检查见analysis_manifest.json和checksums.sha256。

## 唯一NEXT

用户审核正式请求草稿，决定是否及如何发送。未经明确授权不发送。
若收到材料，先验证会话绑定与独立依据，再交科研总控决定是否更新对应结论。
本轮不启动后续分析或定位，完成即停止。
