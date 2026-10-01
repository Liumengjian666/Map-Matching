【执行 AI → 科研总控 / 决策 AI】

PAPER-P6-ALG-INTEGRATION-A3G-R1 执行回报

Git:
START_SHA=88831c3906f05589031af8d06c95bc405d9d24d9
END_SHA=见本报告所属最终 analysis commit / 伴随最终执行回传
remote HEAD=见最终执行回传的普通 push 验证结果
worktree=/home/jian/livox_ws/dog_loc_p6_i6b_ws
branch=research/p6-i6d-full-algorithm

Input:
dataset=SuperLoc Corridor01
full catalog=2777; pre-handoff=51; expected terminals=2726
artifact SHA gate=PASS, 全部22个冻结文件及其字节数一致
ledger SHA256=4992c4f56fa49e2b8837f57733b9edc2bde51efb997e6c475a716be35a3cdf05
official params SHA=7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d
REAL_REPLAY_COUNT=0
GT_USED=false

Timeline（全部按sensor stamp排序）:
last LiDAR committed tx=182
stamp=1517157237443522287
selected rank=5
selected NIS=14.25169804323739 / threshold15.086, accepted
U_obs=VALID_GEOMETRIC_GAUSS_NEWTON_PROXY
map support=VALID_INFERRED_FROM_SUCCESSFUL_ADMISSION
NDT converged=1; fitness16.096816828119337; objective170.95783834476933; iterations40
predicted map_T_imu p=[18.84510535,-18.71374009,2.843835062]
NDT converted map_T_imu p=[18.617047220864194,-18.311478189630257,2.3462194538698635]
innovation=0.679298491m /0.402429971rad
raw NDT matrix和完整quaternion见ANALYSIS_FACTS.json / NDT_TRANSITION_AUDIT.csv

first subsequent noncommit tx=183, stamp1517157237544382406
first persistent rejection tx=183（>=20 consecutive terminals）
completed rejection streak=tx183–365,183帧
其中181帧NO_VALID_GEOMETRIC_CORRESPONDENCES、2帧MAP_SUPPORT_INSUFFICIENT
tx366另为premeasurement covariance failure,不是第184帧map-support rejection
first NO_VALID_GEOMETRIC tx=166, stamp1517157235829884363
temporary first loss=tx166–172,7帧，随后恢复commit
first persistent exact NO_VALID_GEOMETRIC tx=188,178帧至365
first >0.5m increment tx=160,0.684130637m
first >1m increment tx=176,1.219916666m
last active-LiDAR count>0 tx=201,count1
first active-lidar-count=0 tx=202
zero-active streak=tx202–366,165 terminals（164完成+1失败）
tx202 QR enforcement302 attempt1消耗tx182 state及最后一条active LiDAR factor
attempt2也成功，final39 nodes/span1.916218996s
历史LiDAR信息仍在prior；active factor=0不是historical information=0
first >2m increment tx=215,2.024439620m
first >5m increment tx=285,5.032843337m
max increment=tx365,10.221731186m
tx366 failure timestamp=1517157256000664307

NDT:
first abnormal evidence=必须区分描述性大innovation与错误定位
earliest >1m NDT innovation=tx83,1.683044257m; 后续commit恢复
first >pi/2 NDT rotation innovation=tx160,1.624361713rad; 当帧commit
first evidence of no effective registration=tx166,converged1/iterations0/objective0
所有188个NO_VALID correspondence帧同时iterations0/objective0
审计窗口这些NDT terminal几乎等于prediction seed，不能当成独立正常定位证据
tx165 innovation3.880847m/0.614061rad；tx166约4.47e-7m/6.25e-9rad
tx183约7.51e-7m/2.01e-8rad
tx188 fitness14.039987→tx20166.044057→tx20271.394847→tx215384.592079
objective0不是精度好；fitness/objective是不同PCL指标，不作GT评分

Geometric support:
全完成前缀=107commit,16NIS reject,188NO_VALID,2不足,1rank reject
tx140–220=22commit,13NIS reject,43NO_VALID,2不足,1rank reject
valid correspondence/raw-neighbor/rejected-covariance具体数量未日志化
NO_VALID status只能确认accepted correspondence=0，不能区分无neighbor或全部被过滤

NIS:
was NIS primary starvation cause=NO,不支持持续NIS starvation
证据=最终183个完成拒绝均preview invalid,NIS_NOT_REACHED
早期NIS rejection有恢复，不能排除其对状态演化的历史影响

Covariance:
P15 health during support collapse=tx140–220全部81/81 AVAILABLE/full column rank
所有314个完成terminal的P15均可用，first unavailable=tx366
tx366 after starvation=YES
tx366 rank498/600,maxPivot6.809639472427348e15,threshold907.22622380221742
triangular solve NOT_EXECUTED;日志residual0不是成功证据
本轮不宣称rank gate正确，也不修threshold/scaling/fallback

Deskew:
81/81 nonempty; input/output count preserved; stamp min=start, max<=end
provenance=WINDOW_OWNED_SE3_DESKEW,所有完成314个terminal同样保持
counts27614–29107,duration0.100775553–0.100839450s
mean displacement0.096630→1.079561m; P95 0.178287→2.093892m
max spike披露:1651.870857→1664.123055m,1680.743838→1693.353809m,max5.983417m at211
DESKEW_ANOMALY_NOT_SUPPORTED仅限时间/数量/provenance错误的证据
基于可能偏离的Window state产生的物理deskew是否正确，CSV无法证明

Hypotheses:
H1 NDT incorrect first=NOT ESTABLISHED
H2 prediction incorrect first=NOT ESTABLISHED
H3 correct NDT incorrectly rejected by support implementation=NOT ESTABLISHED
H4 persistent NIS starvation=CONTRADICTED as primary sustained mechanism
H5 covariance failure before starvation=CONTRADICTED
H6 deskew/timestamp failure=NOT ESTABLISHED; structural anomaly not supported
H7 unique other cause=NOT ESTABLISHED

PRIMARY_ROOT_CAUSE_CLASS=A3G-R1-H / ROOT_CAUSE_NOT_ISOLATED
DIRECT_SUSTAINED_ADMISSION_BLOCK=MAP_SUPPORT_REJECTION
UPSTREAM_PHYSICAL_OR_IMPLEMENTATION_ORIGIN=NOT_ISOLATED
TX366_COVARIANCE_FAILURE=DOWNSTREAM_OF_LIDAR_MEASUREMENT_STARVATION（时间关系）
这不证明starvation单独造成rank failure，也不证明rank rule无bug

Forensic acceptance:
sha gate、last commit、persistent rejection、首次及persistent NO_VALID、NDT/prediction/
U_obs/NIS/P15/deskew时间关系、active-zero streak、增量chronology、tx366位置均完成
A3G_R1_PRE_TX366_ROOT_CAUSE_ISOLATED=PASS（任务标准允许诚实分类H；不是上游根因已隔离）
offline self-tests=7/7 PASS; frozen artifact/output reproduction checks=PASS
production code changed=NO; REAL_REPLAY_COUNT=0; GT_USED=false
A3G_FULL_CORRIDOR_ENGINEERING_GATE=FAIL（保持）
LIDAR_IMU_CORE_ENGINEERING_FREEZE=NO（保持）
READY_FOR_FORMAL_EXPERIMENT=NO

边界/交接:
没有选择production repair。本轮最多定位到“持续support拒绝→无新LiDAR measurement→
active factor被正常QR消费→晚期covariance rank failure”的两级以上时序链。
增长运动早于永久support丢失；不能仅依据tx166先于>1m阈值就选C分类。
若后续获独立授权，证据缺口在tx160–166及174–188的prediction/NDT/geometric-support
边界与correspondence拒绝明细，而不是先修tx366 rank gate。
这是待科研总控判断的调查范围，不是执行AI启动下一阶段或擅自选择算法。

执行AI已STOP。未调参、未修改任何核心、未runner/rosbag/点云/NDT replay、未GT。
