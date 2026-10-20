# P62-R6 — 执行前冻结合同

START_SHA=c1769471f291932a749fc44705d8ddc30108cf31；独立分支research/p62-r6-dog-shadow-prototype。
R5本地与本轮实际远端查询相同，工作树干净。

## 输入及时间选择（看结果前）

仅使用loop2实际bag的**前100个LiDAR消息，原始顺序，不按匹配结果选择**。前20为Smoke，只有通过才继续相同窗口后80，不重启或重跑前20。
共同初值读取已验收final_delivery_run_A_20260922/ndt_determinism.csv中同序号/同header的initial_guess，非GT、非参考轨迹。仅取初值/时间/Source哈希列，不取历史Raw候选作为本轮优化输出。
使用真实原始XYZ及offset_time_ns；稳定配置deskew_enable=false、scan_reference_time=mid，不凭空改为scan-end或开启Deskew。记录整数点时间，冻结历史共同初值对应中点语义；这是raw几何Shadow，不是精确运动去畸变/融合实验，区别于R1一般scan-end表述，不能宣称时间参考与历史GEODE链相同。
第一帧可没有IMU前置包围，但其初值是稳定算法的非GT启动状态，不伪称成功IMU积分；没有Deskew因而不外推点运动。
MID360消息IMU/LiDAR都使用livox_frame。生产NDT直接在L系匹配、没有本轮需要猜测的I→L旋转；不额外启用gyro验证证据，β=0。

## 真实NDT与生产差异边界

只在研究离线程序调用本机PCL1.10原生NormalDistributionsTransform，不修改优化器。参数保持0.8m/.08/.001/40/outlier .55；Source0.25m、0.5–80m、cap1400；Target两次0.15m体素。
本机生产loadMap在setInputTarget后才setResolution；PCL在未有Source时可能留下默认1m真实Gaussian Grid。**Shadow按R1预注册先设0.8再Target并检查真实target_cells_；这不是生产逐数值复现**，不修改稳定生产代码/参数，不将CONTROL_FULL冒称历史正式定位器的等价回放。Full和Split及恢复候选全使用同一个0.8m Grid。
生产来源实际HEAD=41999ea700c66c4cadf0eca9e0c5d73caa2783fd且有用户未提交内容，以实际文件SHA为准，不按HEAD冒充全部来源；只读核查，不编译/启动该工作区。

## 拆分、方向、预算与评分

严格复用R1：scanner方位[-pi,pi)的16扇区；偶数训练，奇数为8验证块。训练≥300、每块≥10；不满足NOT_EVALUABLE。分割在名义优化、方向、种子及排名之前。空间块不称IID。
G/N_train=Σa_ic J^T invCov J，只用训练点及固定Target，J=[0.8I,-skew(Rx)]；a_ic采用原生正得分核权重(-gauss_d1)*gauss_d2*exp(-gauss_d2*q/2)，非负。G只定义方向，不是原生Hessian/协方差。lambda_max≤1e-12/非有限为不可用；lambda_min/lambda_max≤.01才生成方向。
训练一次align产生T0；事件=不收敛/迭代≥40/验证块loss中位数>.25。±弱方向种子按同一因子缩放到平移≤.30m、旋转≤3°，各最多1次align。总额外token20，不补发。原生align异常立即保留并停止，不自动重试。
所有输出候选相对共同初值超过.8m或5°标为不可接纳，不截断。SE3/原生有限得分/收敛有效性及overlap≥.50共同记录。最终位姿≤.10m且≤1°去重，保留全部原始重复记录。
验证块loss=mean(min(NN_sq/.25,1))；共同权重D=min_w Σw(loss_k-loss_j)，8块、cap.25，即4个最小成对差的均值。独立穷举70个4块子集检验，不能用两个稳健分数相减。
所有有限、去重后的池成员都作为D对手（含T0，即使它不满足接纳条件）；候选只有自身合法且overlap过门才可被选择。Native/mean规则也仅从这些可接纳候选排名，并用同一池；robust只接受替代候选对所有其他池成员D>.02，不能唯一胜出则弃权，不用T0伪装恢复。
每帧CONTROL_FULL独立一次align，CONTROL_SPLIT即PROPOSED名义一次align，不重复跑；三种选择器直接消费同一个缓存候选池。FULL输出不进入池、不反馈下一帧，选中候选亦不反馈。

## 资格、界限与B线

真实数据未冻结/时间合同不合法立即停止真实计算。任何模型数值非法停止并保留。原map同loop2记录的工程关联需披露，不是独立测试GT地图。
无GT、无EKF、无ROS节点、无定位反馈；原生分数/NN不是绝对误差；普通帧额外align0、全段≤20，Full成本单列。
B线仅官方页面/PDF及已有清单，固定2012-04-29/2013-01-10，最多30分钟，不下载大型数据，不阻塞A。
