# R11输入与因果特征合同（参考诊断前冻结）

START e25754fd99e81144376e9717c23063c5b575e667，本地R10归档。独立分支research/p63-r11-harmful-ndt-observation。R10结果已知且要求先阅读历史报告，不能宣称研究者不知道其失败；本轮隔离的是特征计算数据通路与后续参考诊断。无远端可见性要求，无定位重跑。

正式分析A raw180–209、B raw299–328、C raw505–534，全部90帧。为每帧的前1/3/5项历史特征，读取原始历史上下文；最早raw174/293/499只作支持，不增加分析样本。所有当前特征只能依赖raw≤k，滚动统计只能依赖raw<k。

## 输入与可用字段

只读R10 `raw/full_01/ONLINE_RESULTS.csv.gz`、`NATIVE_NDT.csv.gz`、`IMU_PREDICT.csv.gz`、`IMU_RECEIVED.csv.gz`、`CALLBACKS.csv.gz`及原seal/manifest、INPUT_FREEZE/BUILD/COMMAND。先核对压缩前后SHA、2375个实际NDT/输出、44267个实际IMU消费、现场Source/Target。R10报告和FULL_RESULTS仅为历史身份/范围证据，绝不作为特征程序输入；不读取FULL_TRAJECTORY或参考误差列。

机器可读允许字段清单保存于ALLOWLIST.json。ONLINE仅提取raw、整数时间、原生质量、累计IMU计数/时间以及原UKF位姿；NATIVE提取真实guess矩阵、终值、质量及Source/Target。实际IMU使用stamp、prev_stamp、scan_stamp、active、dt、last_correction；RECEIVED仅作数量/时间一致性核验。

当前UKF输出发生于融合之后，不能作为融合前风险信息。CAUSAL_FEATURES不包含当前UKF输出；其位姿仅另存ONLINE_POSES.json供封存后解释。前帧UKF输出在当前帧之前已存在，可以使用。不读取或重建协方差，没有NIS、马氏距离、概率置信度。

## 固定公式

T_A_B将B坐标系变换至A。P_k为实际NDT guess，N_k为NDT终值，U_k为UKF输出，s_k为扫描ns。所有范数为欧氏范数，角度为SO(3)主相对旋转角，单位degree，非欧拉角差。

- d_t=||p_N−p_P||；d_R=angle(R_P^T R_N)。另存ΔT=P^-1 N的16个数与地图系平移差，保证可复算。
- dt_k=(s_k−s_(k−1))/1e9；预测运动a_t=||p_Pk−p_U(k−1)||，a_R=angle(R_U(k−1)^T R_Pk)。
- 实际NDT输出跳变量b_t=||p_Nk−p_U(k−1)||，b_R=angle(R_U(k−1)^T R_Nk)。这些与上一已发布状态比较，仍在当前融合前可算。
- 记录d_t/dt_k、d_R/dt_k、a_t/dt_k、a_R/dt_k、b_t/dt_k、b_R/dt_k。它们是按真实扫描间隔归一化的位姿变化率，不是已认证的真实速度/角速度或不确定性统计检验。
- 平移方向cos = dot(p_N−p_P,p_P−p_Uprev)/(两向量范数乘积)。任一为0则缺失；不以角度构造新拒绝门。
- h∈{1,3,5}：从前h帧原NDT修正计算d_t/d_R中位数、最大值及当前/历史中位数比；零分母留空并标记，禁止加一个事后epsilon。历史率采用sum(d_i)/sum(dt_i)，当前率与之比较；记录历史实际跨度s_(k−1)−s_(k−h−1)，不将h当固定秒数。
- h∈{1,3,5}：记录从U_(k−h)至P_k的相对平移/旋转及其真实时间跨度(s_k−s_(k−h))/1e9；不读取未来输出。
- IMU消费数、active数、累计dt和末IMU年龄来自实际原生记录，并与ONLINE逐帧核对。最大真实IMU间隔=max本帧消费sample与其真实上一个已消费sample的时间差；包括跨扫描边界，不能把一批首条间隔漏掉。
- 上一次地图校正时刻由前扫描及原生last_correction核验。另记录首条active传播跨越上一校正时刻的时长=max(0,last_correction−prev_stamp_before)；属于原时间近似诊断，不改算法。
- 原生Gaussian、hasConverged、全点NN fitness、原Inlier、matching_error和迭代数原样保留。Inlier/matching_error限地图原点25m裁剪统计，不是全点Overlap。质量计算在当前配准之后可得；R10实际status回调可能在UKF之后才发出，本轮只证明其信息本身可用，不声称已实现前置接口/门控。

## 缺失与封存

非法核心位姿/缺关键记录则阻塞。没有历史支持则留空并明确history_complete；非有限质量标为缺失，不补数；零运动的比值/cos留空。必须保留90行。算法输入只读取明确字段，列名出现error/reference不会被容许进入特征列。输出CAUSAL_FEATURES、ONLINE_POSES、允许字段、输入哈希与特征源码SHA共同封存，并本地提交，之后才打开参考位姿进行诊断。运行时算法修改/NDT/UKF/ROS/候选/门控均0。

## 封存后评价及安全比较规则

沿用R10冻结Reference.at：原地图坐标，参考SHA不变；位置线性/姿态SLERP，bracket≤150ms、最近端点≤50ms，无对齐或时间拟合。每维分别记录e_pred、e_ndt、e_UKF与Δe，正/负含义不可混用。

A窗固定30帧分布必须完整报告，不以GT剔除A中的异常。附加正常观测描述组在90帧内固定为NDT参考位置≤0.15m且姿态≤2°（沿用R7/R8正常定义，仅离线分组，不是新准入门）。另列“本轮有益修正”：Δe_t<0且Δe_R<0，不加事后幅度阈值，分母和成员逐帧保存。零误差差值或一好一坏单独保留。

各组报告Min/Median/P90/P95/Max；raw309、raw515在A组和正常组中的经验位置用#{x≤x_key}/n描述，不能称p值。比较达到每个关键帧修正幅度的正常/有益例子，仅回答简单“大修正拒绝”的误拒风险，不选取新分类阈值、不计算优化后的检测准确率。固定h全部报告，不事后挑一个最有利历史长度。

最终按用户A/B/C/D分类作有边界的证据判断：区分线索不等于可靠检测，raw515机制不同或正常例子重叠须报告MIXED。只做建议，不执行任何拒绝或轨迹重算。
