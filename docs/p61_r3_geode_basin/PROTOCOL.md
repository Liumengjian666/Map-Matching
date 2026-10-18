# P61-R3 固定方案：初值分支敏感性

START_SHA=71f12c062c38ff8489124cf8e648269422919776。
只在系统内盘复用R2冻结100对及导出PCD；不访问bag/外盘、GT、IMU或在线定位。
不修改生产源码或R2。相邻扫描Target，不是独立先验图；不运行Coupled精修。

## 预注册

1. 校验R2完整screen输入/输出冻结SHA及归档字节身份。输入冲突立即停止。
2. 全100个Hessian采用R2 chart：p=p0+0.8 eta_t[m]，R=Exp(eta_r[rad])R0。
   Hrr正定且cond<=1e12才用Cholesky两次三角解，求X=solve(Hrr,Hrt)，Heff=Htt-Htr X。
   不显式求逆。独立LU solve复核，另验全块能量和特征方程残差。
   曲率损失报告1-lambda_min(Heff)/lambda_min(Htt)，以及trace损失；不是协方差或旧退化判据。
3. 从100对按实际Source时间均匀最近选20对，含首末，等距冲突取早者。
   不按曲率、Fitness、结果或GT选样。特征向量最大绝对分量固定为正。
4. 每对以R2原始输出为中心，弱/强方向各±0.30/±0.80m平移。
   另4个弱平移伴随eta_r=-solve(Hrr,Hrt) eta_t，eta_t=physical_delta/0.8。
   初值通过现有coupledPoseAtEta chart函数生成，仅是位姿数学助手，不执行Coupled算法。
   旋转>5deg则OUT_OF_BOUNDS，保留且不执行、不裁幅。最多240次新align。
5. 原始nominal只读取，不重跑align。先对20对执行只读目标构建/nominal jet校验
   Source/Target处理后FNV身份、点数、原目标值和Hessian。此为20次额外导数请求。
6. 每个合法初值只align一次；每次额外一个只读native jet记录完整g/H。
   所有初值的Target一致；参数、实际0.8m网格、Source/Target预处理均沿用R2。
   输出目录必须不存在，不覆盖；所有失败保留，不自动重试。

## 聚类及证据标准（新实验前固定）

对每对的有限刚体且收敛输出，使用确定性complete-link聚类；任何簇内所有两两差
必须<=0.10m且<=1deg。每次合并选最大归一化距离最小者，同分按初值ID字典序。
簇至少2个不同物理初值才为重复分支；nominal不计入该数量。
实际初始位姿差<=1e-6m且<=1e-4deg的重复构型只计一次，不能用不同ID凑数。
代表为两两归一化距离和最小的medoid，同分取ID字典序。

候选远分支要求该簇所有成员相对nominal均>0.30m或>3deg。
同时报告不同重复簇之间的最小/最大分离，不把单个离群点当成稳定分支。
竞争性要求该簇所有成员abs(E-E_nominal)<=0.01*max(1,abs(E_nominal))。
E=-native_score_sum/source_count，是原生平均NDT能量，不是Fitness。

局部驻点独立记录：g范数、H是否SPD、scaled_gradient=||g||/max(1,||H||2)、
Cholesky Newton步范数及decrement；strict_stationary仅在有效支持>=0.5、H正定、
||g||<=1e-3且||solve(H,g)||<=1e-3时成立。这是保守研究诊断，非PCL停止准则，
不修改R2严重退化标签。收敛本身不证明局部极小值。

SUPPORTED_COMPETING_BASINS_OBSERVED要求至少两个重复且竞争的簇，
每簇至少两个不同物理初值满足strict_stationary，两簇所有跨簇成员的分离
均>0.30m或>3deg，且至少一簇远离nominal。
不能把非驻点nominal加唯一新极小值称为两个basin。
否则若存在不同结果（相对nominal
>0.30m或>3deg）只标INITIALIZATION_SENSITIVITY_ONLY。
无该分离则NONLOCAL_BASIN_NOT_DEMONSTRATED。输入/核心合同失败则BLOCKED。
该规则不以GT决定任何分支，也不将任一簇叫正确/错误定位。

保留全部初值、OOB、失败和不支持反例。完成固定预算后停止，不扩大搜索。
