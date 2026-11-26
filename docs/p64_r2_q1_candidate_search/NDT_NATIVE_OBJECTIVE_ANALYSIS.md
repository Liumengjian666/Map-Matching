# 原生 NDT_OMP 目标函数及源码证据

本阶段以 Koide 等开源的正态分布变换多线程实现（NDT_OMP）提交 5495fd9214945afcb4b35d5a1da385e405c52bf9 和本机 PCL（点云库）1.10.0 为准。以下行号指未插桩的上游文件。P63 使用的 HDL Localization 提交为 de2dc7769aa2412876d05ae6161408d3295b6a3c，HDL 的最近邻内点诊断与 NDT 优化器的高斯关联计数为不同指标。 [NDT 声明与目标初始化](https://raw.githubusercontent.com/koide3/ndt_omp/5495fd9214945afcb4b35d5a1da385e405c52bf9/include/pclomp/ndt_omp.h) · [NDT 目标与优化](https://raw.githubusercontent.com/koide3/ndt_omp/5495fd9214945afcb4b35d5a1da385e405c52bf9/include/pclomp/ndt_omp_impl.hpp) · [HDL 状态发布](https://raw.githubusercontent.com/koide3/hdl_localization/de2dc7769aa2412876d05ae6161408d3295b6a3c/apps/hdl_localization_nodelet.cpp)

## 输入、构建与关联

| 步骤 | 文件、函数及原始行号 | 处理 |
|---|---|---|
| 1 源云 | /usr/include/pcl-1.10/pcl/registration/registration.h:189–194，Registration::setInputSource | 标记源云更新并将 ConstPtr 交给 PCLBase。NDT 的 computeDerivatives 用 input_->points.size() 及相同索引访问变换点，因此本实验设定全源云、同点序。 |
| 2 目标云 | ndt_omp.h:122–142、275–283，setInputTarget / setResolution / init；PCL impl/registration.hpp:43–52、56–82 | 保存目标指针，按边长 h 设置目标高斯网格并 filter(true)。PCL 基类另在 initCompute 准备原始目标点最近邻树。改变 h 可能重建高斯地图，实验在 setInputTarget 前设定 h。 |
| 3 分格和均值 | voxel_grid_covariance_omp_impl.hpp:67–104、159–177、218–237、282–297，VoxelGridCovariance::applyFilter | 以坐标下取整得到三轴格坐标，扣除最小格下标并线性化。对格 g 用双精度累加 a=Σy 和 B=Σyyᵀ，再取 μ=a/n。地图点来自各帧封存的预处理目标输入。 |
| 4 协方差 | voxel_grid_covariance_omp.h:103–110、208–215；voxel_grid_covariance_omp_impl.hpp:328–330 | 各格的累加矩阵初值是 I。原码计算 C₀=[(I+B−2aμᵀ)/n+μμᵀ]·(n−1)/n。代数上等价于 (n−1)(I+Σ(y−μ)(y−μ)ᵀ)/n²，和 Park 公式 (2) 的无偏样本协方差不同。应按实际代码理解并保留浮点运算顺序。 |
| 5 规则化 | voxel_grid_covariance_omp.h:208–215；voxel_grid_covariance_omp_impl.hpp:332–364 | 默认至少 6 点。协方差特征分解后，拒绝负的前两特征值或非正最大值。较小特征值下限取 λmax·0.01，必要时按原特征向量重建，再求逆。逆矩阵系数命中 ±∞ 时将点数标成 −1。 |
| 6 查邻域 | ndt_omp_impl.hpp:229–249，computeDerivatives；voxel_grid_covariance_omp_impl.hpp:374–441 | DIRECT7 按中心、+x、−x、+y、−y、+z、−z 的顺序检查，只有在图内且点数达到阈值的格返回。DIRECT1 只查中心，DIRECT26 用 PCL 的周围相邻偏移表。另一分支 KDTREE 按记录的体素质心半径 h 查询，库头文件 477–504 将索引映回叶结点。 |
| 7 固定点变换 | /usr/include/pcl-1.10/pcl/registration/impl/registration.hpp:160–202，Registration::align；ndt_omp_impl.hpp:93–119、825–837 | 复制源输入的排序，初始状态复位。初次评分变换使用传入的矩阵，后续线搜索采用 t、Rx、Ry、Rz 的乘积构造 float32 矩阵。固定姿态检查应调用相同 PCL 变换，避免换一个矩阵重构路径引入边界差。 |

[高斯体素声明](https://raw.githubusercontent.com/koide3/ndt_omp/5495fd9214945afcb4b35d5a1da385e405c52bf9/include/pclomp/voxel_grid_covariance_omp.h) · [高斯体素实现](https://raw.githubusercontent.com/koide3/ndt_omp/5495fd9214945afcb4b35d5a1da385e405c52bf9/include/pclomp/voxel_grid_covariance_omp_impl.hpp)

## 实际固定姿态评分

记变换后源点为 x′，邻域格 g 的均值与逆协方差分别为 μg、Wg，残差 r=x′−μg，马氏距离平方 q=rᵀWgr。异常比率记为 o，边长为 h。构造器和每次完整优化初始化分别在 ndt_omp_impl.hpp:46–75、79–93 计算相同常数。若固定姿态评估走无 align 路径，必须用所设 h、o 更新这些常数，尤其 h 不是默认 1 m 时。

    c1 = 10(1−o), c2 = o/h³
    d3 = −log c2, d1 = −log(c1+c2)−d3
    d2 = −2 log([−log(c1 exp(−0.5)+c2)−d3]/d1)
    e = exp(−d2·q/2)
    s_ig = −d1·e  if  d2·e ∈ [0,1] and finite; otherwise 0
    s_i = sum_g s_ig
    S_native(T) = sum_i s_i
    reported mean = S_native(T)/N

原生内层 q 使用 4 维扩展残差、将 W 转 float32 并扩成 4×4 矩阵。指数及 s_ig 是 float32，点内、点间奖励总和为 float64。computeDerivatives 并行给每点独立槽位，最后按源点顺序串行求和。DIRECT7 中同一点可获多格贡献，每格加一次，邻域数不作除数。缺少任何邻域的点给零奖励，归一化时仍计入全源云点数 N。原生更新的异常值使贡献为零，因此应同时保留邻域关联和正贡献两个诊断量。 [评分循环与贡献核函数](https://raw.githubusercontent.com/koide3/ndt_omp/5495fd9214945afcb4b35d5a1da385e405c52bf9/include/pclomp/ndt_omp_impl.hpp#L180)

### 逐点评分验证路径

固定姿态参考是上游 computeDerivatives 的返回值。逐点接口将相同变换点和邻域的残差、逆协方差交给上游 updateDerivatives 的只读分值路径，输出点序、所查询格的编号/统计量、马氏距离和奖励。两个路径在逐点核函数共享、循环及输出方面可独立核查。将 s_i 累加，再用相同 N 归一化。由伴随诊断的 score-only 调用计算固定姿态奖励时，不需要执行 Newton 步、线搜索、地图更新或状态估计。CPU 编译选项、邻域查找次序及浮点类型应归档。 [computeDerivatives:180–288、updateDerivatives:485–536](https://raw.githubusercontent.com/koide3/ndt_omp/5495fd9214945afcb4b35d5a1da385e405c52bf9/include/pclomp/ndt_omp_impl.hpp#L485)

## 梯度、二阶矩阵及优化终止

以 q=rᵀWr、e=exp(−d₂q/2)、a_k=J_kᵀWr 表示单格，在不跨体素邻域边界时，J_k 为变换点对第 k 个坐标的一阶导数，B_kl 为变换点的二阶导数。该贡献在代码的浮点近似下对应：

    gradient_k = d1·d2·e·a_k
    H_kl = d1·d2·e·[−d2·a_k·a_l + rᵀW B_kl + J_kᵀW J_l]

因此原生二阶矩阵同时含指数曲率项、变换二阶导数项及一阶雅可比乘积项。它是当前格集合内的局部二阶量，体素选择发生离散变化时不表达跨格跳变。角度很小时 computeAngleDerivatives 存在把正弦、余弦近似为 0、1 的分支。首次及线搜索建议步以 float32 局部导数、float64 输出求和计算。若内层线搜索发生重试，末尾的 computeHessian / updateHessian 另以双精度重算 H。 [角度及点导数 290–481、单格更新 485–536、Hessian 重算 541–645](https://raw.githubusercontent.com/koide3/ndt_omp/5495fd9214945afcb4b35d5a1da385e405c52bf9/include/pclomp/ndt_omp_impl.hpp#L290)

采用雅可比奇异值分解（Jacobi SVD）求解 HΔ=−g。对 Δ 归一化，通过 More–Thuente（按充分下降和曲率条件的步长选择法）搜索，其标量函数为 φ(α)=−S_native(p+αd)。因此等价于**最大化 NDT 原生奖励**。初始步长来自 ||Δ|| 并受 step_size 和 transformation_epsilon/2 约束，若非下降且导数不为零则反向，线搜索最多 10 次内层迭代，第一次试步另计。实际外层记录使用严格大于 max_iterations 的条件，或已有至少一次迭代时接受小步长。最终报告不可按理论循环上限反推实际用时。 [Newton 法及外层终止 119–171、线搜索 773–931](https://raw.githubusercontent.com/koide3/ndt_omp/5495fd9214945afcb4b35d5a1da385e405c52bf9/include/pclomp/ndt_omp_impl.hpp#L119)

## 三种报告量和空关联

| 量 | 读取位置 | 定义 |
|---|---|---|
| 目标函数 | ndt_omp_impl.hpp:180–288、485–536 | 原生贡献直接累加，供优化与本实验固定姿态真值无关的一致性比较。越大越好。 |
| Transformation probability（变换后平均奖励） | ndt_omp.h:193–200；ndt_omp_impl.hpp:134–139、170 | 原生目标除全源点数，值可大于 1。它不构成概率质量。只有完整优化输出对象的这个成员在 align 路径中被更新。 |
| Fitness score（对齐后最近邻平方距离均值） | /usr/include/pcl-1.10/pcl/registration/impl/registration.hpp:116–147 | 将源云按最终矩阵变换，求目标原始点最近邻。平均纳入范围内的平方距离，越小越好。空计数返回 double 最大值。 |
| calculateScore（公开备选评分） | ndt_omp_impl.hpp:936–982 | 每格用双精度指数、带 −d3 常量项，每点除以邻域大小，最后除点数。其常量及邻域归一化会改变候选排序，不是逐点合同的参考通道。 |
| HDL 匹配误差和内点比例 | hdl_localization_nodelet.cpp:445–474 | 按实配最大有效距离和最近邻门限计算。默认对齐点模长限 25 m，内点阈值 0.5 m。内点率分母是这一路径的有效点数并至少为 1。零内点使 matching_error 除零，这一诊断与高斯邻域关联独立。 |

当所有点都无高斯关联，S、g、H 都为零。SVD 的解为零向量，原码在 ndt_omp_impl.hpp:134–139 因步为有限的零值立即把 converged_ 设为 true 并返回，此时迭代计数可保持为 0。如果步长为 NaN，则该分支将 converged_ 设为 false。有效驻点也可在第零次更新就退出，因此需保留支持证据、目标值与是否触边。P63 Full Hard 末帧的 0 次迭代、零内点、报告收敛与这一退出机制一致。仅凭该历史记录不能确定具体执行了哪一个关联支持分支，因为 HDL 内点与高斯邻域并不等价。 [历史逐帧证据](../p64_r1_ambiguity_discriminative_planning/historical_audit.json)

### 原始源码指纹

| 文件 | SHA-256 |
|---|---|
| ndt_omp.h | 69b62312c2c7bc5dc605c0dfd267ee0e1f1c12a69d10e673de2213427ddbc556 |
| ndt_omp_impl.hpp | 2911cd75bf11e8674ffe72c411466abf99705204062b5f86cb71d7eb78cdb80d |
| voxel_grid_covariance_omp.h | cc4bdccae332d7b464bc505a3dd28a9c856c8f7be286ba7f5d271ea2e7fb78e4 |
| voxel_grid_covariance_omp_impl.hpp | 4f71cc8e59c9401cac97cd18501253c02dbfa1f84a689b88f9842994fab228d1 |
