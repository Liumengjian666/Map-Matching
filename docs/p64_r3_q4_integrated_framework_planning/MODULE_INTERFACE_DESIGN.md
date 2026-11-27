# 模块接口合同

## 公共坐标、时钟和数据身份

W 固定地图坐标，B IMU/机体坐标，L 激光雷达坐标。T_AB 将 B 中点变换到 A，T_WL=T_WB T_BL，T_BL 为冻结外参。点云在扫描参考时刻 t_s 去畸变，接口同时携带首末点时间、时间跨度、点级偏移来源、外参与地图版本。ROS 消息保存源时间纳秒。墙钟毫秒只用于预算，不能参与运动时间计算。回放以仿真时钟驱动，处理延迟不改变源时间。输入异常若不能因果地补齐则拒绝本帧地图观测。scan_id 含查询测次和时间戳，point_id 是原生预滤波后的稳定序号；另留原始点到源体素质心的映射。map_id、score_id、candidate_id、config_hash 参与缓存键。缓存中的点号和评分规则必须同版。

位姿使用右扰动 T·Exp(δξ)，δξ=[δρ_m,δθ_rad]。统一分析坐标 q=Sδξ，S=diag(I,ℓI)，ℓ=5 m 是 R3 复用的分析尺度，需随记录输出。H_q=S⁻ᵀH_ξS⁻¹。从 L 到 B 的右切空间转换为 δξ_B=Ad(T_BL)δξ_L，协方差用相同雅可比传播。固定 (t_s,map_id,scan_id) 内所有候选使用相同点号、地图分辨率和分母。Native 原始奖励、Park 对数似然和学习评分分别携带单位、规则与缺失政策。不跨评分单位合并。

## 接口清单

| 模块 | 输入 → 输出 | 规则和必要参数 | 失败状态与预算记录 |
|---|---|---|---|
| 时序/传播 | IMU 包、扫描时标、T_BL、上一提交状态 → Frame(t_s,source_id,cloud_L,imu_span)、Prior(T_WB,P_B,prior_id) | 短时间因果去畸变与传播；样本序号和时标完整；无可信噪声时标记 P 不可用。原始 UKF 或 IKFoM Adapter 支持坐标转换 | BAD_TIME/IMU_GAP/FRAME_MISMATCH；消费 IMU 数、时间差、传播耗时 |
| 原生 NDT | Frame、Prior、Map → Native(T_WL,score,iterations,valid_count,optimizer_flag,H_native,score_id) | 原生 0.10 m 预滤波、1 m NDT 与 DIRECT7 为回归默认；按实验地图冻结 | ZERO_SUPPORT/ZERO_ITER/INVALID_TRANSFORM；实测匹配及评分耗时 |
| 局部可观测性 | 原生局部体素贡献或点面参考 → Weak(U_q,eigenvalues,rank,condition,method_id) | 对称正半定代理，量纲统一；比较 X-ICP 投影和 DCReg Schur 的各自语义；原生完整 Hessian 单列 | UNAVAILABLE/SINGULAR；点/邻域数及方向诊断耗时 |
| 事件调度与候选 | Prior、Native、Weak、历史因果质量和计算预算 → Trigger、CandidateSet | 局部/非局部标志分开。每候选含姿态、来源、独立模式证据、关联数、边界标志、时间、分数及搜索预算；模式类别 SINGLE/FLAT/MULTI/UNKNOWN；固定姿态数与完整拟合数分开 | NO_VALID_MODE/BUDGET_EXHAUSTED；触发理由、峰提取、搜索及拟合耗时 |
| 预测视图 | 固定图、CandidateSet、共享射线 → ViewSet(K) | T_LW 裁剪；视场、射线首交、支持球/局部平面、遮挡和未知状态。原始图和评分图 ID 分开；K 点去重靠索引 | NO_MAP/LOW_COVERAGE；视图点数、查询数及可见性耗时 |
| 结构与共享点选择 | ViewSet、Frame → Evidence 与 Mask | 各点/对记录证据来源、COMMON/DIFFERENT/UNKNOWN、符号/法向、支持数、经验扰动标记和体素分布。未知不能当差异；补齐数与真实支持独立计量；全候选固定同一掩码和权重 | INSUFFICIENT_EVIDENCE；支持占比、空间覆盖、地图密度与构造耗时 |
| 判别 | Mask、点级奖励缓存、CandidateSet → Ranking | 共享分母；缺失关联零贡献；输出全云/选点排名、分差、分区域验证、原生和 MAP 支持、边界状态；保留候选集合一致性 | TIE/INCONSISTENT/UNRESOLVED；评分与验证耗时 |
| 精修 | 选定候选、完整源云、模式邻域、Map → RefineResult | 原始全自由度几何恢复；真正有界优化进入后续阶段。普通拟合加越界否决，明示事后保护。检查优化前后、是否跨峰 | MODE_ESCAPE/DEGENERATE/NOT_RUN；次数、耗时、前后偏移 |
| 决策与融合 | Native、Ranking、RefineResult、Prior → Decision(T_WB,Cov 或 Basis,R,reason,t_s) → State | ACCEPT_NATIVE/ACCEPT_REFINED/PARTIAL/PROPAGATE_ONLY，给至多一条地图观测。投影观测残差 r=U_obsᵀS·Log(T_prior⁻¹T_obs)，R 随同空间传播并由开发数据校准。候选混合协方差、Hessian 逆和似然分差均不能直接当滤波测噪 | STALE/UNCALIBRATED/STATE_MISMATCH；两次发布/同源重复更新计数须为零 |

点数、关联数与除数每次评分独立输出。异步迟到任务如不能在提交前及时完成，则输出同一 source_id 的传播或原生回退，迟到结果只进入诊断。需要后续测量回溯时，按另行验收的延迟更新模块执行。默认不回写已经提交的状态。
