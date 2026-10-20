# R7 执行前冻结合同

START_SHA=1be4ad3c0070a0cd41f592498f1ae85795d76c4a。R6本地与本轮远端实际查询均为此提交；R6工作树干净。独立R7工作树，不push。

## 唯一窗口与预注册

完整读取原始bag的LiDAR头/点offset和Livox IMU角速度，不运行ROS节点。所有10905帧得到10806个连续100帧窗口；10800个符合资格，6个拒绝原因逐项保留。R6原始最前100帧窗口排除，不额外排除其他重叠窗口。

按完整点时间跨度的原始gyro模长分段ZOH分布，采用整数ns真实时间权重，逆经验CDF的P90、median降序，最后最早时间。无插值、无bias校正、无积分位姿。起止边界须有IMU包围，IMU严格单调且角速度有限。间隔只记录，不增加未经批准的间隔阈值改变排名。若整包IMU不单调，停止而不排序修补；本次真实整包审计已通过，无此阻塞。

只用历史因果initial的seq/time及有限单位四元数检查资格。历史时间是double秒，沿用R6最多512ns表示容差，当前时间仍为原始整数ns。Source预处理count/FNV仅在排名锁定后读取作身份核验，不参与排名；不取Raw/Used/Fitness/历史误差或R6候选输出。

唯一选择原始10641–10740，seq15376–15475；点时间1786180543409424067–1786180553409787914ns。加权P90=.8071989866894678rad/s，median=.4153393072590976rad/s，最大IMU间隔6.510496ms。全部100个初值来源标签local_imu_rotation_prior。

WINDOW_PRE_REGISTRATION.json SHA256=f864f6926a4fcaf11c4c17e56919657f532dbd2e3968fcd73a541f3a7c92eaaf。诊断位置20/40/60/80已在其中冻结，正式开始后不改文件或选择第二名。

## 数学与调用预算

直接include R6的native_kernel.hpp及contract.hpp；分区、原生PCL NDT、G、seed、触发、去重、准入、选择数学不变。0.8/.08/.001/40/.55；Source .25m、.5–80m、1400cap；Target两次.15m、先resolution后Target，实际Gaussian叶尺寸检查。训练≥300，每块≥10；全部100帧预处理身份和分区资格在任何align前检查。不合格停止，绝不换窗口。

正式每帧FULL一次、SPLIT一次。SPLIT是PROPOSED名义align，不重复。普通帧额外0，真实触发最多2；全程共享20 token。G归一化PSD方向矩阵，不称原生Hessian；ratio<=.01才按最弱特征向量±统一缩放，平移≤.30m、旋转≤3°。候选相对共同预测.8m/5°拒绝，0.10m且1°去重；每块损失和D共同权重算法保持R6，D>.02，对手不因准入失败而消失。β=0，不增加Coupled或IMU评分。

100帧单次执行，前20通过后同实例继续80。保存100个名义Candidate及同一组sources/splits/map/tree；**仅当正式natural_events==0**时，从缓存取20/40/60/80，强制G路径但不改.01，最多各2align、诊断合计8，仍共用20预算。缓存名义不重新align、不重新验证/优化。诊断phase=DIAGNOSTIC_FORCED，独立结果CSV、独立成本，不更改正式选择。无弱方向则NO_WEAK_DIRECTION，不扩大诊断。

三选择器使用同一去重候选池；逐对D另表记录。无对手或弃权时margin=NA，不再记1哨兵。此为记录层修正，不改变select()。nominal/FULL非法先保存FAILED_NATIVE_OUTPUT.csv后停止；恢复非法先记录池/帧再停止。所有失败保留，不自动重试。

## 真实输入限制与保护

与R6一样：raw点云、原生产扫描中点参考，Deskew关闭；没有新IMU积分/EKF。冻结共同initial不会因研究选择改变。原地图与loop2同记录依赖，非独立精度实验；FULL含验证点，只作工程对照。研究实际.8m Gaussian Grid，不宣称历史生产网格数值复现。

run_shadow.py要求干净已提交代码、独占RUN_STARTED和新formal目录；前后核验完整原始bag/map/initial日志、100份点字段、预注册、排名/原始运动清单、研究代码和二进制、PCL、R6冻结原始输出、生产源码及当前HEAD/脏状态。研究子进程局部预加载已核验系统libusb，先0align的argc/loader检查；不改系统或生产环境。

GT=0，EKF=0，ROS节点=0，rosbag play=0，反馈=0；不读取其他数据集，不访问HIKVISION，不执行外部CLI。本轮完成后停止。
