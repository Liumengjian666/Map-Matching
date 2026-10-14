# P32-R2：Corridor02初始化适配结果

`FINAL_RESULT = CORRIDOR02_INITIALIZATION_NOT_READY`。
完成有界传感器解码与标定计算、检查程序编译、5项定向测试和**唯一一次**
非GT首扫描几何检查。该种子不能支持工程初始化，按合同停止F/G。
这不是成熟融合算法的定位精度失败；没有运行定位或读取GT位姿。

## Git及实际修改

START_SHA：9c0399b7c7370a191e1ede127c611b216cf9639d。
CODE_SHA：48569e92d075c5f14598761ad2e349cce88ee167。
END_SHA以本归档所在Git提交及聊天回执为准，避免伪造自引用SHA。
沿用research/p9-r4-heldout-visual-evidence，只写P10研究工作区。
新增有界适配工具和本归档；NDT/EKF/MSCKF/Deskew运行源码、参数、
标定原文件、历史缓存与受保护工作区均不改。未自动push。

## Corridor01真实约定与本轮边界

已读真实initializer/normalizer：官方OpenCV矩阵作为数值搜索种子，
历史first5s ICP与24朝向候选得到firstscan的T_map_lidar；normalizer
对地图施加该估计的逆，正式NDT首帧identity在归一化地图内使用。
因此不能机械把world_darpa直接作为已认证的T_map_lidar。

本轮不运行旧24候选、不重建子地图，不试逆矩阵或4组注释矩阵。
只读取Corridor02生效R/t，投影舍入R到proper SO(3)，冻结唯一数值
工程种子，再执行一次原生NDT几何检查。种子不被宣称为官方语义。
这一限制不能排除其他合法坐标转换；失败后没有扩展搜索。

## 真实传感器适配

原bag当前可读，历史元数据8859点云/178499IMU/21339图像，约893秒。
本轮有限读取最初5秒：50扫描、968IMU、15图像。
PointCloud2按字段offset、字节序和row padding解码float32XYZ/time。
前缀抽样点time范围0–0.100839450955s，与约0.10086s扫描周期一致；
在本raw几何探针中按header加seconds offset解释时间，不使用2024年的record时钟。
范围和cadence支持这一工程解释，但不能单独证明驱动的header首/末约定、
点时间原点或跨传感器同步；这些未被升级为正式运行时间合同PASS。
首两帧缺少IMU头覆盖，按非GT覆盖规则选第3帧作为唯一检查扫描。
scan_ns=1645999726479818106；end_ns=1645999726580657319；28089原始点。
第3帧自己的max offset=0.10083921253681183s，四舍五入到100839213ns，
与end-start一致。上述0.100839450955s是50帧前缀最大值，不是第3帧值。
Camera延迟4.423431105s，640×480 bgr8；不伪造前几秒视觉观测。

RC1 MEI内参与Camera/IMU、LiDAR/IMU外参读取成功；没有使用RC2外参，
也没有导入标定文件中的噪声值覆盖P31冻结噪声。完整链见FRAME_AND_INIT。

## 唯一几何检查：真实失败

|项目|实际值|
|---|---:|
|原地图点数|5223422|
|目标点数（两次0.15m生产helper）|3671146|
|Source点数（0.25m，cap1400）|1400|
|原生NDT完整align|1|
|hasConverged / iterations|true / 0|
|Fitness|87002.227399553565|
|初始及最终NN median|293.606414795m|
|初始及最终NN P90|305.379516602m|
|<=0.5m覆盖率|0|
|种子平移修正|5.211876238e-7m|
|种子旋转修正|6.971353950e-7deg|
|单次align耗时|673.238949ms|
|检查程序wall time|3.053293s|
|检查程序Peak RSS|736904KiB（约719.633MiB）|

hasConverged不是初始化正确性证据；此处0迭代、巨大NN距离且零覆盖
明确不能通过预注册工程检查。结果和种子差仅在float舍入量级。
检查使用**未去畸变的原始扫描**；不声称这是运行时Deskew Source、
P19恢复或合法IMU运动轨迹验证。未补偿限制不能被忽略，但也不能
把这些结果升级为可信地图初值或直接宣布NDT精度回归。

原地图SHA重新核实：
6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228。
检查二进制SHA：
33149fc350f2605cef931789402e5b6597792a65488431af2e4136ef08e6997c。
raw bag全文件SHA未重新计算：已在初始化gate停止，没有冒充完整输入PASS。

## 编译、测试、复核

初次链接因MVS路径的旧libusb缺少libusb_set_option失败；仅检查工具
显式链接系统libusb并使用单次LD_PRELOAD，随后编译成功。未改系统库。
5项数据接口测试通过：proper-SO3、非法变换拒绝、float字段与padding/
大小端、错误字段拒绝、遍历上限。点文件在唯一align之前另核对SHA及
准确28089×12字节，种子准确128字节。没有为失败结果重复align。

定向只读复核的两项输入边界问题已修正：缺失话题的遍历上限与
截断Source拒绝。实际有限探针用过的pre-guard源码快照单独保留；
其50/968/15条真实结果并非在修复后伪造重新运行。
外部复核状态另见INDEPENDENT_REVIEW.md，不将意见当成实测。
用户确认精确命令与输入后，实际执行一次Codex CLI gpt-6.1-sol/xhigh
只读复核，退出码0。它支持停止并要求限定C01复用、扫描身份和时间
语义；本报告已采纳。复核未新增数据检查、候选或回放。

## 未执行内容及正式状态

F/G短段与完整均NOT_RUN；NDT/OOSM/MSCKF运行次数、精度、RPE、
尾段风险、定位资源均为N/A/null，不是零误差或性能PASS。
没有新视觉轨迹提取，没有GT评价，没有平方根状态或运行Deskew测试。
没有可准入的runtime YAML，未对失败结果归一化地图或填入identity。
保留历史metadata YAML不动；不把不合法初值包装成可运行配置。

ENGINEERING_INIT：FAIL_SOLE_SEED_GEOMETRY。
CROSS_SCENE_RUNTIME：NOT_RUN。
VISUAL_BENEFIT / ACCURACY / GENERALIZATION：NOT_EVALUATED。
GT_REFERENCE：UNRESOLVED，本轮没有用该缺口拒绝本来合法的工程短段。
实际停止原因是地图相对工程初值没有建立。

## 唯一NEXT

补齐一个可追溯、非GT的第3帧T_PCD_L3，参考时刻为header
1645999726479818106ns，或将发布数值种子的实际坐标定义接到该PCD
及此LiDAR扫描的转换链。若只提供第1帧位姿，还需合法的1→3时间传播，
不能把第1帧位姿直接挪给第3帧。此处PCD坐标由实际
文件XYZ定义，不把未知world_darpa语义先验当作某个已知传感器帧。
不请求重下载bag、不执行逆矩阵或全局搜索、不进入P33。
执行端停止，等待科研总控核查源码/归档后决定下一合同。
