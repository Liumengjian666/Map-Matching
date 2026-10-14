# P32-R1 独立场景准入结果

`FINAL_RESULT：DATA_NOT_READY`。

本轮完成有界真实资产检查，但未发现已经通过全部坐标/参考点合同
的独立场景。因此 F/G 短段和全段均未运行；不能据此判断成熟融合
精度成功或失败，也不能宣称泛化。P31已接受的C基线保持不变。

## Git与保护

Repository：https://github.com/Liumengjian666/Map-Matching。
Branch：research/p9-r4-heldout-visual-evidence。
START_SHA / CODE_SHA：2b5d7e6cad433e3830c8f1e03c5ded499b11d341。
END_SHA是本归档所在的最终提交，由Git/聊天回执提供，不伪造自引用SHA。
PUSH_EXECUTED=false；开始时本地/远端HEAD一致。

只新增本P32归档。保护的机器狗/主论文工作区和历史缓存未写入；
未reset/clean/强推，未修改运行源码/参数/地图/传感器文件。

## 实际资产与停止点

优先检查Floor01：三个原始bag、Camera/IMU/LiDAR标定、官方先验PCD、
GT和初始矩阵存在。SSD H1冻结地图SHA重检一致；H1是实际存在的旧非GT
工程初始化，不是Oracle。现有证据仍未闭合运行body、初始化和地图
之间的完整坐标链。官方矩阵语义或有依据的工程坐标链都可以支持准入，
不要求必须由官方说明，也不因H1来自地图一致性选择而直接否定它。
IMU GT lineage已在旧官方同序列审计闭合，不称GT全部不可用。
历史Floor01研究使用不能单独证明当前冻结C有开发泄漏，但严格
未见场景资格也没有建立。没有复用旧Deskew XYZ或PnP伪位姿。

备选Corridor02：原始bag**当前确实存在且可读**，不是旧报告中的
下载阻塞。24,494,797,838 bytes，893.404680秒record duration，
8859点云/178499 IMU/21339图像。相机640×480 bgr8，真实点云有
float32 XYZ/time和ring，标定、原地图、GT、初始YAML均存在。
但GT参考点以及初始化source/target、传感器/body关系、首扫描关联仍未闭合。
首扫描关联可以有依据地建立，不要求初始YAML含字面时间戳字段。

Corridor02 record时间1724621084…与header1645999726…分属不同时间轴；
后者与旧GT时间范围相容。这个时钟区别不是数据不匹配的证据。
未重新校验全bag/地图/GT哈希或全段时间覆盖，未冒充完整数据PASS。

未知地图→GT绝对变换本身**不作为hard gate**：合法参考点成立后
可以使用冻结前缀相对漂移。当前缺的是起始传感器及参考点语义，
不能通过GT首帧、经验选择、默认identity或新的初始化搜索补齐。

官方页面本轮访问：[SuperLoc dataset](https://superodometry.com/superloc.html)。
它公布了数据及初始化配置链接；本轮页面检查没有提供将这些具体
YAML矩阵、起始传感器与GT参考点连起来的语义证据。

## 未执行结果（不是零误差）

|项目|F视觉OFF|G视觉ON|
|---|---|---|
|短段/完整回放|NOT_RUN|NOT_RUN|
|NDT/OOSM/MSCKF数量|N/A|N/A|
|平移/旋转RMSE/P95/Max|N/A|N/A|
|RPE/尾段风险/状态跳变|N/A|N/A|
|Deskew/NDT/MSCKF/视觉前端耗时|NOT_MEASURED|NOT_MEASURED|
|CPU/RSS|NOT_MEASURED|NOT_MEASURED|

总新回放0、新NDT align0、新特征提取0、GT评价0。没有场景配置
或格式适配实施，没有编译定位算法；CSV中指标为空，JSON为null。
没有不存在的pre-GT输出冻结或已选场景数值评价窗口。
Corridor01指标只属于已验收历史基线，未复制到F/G列。

## 科学与工程状态

- 数据合同：FAIL_FRAME_AND_REFERENCE_SEMANTICS_NOT_CLOSED。
- 工程跨场景迁移：NOT_EVALUATED。
- 独立场景视觉收益：NOT_EVALUATED。
- 泛化：NOT_ESTABLISHED。
- 参数/数学修改：无。噪声、NIS、NDT/DCReg/Multi-NDT、Step Limit、
  MSCKF/SKF、Square-Root、Scan-Local、OOSM、velocity_blend均不改。

检查与归档按Git/质量技能保留可验证证据。内部限定独立审查支持
参考点/初始化缺口，并纠正“绝对GT地图变换必需”的过强要求。
用户确认精确命令与限定输入后，已执行一次外部Codex CLI复核：
gpt-6.1-sol/xhigh，退出码0。它支持暂不准入F/G，但指出必须有
字面起始时间戳、必须官方闭合语义这两项要求过强；本报告已采纳修正。
外部意见只基于给定文本，不是新的数据资产验证或定位实验。
输入、原始结果、日志和执行回执见external_review/；不会追加审计或回放。

## 唯一最小NEXT

请科研总控补齐一份可追溯的**Corridor02坐标／初始化说明**，明确
发布GT的传感器参考点，并支持初始变换的source/target、传感器/body
关系和首个运行扫描关联。官方配置、exporter或有依据的工程坐标链
均可作为证据；不要求存在字面时间戳字段。初值不能由本轮GT反推。
无需再次下载现有raw bag；相对漂移不强制要求已知绝对地图→GT变换。
补齐这一项只解决当前已知语义缺口，不自动证明所有准入项已通过。

执行端停止，不继续初始化搜索、地图转换、视觉提取、F/G回放或调参。
待核查DATA_ASSET_INVENTORY、MAP_AND_FRAME_CONTRACT、asset_evidence、
algorithm_freeze、NOT_RUN表格与复现命令，再由科研总控决定下一合同。
