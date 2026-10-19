# P62-R4 — Exp18 G0 只读元数据审计

正式主结果：**G0_BLOCKED_COORDINATE_CONTRACT**。
并存：**G0_BLOCKED_CALIBRATION_CONTRACT**、**G0_BLOCKED_TIME_CONTRACT**。
初始化另保持 **INITIALIZATION_NOT_ESTABLISHED**。

两个大文件已完整解析，但坐标、标定与完整融合输入时间合同未闭合，不能宣布 G0_METADATA_PASS，也不能进入定位或算法实现。

## 身份与执行版本

- 仓库：<https://github.com/Liumengjian666/Map-Matching>
- 分支：`research/p62-r4-exp18-g0-metadata-audit`
- 工作区：`/home/jian/livox_ws/p62_r4_exp18_g0_metadata_audit_ws`
- START_SHA：`1b53e7a4164c1e2871c077a4a6f1362e8c228a23`
- 实际完整 bag/E57 解析器 CODE_SHA：`954c49741f7c20b31a2dcb2459825bbb5ed70e97`
- 派生表程序 summarize_metadata.py 在工作树状态执行，实际源码 SHA256：`96fd708a37231d289bea558ae3bd1e3e9a1c88f0650d36763f5a783dc796877c`；它随后归档，不冒称执行时已有该归档 commit。
- R3 本地与实际查询远端均为 START_SHA，工作树干净；新 R4 基线一致。R3 历史及稳定工作区未改。
- 不执行 push。最终 END_SHA 在本地归档提交和执行回执中记录，不伪造自引用 SHA。

## 分项门

| 项目 | 实际结果 | 限制 |
| --- | --- | --- |
| 固定5输入身份 | PASS，开始/结束完整 SHA 一致 | 文件身份不是物理合同 |
| bag 完整解析 | PASS，66633消息 | 不是定位回放 |
| E57 全点解码 | PASS，285455240点 | 不等于地图与GT同系 |
| 坐标合同 | NOT_ESTABLISHED，主阻塞 | T_M_W缺失；GT I/base语义未认证 |
| 标定合同 | NOT_ESTABLISHED | YAML排列/方向/实际frame别名缺官方消费者 |
| 时间合同 | NOT_ESTABLISHED | 秒编码有支持；录制配置来源未认证；首扫描缺前置IMU |
| 稠密参考资格 | CONDITIONAL_REGISTERED_REFERENCE | 不是独立于测试LiDAR与TLS的真值 |
| 稀疏参考资格 | 测绘来源位置参考，参考点转换未闭合 | 无独立姿态；第三点不在dense时间内 |
| 非GT初始化 | NOT_ESTABLISHED | 本轮未构造任何初值 |
| 存储 | 通过启动30GiB及运行20GiB守卫 | 未导出完整点云，无新安装 |

## 1. 完整 bag 结果

真实反序列化全部66633条消息，实际数量与bag声明逐topic一致。

| Topic | 类型 | 数量 | frame_id |
| --- | --- | ---: | --- |
| /hesai/pandar | sensor_msgs/PointCloud2 | 1094 | PandarXT-32 |
| /alphasense/imu | sensor_msgs/Imu | 43656 | imu_sensor_frame |
| /alphasense/cam0…4/image_raw | sensor_msgs/Image | 每路4376，共21880 | cam0…4_sensor_frame |
| /Point_name | std_msgs/String | 3 | 无header |

bag实际record时间支持：1649856227623466000–1649856337028370317ns，跨度109.404904317s。
点云共64227729点；每帧点数52181–63013，中位58934.5。未预处理、裁剪或用于任何匹配。
schema全段一致，PointCloud2字段和偏移详见 bag_summary.json、topic_inventory.csv。
全部带header话题的header减record为0；每topic重复时间、倒退、重复序列号和相同序列化消息计数均为0。不能由此证明传感器物理同步。

IMU间隔min/median/P95/max分别为2473200/2505000/2510100/2535600ns，值有限，无重复/倒退。
点级timestamp为FLOAT64，有真实帧内变化；没有以型号猜测字段，也没有重建firing time。
固定官方SDK和全段数据支持绝对秒编码解释，但不是本会话配置认证。
按此解释，1093帧具备完整IMU包围；首帧缺47303251ns前置IMU。保留首帧，不裁剪、不外推、不跳过。
正式coverage列未从“有条件的数值包围”升级为PASS，详见 TIME_CONTRACT_AUDIT.md。

## 2. E57 完整解码

实际E57 v1.0，物理文件长度5447128064B，pageSize=1024B，XML偏移5447123768，XML逻辑长度3620。
一个合并data3D扫描，声明和解码点数均为285455240；全部7个属性（XYZ、intensity、RGB）逐批解码。
固定缓冲容量100000点，属性缓冲5600000B；未调用全扫描数组API，未导出PCD或另一份完整大点云。

所有解码数值有限。没有cartesianInvalidState字段，故标志无效点数量为NA，不能写成“经标志检查无效点为0”。
实际nonidentity pose有明确w/x/y/z字段和(112.229971,21.414423,0)m平移。
应用该pose后的根坐标范围约：X[-162.5366094,411.0055061]m，Y[-252.5859952,323.5049443]m，Z[-69.3748836,79.8088418]m。
这些范围不是GT地图关系，也不是定位覆盖认证。完整局部/根范围及bounds作用域现象见 e57_scan_metadata.csv、e57_bounds_scope.csv、COORDINATE_AUDIT.md。

coordinateMetadata为空，未附T_M_W。非单位扫描pose是T_M_S，不能充当T_M_W。

## 3. 阻塞的具体来源

1. **坐标**：缺发布E57根系→GT世界系的有据变换。官方FAQ称GT在Alphasense IMU I，固定xacro却称在base，并将base相对I绕X旋转-pi。同原点不能证明同姿态；没有依据选择是否应用该旋转。
2. **标定**：官方747B YAML身份正确，但未找到该格式的官方消费者，不能认证四元数顺序/方向。bag imu_sensor_frame、YAML imu/base_link、URDF alphasense_imu/base的别名链未闭合；旧生成URDF不是当前xacro的可信展开。
3. **时间**：实际编码支持与物理同步分层保留；录制commit和timestampType配置未知。即便以后明确单位/配置，完整bag首帧仍缺约47.303251ms的起点IMU包围，须由新合同决定如何处理，不能在本轮静默丢帧。

未进行GT拟合、ICP、Umeyama或轴序搜索。没有因为缺少证据就宣称发布数据一定错误。

## 4. 参考与初值

读取了2个唯一GT文件，明确不是GT reads=0。
稠密参考789行，时间1649856227723471000–1649856314319139000ns；它来自测试扫描与测绘地图注册，保持条件性参考。
稀疏3行均在bag与IMU支持内，时间关联marker候选为P30/P7/P36；第三个点在dense末端之后。后四列0,0,0,1未当姿态测量。
稀疏文件注释与正文列数不符；测量尖端到IMU的发布处理说明仍缺失。没有算任何位置或姿态误差，详见 reference_eligibility.md、sparse_label_audit.csv。

正式最小资产与所查官方说明没有给出合法非GT地图初始6DoF位姿。没有利用GT、地图扫描站位或单位阵占位，详见 initialization_provenance.md。

## 5. 资源、解析器与工程反例

启动可用34101174272B（约31.76GiB），满足30GiB门；解析过程保留20GiB外加回执余量。实际结束余量见 storage_audit.json。
复用R2已有pye57 0.4.5隔离target；Python3.8.10、NumPy1.24.4、pyquaternion0.9.9、ROS Noetic rosbag。未建立新环境或安装新软件。
解析器文件和原生.so的实际SHA记录在parser_environment.json，并在结束时重新核验。原wheel未保留，未把安装文件哈希冒称为wheel包哈希。
E57文件的e57LibraryVersion是**写入器元数据**，不是本次解析器版本。

bag遍历41.604268s；E57流式解码112.107388s。两进程记录的getrusage(RUSAGE_SELF).ru_maxrss均为459356KiB，按该计量来源报告，不相加为总物理内存。不是定位实时性测试。

早期环境探测曾因覆盖PYTHONPATH报ModuleNotFoundError: rosbag；根因是探测命令隐藏了既有ROS路径，改为进程内追加解析器路径后解决，未安装系统包。一次官方.gitmodules探测404后未重复该URL；后续固定源码获取成功。两项不是正式bag/E57解析失败，均不隐瞒。
本地反证复核修正了未认证单位误判PASS、资源/截止吞失败回执、无效点范围筛选及非有限平移守卫；实际完整解析前已完成并冻结。11项合成数学/布局测试通过，不代表定位有效。

## 边界与唯一NEXT

rosbag play=0；ROS定位节点=0；NDT align=0；EKF=0；定位轨迹生成=0；ATE/RPE=0；时差搜索=0；bias拟合=0；GT拟合=0；生产算法修改=0；push=0。
五个正式输入的前后SHA快照完全相同，原文件未改。未访问HIKVISION，未重下载大数据。

唯一NEXT：请科研总控验收后，定向补齐Exp18发布端的坐标/标定消费者合同（T_M_W、GT I/base语义、YAML排列/方向及measurement_tip处理），并由新合同明确首扫描IMU边界与非GT初始化；不开展定位或新算法实验。
