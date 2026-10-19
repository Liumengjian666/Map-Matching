# P62-R2 Hilti Exp18 G0 准入回执

**G0 未通过，不允许定位或算法实现。**

主分类：`G0_BLOCKED_COORDINATE_CONTRACT`（证据未闭合，不是已证坐标错误）。并存：`G0_BLOCKED_TIME_CONTRACT`、`G0_INITIALIZATION_NOT_ESTABLISHED`。另有直接工程阻塞：完整bag未取得，E57未进入下载；原始资产仅3/5完成本地身份验证。不得将这些分类解释为实际传感器时间错误、定位失败或算法失败。

## 六项分别判定

| 项目 | 状态 | 实际证据与限制 |
|---|---|---|
| ASSET_IDENTITY_STATUS | INCOMPLETE_3_OF_5 | 两份参考txt与747B标定均实算匹配；完整bag/E57缺失，不能核验 |
| COORDINATE_CONTRACT_STATUS | COORDINATE_CONTRACT_NOT_ESTABLISHED | 未核验E57站位/导出；无可直接采用的T_M_W；官方GT body约定尚有歧义 |
| TIME_CONTRACT_STATUS | TIME_CONTRACT_NOT_ESTABLISHED | 没有完整已验bag，未解析partial，实际topic/点时间/IMU覆盖均NA |
| REFERENCE_INDEPENDENCE_STATUS | CONDITIONAL_REGISTERED_REFERENCE; SPARSE_POSITION_ONLY | 稠密由测试扫描注册参与生成；3个位置标签不能认证独立姿态 |
| INITIALIZATION_STATUS | INITIALIZATION_NOT_ESTABLISHED | 未建立初值；未找到已认证的独立起点/粗定位输出 |
| STORAGE_STATUS | PASS_AT_BOTH_TRANSFER_STARTS | 实际开始分别48746586112、48210587648bytes，均≥48000000000；无删除旧数据 |

本轮不是 `G0_BLOCKED_STORAGE`，也不是 `G0_PASS_FOR_CONDITIONAL_DEVELOPMENT_ONLY`。当前不裁定参考限制是否对论文主张不可接受；该决策归科研总控。

## 实际下载发生了什么

固定revision：`e62017f907007fdc5ab8c721842e4ae7359d7f49`。来源均为官方HF此revision路径，无第三方镜像、无认证绕过。

1. 单连接获得HTTP200、Content-Length=8657654860、正确rosbag magic；289.482186s取得452984832bytes，约1.49MiB/s。**它不是因网络错误退出**：执行AI判断速度不足以在自设1800秒上限内完成，主动SIGINT停止，保留原partial。该性能取舍不等于官方资源不可访问。
2. 64byte探针取得206、`bytes 0-63/8657654860`。
3. 随后采用同文件8个互不重叠Range，各响应206且Content-Range严格吻合。但37.272015s内出现真实 `socket.timeout: The read operation timed out`，总写入3627186bytes后停止、取消后续读取，无自动重试。没有得到任何完整range，失败range身份未单独记入初版工具回执；不得伪造其完成状态。
4. **未再请求E57大文件**。没有完整bag可顺序遍历，也没有E57可完整解码。不能把任务要求的完整解析写成已执行。
5. 3个小资产从R1内盘缓存复用，先核验源，再独立staging复制、核验目标大小/SHA后硬链接发布；路径仍绑定固定revision的预期身份。未以HF元数据冒充本地哈希。

全部事实、URL、大小、摘要和原回执见ASSET_MANIFEST.json、download_evidence.json。两个bag partial均保留；ranged partial的逻辑长度7576092054bytes包含稀疏空洞，**不是已下载7.57GB**，不能按该长度直接续传。

## 坐标审计的实质发现

公式约定T_A_B为B→A，因此T_M_W为W→M。用户合同中“地图到世界”的文字与公式相反，本轮以统一记号/公式记录，未据此构造数值变换。

官方论文说明稀疏控制点与稠密参考有共同TLS地图来源，但没有在本次已查材料中认证发布E57的M与发布参考W的数值关系；本轮未读取E57，不能进一步闭合。**不假设恒等、不拟合、不ICP补对齐。**

官方challenge README称GT在IMU；官方phasma URDF另以base作为GT框架，base与alphasense_imu同原点但含Rx(-π)轴变换。这个发布材料差异需要结合实际导出代码核实，不能任选一种解释。747B YAML含四元数，但所查官方URDF消费者使用的是另一份相机rpy标定，不足以认证该YAML的排列/方向。

## 时间及三个参考标签

实际LiDAR/IMU topic、字段偏移、频率、完整扫描点时间、bag-record差值、每帧IMU覆盖全部NOT_EVALUABLE，理由为缺完整已验bag。没有解析未验证partial，也没有填写预期型号为“实测设备”。

两份GT文件本轮读取用于身份、格式、时间支持和语义检查，GT读取不能记0。稠密789行8列；稀疏3行8列但头部5列说明不一致，需官方消费者解释。三个标签逐条保留，前2个位于稠密参考时间范围、第3个不在其中；**这不证明第三个超出bag**。三者的bag支持、传感器配对与tip转换均未认证。未计算ATE、RPE或其他定位误差。

## 工程代码与验证

原传输工具实际版本：`1519e8d679c55bedac841d8e88f4ac3218d787da`。初版Range工具缓存分支存在“直接写最终路径后验SHA”的保护缺口；本次传输在首个大bag就停止，没有执行此分支。已新增publish_small.py进行独立安全发布，并修复后续工具代码；不得把后来修复版冒充参与原传输的代码。

本地反证复核修正了余量跨界、符号链接目录先创建后检查、缓存直接发布等问题；17项合成测试通过。它们只验证下载/发布保护，不代表数据、时间、坐标或定位通过。用户明确拒绝外部第二意见，本轮外部CLI执行0。

仅在R2内盘目录写入；HIKVISION访问0、生产C++修改0、ROS定位节点0、rosbag play0、NDT align0、EKF0、新定位轨迹0、ATE/RPE0、时差搜索0、bias拟合0。R1及历史资产未修改，现有partial未删除。

## Git与唯一NEXT

START：`97acc922c1d9073c2b28c1877eb61089a0ea4bfc`；开始时R1本地与远端相同且工作树干净。分支：`research/p62-r2-hilti-exp18-g0-admission`。本轮只本地提交，不push；实际分析CODE/END见最终回执，避免文档嵌入自身commit。

**唯一NEXT建议**：由总控决定是否批准一次明确限时、保留原452984832byte线性partial的固定版本资产恢复；先取得完整bag/E57及其SHA，才有资格继续实际时间和E57坐标元数据准入。不能直接进入NDT实验，也不能将本轮未闭合关系用恒等矩阵补齐。当前不自动续传或开始下一轮。
