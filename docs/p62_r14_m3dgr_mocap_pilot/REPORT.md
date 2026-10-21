# P62-R14：资产准入阻塞，未执行定位实验

正式状态：`BLOCKED_ASSET_ACQUISITION`。存储子门：`BLOCKED_PEAK_SIZE_NOT_CERTIFIED`。

本轮按用户停止合同结束；不是算法失败结论，也未完成独立位姿评价。

## 身份与实际执行

- 仓库：https://github.com/Liumengjian666/Map-Matching
- 分支：`research/p62-r14-m3dgr-mocap-pilot`。
- 工作区：`/home/jian/livox_ws/p62_r14_m3dgr_pilot_ws`。
- 冻结起点：`cfae39344a17ac2ee5e598fd7875d33f8c109794`；执行前 R13 本地与远端 HEAD 相同，原工作树干净。
- 实际前检 CODE_SHA：`5bb6ad76596a5046cb7461f3d7b84ed5d09b3cc5`。未编译研究定位二进制。
- 已执行：限定目录文件名盘点、磁盘检查、官方小型说明/摘要获取、一次官方 bag 链接 HEAD 请求、保护快照、6项存储门单元测试。
- 未执行：bag/GT 下载及解码、坐标/时间闭合、地图构建、测试帧选择、预测、NDT、GT误差评价、EKF、ROS回放。

## 阻塞的可复核证据

前检时可用 `22,944,526,336 bytes`，安全预留 `21,474,836,480 bytes`，新增资产余量 `1,469,689,856 bytes`。

官方固定版本的序列表列出 Wheel-float01 为约 `1.5g`。这是舍入大小，不是完整文件长度。仅作乐观容量示例，按1.5十进制GB计，下载后剩 `21,444,526,336 bytes`，低于20GiB预留 `30,310,144 bytes`，尚未计入GT、地图及解析临时空间。**不能把这一示例当作实际文件精确大小的测量**。实际文件大小和处理峰值均未认证，因此不获准开始下载。

一次本机 HEAD 返回 HTTP200、`text/html; charset=utf-8`、Content-Length322312；这是OneDrive网页，不是 bag 响应，不能作为 bag 大小或可获取性证明。没有宣称官方链接全球失效，也未继续尝试镜像或绕过访问控制。浏览工具对bag/GT入口的访问失败仅作辅助现象，本机正式回执见 `PREFLIGHT_RECEIPT.json`。

限定本地文件名盘点未发现目标名称；范围及局限见 `LOCAL_INVENTORY.json`。这不排除范围外或改名的已有文件。

## 合同与科研验收

坐标、时间、GT参考资格、轮式预测、地图、测试帧和参数集成均为 `NOT_EVALUATED_AFTER_ASSET_BLOCK`。不能将官方平台描述代替实际会话合同；未假定动捕刚体等于IMU、wheel或雷达。

S1–S6全部 `NOT_EVALUABLE`：没有独立位姿误差、5°误拒、事件准确性或恢复正确性的新证据。事件不是“0个已观察”，而是未运行、数量未知。无测试样本，误差及性能字段为null，CSV仅保留表头。

实际计数：传感器载荷下载0 bytes；GT位姿文件读取0；地图构建0；新增NDT align0；EKF0；ROS定位节点0；rosbag play0；定位轨迹0；生产修改0。

## 保护、归档及唯一NEXT

前检保护快照一致：稳定工作区HEAD/index/refs/reflog/状态及被快照文件、R13 HEAD/状态及归档摘要清单身份未变。最终离线复核另见 `FINAL_PROTECTION_RECEIPT.json`。这不是重做历史保护备份或改写R10历史结果。

仅提交本轮前检程序、测试和阻塞证据。不push，不进入R15。

唯一NEXT：请科研总控先解除**同一Wheel-float01序列的资产准入阻塞**：确认可安全使用的存储容量及峰值预算，并提供可获取、可核验精确大小和身份的官方bag/GT入口；随后另行决定是否恢复R14。当前不自动重试、换数据集或开展算法实验。
