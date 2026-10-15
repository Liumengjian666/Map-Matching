# Corridor01 全程 GT / 地图显示：修复未成立

结果：`DISPLAY_ALIGNMENT_NOT_ESTABLISHED`。RViz 已重新打开，但显示的仍是历史
前缀变换下的完整 GT，**不是已正确对齐的全程地图真值**。两次新估计均未应用。
没有修改定位器、原始地图、GT、传感器标定或冻结科研评价。

## 已确认的问题与边界

当前 `T_gt_map` 来自 Control V2 短前缀评价。逆变换用于显示的乘法方向与原评价
一致，但它不是官方认证的全程地图标定。将该变换延伸到约279秒的完整轨迹，
转弯区会明显错位。配置和 RViz 已明确标记 `WHOLE_SEQUENCE_ALIGNMENT_UNRESOLVED`。

归一化地图 XYZ 已经经过 `inverse(T_rawMap_L0)`。PCL 的点云变换保留了原始
`sensor_origin_` / `sensor_orientation_`，因此本文件仍带有原 PCD VIEWPOINT；
不能将这个保留字段无条件再次应用到归一化 XYZ。没有用虚假 TF、镜像或缩放
处理截图中的“反向”。也没有证据足以宣布官方 R/t 错误。

## 实际标定尝试

只使用一个 proper SE(3)，复用 Open3D FPFH / RANSAC / 两尺度 ICP。来源为实际
decoded raw 扫描和合法 header-time GT 插值；没有 GT 逐点去畸变、NDT 或定位回放。
第一试验使用4个拟合扫描和5个验证扫描；一次有限补充使用原拟合集150秒转弯
扫描，验证45/100/160/200/260秒。第二次不是独立科研确认实验，不能用来宣称泛化。

| 尝试 | 拟合扫描通过 | 未参与配准扫描通过 | 关键反例 NN median | 耗时 | 峰值 RSS |
|---|---:|---:|---|---:|---:|
| 多扫描 | 3/4 | 2/5 | 150秒拟合33.445m；120秒验证27.241m | 7.444s | 569540KiB |
| 单转弯锚点 | 1/1 | 0/5 | 锚点0.198m；45秒验证110.897m | 7.284s | 569540KiB |

逐帧全部数值、两个未应用矩阵、输入身份见
[corridor01_alignment_attempts.json](corridor01_alignment_attempts.json)。缓存位于
`.corridor01_display_alignment.PGzVLI/`，不提交生成的训练点云。

这些结果只否定两个候选；不能据此证明不存在固定 SE(3)，不能确认镜像、错误
外参、GT 漂移、地图变形、时间偏移或 deskew 是根因。重复走廊配准误关联仍未排除。

## 复核和补查

一次独立只读复核发现输入断言可在 `python -O` 下失效，已改为显式异常。
用户确认后只执行一次 Codex CLI / gpt-6.1-sol / xhigh 只读复核，退出码0。
其输入完整存于 [corridor01_alignment_cli_input.json](corridor01_alignment_cli_input.json)
的 `input_utf8` 字段（含原始结尾换行），
SHA256=`032bd72ad1523ffd460ff7c0e4e7251510bf53305de96869cab481a6b68301d4`。
原始复核输出 SHA256=`217430e6f9a62d591ea0958b0484996d0daf07aa4a6419fd5ae46cf9a820a350`；
[归档文本](corridor01_alignment_external_review.md) 只补了结尾换行，SHA 因而不同。

CLI 指出记录时间窗口不能独立保证选到全局最近的 header。现工具改为按全部
同话题 header 选最近帧。随后只做一次选择核查、不运行配准：两次使用的14个唯一
扫描的 header 与 payload SHA **14/14完全一致**。该缺口没有改变实际实验输入。
“验证扫描”描述也已明确为拟合与验证扫描均须通过。没有再次执行配准。

4项标定守卫测试、13项GT显示回归、5项launch合同测试通过，共22项。
没有读取 GT 作为滤波观测或初始化。历史脚本 SHA 已保留，但没有两次运行前
完整脚本快照，因此不能声称已独立复现两个历史版本的逐字节代码。

## GUI 验证

隔离 master `http://127.0.0.1:11438`；只运行地图发布器、GT显示和RViz。
地图338210点、完整GT1385点均为 `camera_init`，只是同一个显示坐标标签，
不代表物理坐标标定已通过。界面标题及 Path 名称均标为 `UNVERIFIED`。
无 TF 发布器，无 NDT/EKF/MSCKF 节点，也未影响用户11311 master。
截图已人工检查，转弯错位仍然存在，不宣称修复成功。

## 唯一 NEXT

取得 Corridor01 **原始 PCD XYZ 与 GT world 的可追溯导出变换**，再与已知地图
归一化链组合；在此之前不继续自动换候选、改阈值或人工翻转轨迹。

没有执行 push。当前变更只是负结果及明确的显示限制，不是新的定位成果。
