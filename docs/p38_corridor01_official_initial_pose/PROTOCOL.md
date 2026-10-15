# P38 单帧官方数值初值验证合同（先于几何结果）

TASK：P38-CORRIDOR01-OFFICIAL-INITIAL-POSE-VALIDATION。

把真实生效 YAML 的完整 4×4 数值 Y 视为待检验的某时刻位姿，不再作为 GT 轨迹的
固定变换。官方发布页明确提供 initial poses 用于定位，但其具体帧方向和 `# s 67`
语义尚未闭合，不将文件位置自动解释成 `T_map_lidar(67s)`。

- 采用 `p_A=T_A_B*p_B`。仅 A=`Y`、B=`inverse(Y)` 两个直接放置。
  两者均暂按 LiDAR→原始 PCD 地图的数值假设。没有 GT 乘法、附加外参或轴搜索。
- 实际 YAML 位于 `Corridor01/initial_pose/corridor01.yaml`，而非本机不存在的
  `init_pose` 目录。读取 active OpenCV R/t；保留文件原数值，检查齐次行、有限性、
  det 和正交残差（沿用已有工具 1e-5 容差）；逆使用完整数值逆，不替换为转置逆。
- 主要实验时间零点：原始首 LiDAR header `1517157219088119030 ns`。
  目标为该 header +67s，不依据 GT 选择时刻。
  全部2777个严格递增 header 索引中唯一最近帧固定为 raw665，
  `1517157286055073023 ns`，相对 `+66.966953993 s`。
  其 record 在完整解码 bag 中为 scan_end，不等同于原 bag 的 record 时间。
- 从真实解码 bag 重读此帧，核对其有序 XYZ/time 与标准缓存转换；XYZ SHA
  必须等于 P33/raw665。Source为 raw、header时刻刚体投影，不额外deskew。
- 固定 P33/P36 完全相同的原始 PCD、工具及预处理：range .5–80m，source
  voxel .25m/cap1400，target连续两次 .15m；PCL KdTree；median取排序N/2，
  P90取floor(.9*(N-1))，overlap采用<=阈值。PCD VIEWPOINT不作为变换。
- 第一阶段恰好一次探针 `--NN-only`，两个槽位分别传入Y及inverse(Y)，
  只产生两组NN统计；其历史H1/H2名称不代表本轮读取或使用GT。
- 与 P33/raw665、P36/raw665 的预处理 XYZ 必须逐字节一致。
  绘制XY/XZ/YZ三个投影，所有面板坐标单位、比例及绘图范围统一，不为候选单独缩放。
- 只在初始几何具有合理依据时允许一次NDT。本轮采用既有初始化验收尺度作为
  保守准入：median<=1m、P90<=3m、0.5m overlap>=50%，并需图示无明显结构矛盾。
  这是执行前固定的工程门槛，不是GT调参或统计显著性判据。
  两者均不满足则NDT=0并停止；仅一个满足则只用该候选一次；两者均满足则
  主假设A优先，报告歧义而非用GT选择，不为B追加NDT。
- 若进入NDT，参数沿用原resolution=.8、step=.08、epsilon=.001、max_iterations=40。
  不运行ICP、完整定位、候选网格或第二次NDT。
- 候选矩阵及源身份写入冻结文件后，才单独解析本轮GT文本作覆盖/同时间插值/数值诊断。
  准备阶段复用的P33 JSON确实含历史GT数值，但不使用其位姿矩阵构造本轮候选。
  GT不参与选帧、构造初值、候选排名或几何/NDT准入。直接数值差不称为定位误差。
- 真实输入/map/GT/YAML和索引小文件SHA新计算；大bag/cache完整SHA明确继承P34，
  本轮核对大小和该实际消息内容，不宣称重算大型文件完整SHA。
- 仅修改本新归档及独立本地缓存；不修改运行节点、标定、原数据、历史归档或受保护工作区。

结果分别报告几何支持与官方语义认证。几何失败不否定其他时刻、传感器或地图版本的
合法使用方式；不增加新候选以找通过结果。初始化失败则保留离线图，不开发RViz入口。
