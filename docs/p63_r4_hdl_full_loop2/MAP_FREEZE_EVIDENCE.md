# 冻结地图与运行身份

|对象|运行前|运行后|
|---|---|---|
|原始PCD SHA256|016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca|相同，已重新计算|
|原始PCD点数|459154|原文件未改|
|原生0.1m target点数|424323|全部3377条native日志均为424323|
|原生target XYZ SHA256|d4ae17e75e22b2c660df1ca08140ad3675479c79f214da6253706c239ca4325e|全部3377条逐次相同|
|/globalmap|map坐标系、424323点及同一XYZ SHA|运行中消息身份一致，无替换|
|上游源码|四份固定Git SHA、pristine|相同|
|实际Release库|复用R3六份ELF SHA|全部相同，未重编译|

PCD路径为`/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd`。未进行UTM减原点、重力对齐、坐标拟合或当前扫描插入。目标几何摘要只认证实际XYZ序列，不冒称对内部所有缓存做了内存逐字节证明；固定上游路径、同一文件、相同target输入及逐次几何身份共同支持冻结。

原始bag在运行前新算完整SHA并逐条预检10905帧。运行中3377条payload／桥接XYZ一一核对，运行后原bag大小及mtime未变；没有在收尾重复读取19.7GB仅为再算同一全文件SHA。

证据：`INPUT_IDENTITY.json`、`INPUT_FRAMES.csv`、`SOURCE_FREEZE.json`、`evidence/NATIVE_AFTER.json`、`evidence/POSTRUN_LINEAGE_CHECKS.json`、`raw/PUBLISHED_MAP.json`及`raw/NATIVE_NDT.csv.gz`。完整源码与二进制路径、哈希均保留。
