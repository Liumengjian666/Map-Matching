# P32-R5固定GT辅助诊断（NN计算前冻结）

START：a5798d3398503dd8ee7d6bcaf698735d1e1e8fa5。
仅Corridor02真实GT、map、RC1标定、active initial YAML、raw bag。
GT允许离线语义/几何诊断，不进入任何正式定位初始化或运行配置。
运行节点、标定原值、原数据、旧缓存与报告不改，不自动push。

记T_target_source。H1：GT=IMU/Body、T_BI=Identity，
T_WL=T_WGT*T_I_L；H2：GT=LiDAR，T_WL=T_WGT。
T_I_L来自本场景laser_to_imu原矩阵，不用逆方向或Corridor01数值。
该方向有既有官方schema和工程使用支持；GT Body=IMU仍未认证。

只取C0：T_MW=Identity；C1：T_MW=T_yaml；C2：T_MW=inverse(T_yaml)。
读取active OpenCV字段，保留原数字；不试注释矩阵、其它yaw或偏置。
每帧T_ML=C*T_WGT*[T_I_L或Identity]，完整矩阵记录。
inverse使用实际4×4逆，不把近似舍入矩阵转置当作精确逆。

GT按官方TUM字段timestamp xyz qxyzw解析，秒Decimal转整数ns。
只线性插值p、normalized shortest-arc quaternion slerp；不外插。
最早5个header位于GT真实支持范围的扫描，且完整扫描end也在范围内。
先选取并冻结，不按NN结果选帧；raw LiDAR遍历上限100条，不扩大搜索。
本轮选RAW模式：同一raw点顺序、只剔除非finite XYZ并记录计数，
不做GT逐点deskew，不称为定位NDT Source。GT仅在header时刻刚性变换整帧。
不提取图像、不使用IMU或相机未来数据。

Target沿用PCL voxel0.20m、PointXYZ、KdTree最近邻；同一5帧Source用于6假设。
共5×2×3=30次整帧NN诊断，记录bbox、median/P90、<=.5/2/8m覆盖。
每帧全部有限点参与，无ICP/NDT/pose refinement/normalization/ROS回放。

支持判据沿用既有工程量级：同组合5/5帧median<=1m、P90<=3m、
.5m覆盖>=50%才标记数值几何支持；不是官方语义或非Oracle初始化证明。
全部失败则GT_ASSISTED_FRAME_HYPOTHESIS_NOT_SUPPORTED，不扩大搜索。
H1/H2因零杆臂原点相同、旋转仅约.544deg；必须报告差异，不以相近NN
宣称Body=IMU。无合法几何支持或不足以辨识时GT_REFERENCE_AMBIGUOUS。
若相应C下两者均合理，median差<=.1m且coverage差<=5个百分点仅作为
几何难区分的工程标记，不是统计显著性或严格原点证明。

结果按逐帧、同组合5帧稳定性及参考点敏感性报告；原数据大文件不入Git。
完成30组后停止，科研总控决定唯一NEXT。
