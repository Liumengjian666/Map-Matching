# GT来源与证据等级

实际输入：Corridor02/gt/corridor02_gt.txt，5522条，TUM格式。
首/末时刻：1645999726.984117 .. 1646000619.653331 s。
SHA256：cddb6739230ded86c57412e768e071e5cd6d62a5de5ff310618032ca2e38be0a。
完整GT文件不提交Git；只保存输入SHA、时间支持及固定假设派生矩阵。

本轮实际读取官方SuperLoc页面：
https://superodometry.com/superloc.html
数据表将Corridor02列为SuperLoc/Hawkins/RC1，明确给出TUM字段
timestamp xyz q_x q_y q_z q_w，并提供GT、标定和初始化链接。
该页面没有明确发布GT参考点是Body、IMU还是LiDAR，没有认证T_BI=I。
Body=IMU仍是用户要求检验的假设，不能继承Corridor01的GT IMU原点结论。

方向依据分层：本场景官方文件key为laser_to_imu，既有官方schema
审计把同类imu_laser字段定义为imu^R_laser，项目adapter亦按R_i_l与
R_l_i=transpose(R_i_l)使用。本轮读取RC1原矩阵，不复制RC2数值。
这支持采用T_I_L而非T_L_I，但不证明GT exporter输出哪个参考点。
本轮只读检查了既有adapter对应矩阵使用，不运行或修改它。

GT世界W与名字含darpa的YAML世界不自动等同，更不把darpa世界当成
运动机体B。C0/C1/C2只是数值诊断，不是官方source/target方向认证。

最早合法扫描为原第8帧：1645999726984117031ns，GT首时刻为
1645999726984117000ns；差31ns。原scan1–7均早于GT支持，尤其
scan3=1645999726479818106ns不外推。
时间用Decimal整数ns，p线性插值、normalized shortest-arc q slerp，
不使用bag record时间匹配GT。完整扫描end也在GT时间支持内。
RAW点云在header时刻刚性变换；不做GT逐点运动补偿，不称为运行Source。

结论证据等级：数据格式/实际时间支持已核验；GT参考点官方语义未认证；
本次固定6假设几何均失败，不能以细微NN差异选择GT原点。
