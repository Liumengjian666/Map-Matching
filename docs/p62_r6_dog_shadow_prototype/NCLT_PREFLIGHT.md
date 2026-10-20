# B线 — NCLT最小资格预筛（完成后不继续）

结果：**NCLT_PREFLIGHT_BLOCKED**。并存 **CROSS_SESSION_OVERLAP_NOT_ESTABLISHED**、**INDEPENDENT_EVALUATION_NOT_ESTABLISHED**。
实际检查于2026-10-20约08:37–08:41 UTC完成（少于30分钟）；只读官方页面/PDF及已有缓存和文件清单，无大型数据下载，无SLAM/地图构建。

来源：[官方页](https://robots.engin.umich.edu/nclt/index.html)、[官方论文](https://robots.engin.umich.edu/nclt/nclt.pdf)。复用R1缓存nclt.pdf/nclt.txt/nclt_page.html，身份由归档manifest记录；本轮在线核对官方页与论文，不使用第三方同步或标定。

固定建图候选2012-04-29、测试2013-01-10，不更换日期。

| 问题 | 实际证据 | 结论/限制 |
|---|---|---|
| 两会话重叠 | 论文Table1长度3.1km及1.1km，一般性说明校园重复采集 | 没有这两天轨迹/覆盖多边形，不能认证可定位重叠 |
| 非GT独立建图 | 原始Velodyne和IMU/里程计格式公开 | 原理上可只用建图日，实际数据/成功运行未核验；没有建图 |
| 建图日地理注册 | 论文Sec3/Table3规定NED及固定GPS线性化原点 | 仍需该日合法RTK覆盖与杆臂，不能用测试轨迹拟合地图 |
| 测试日独立位置 | 存在gps_rtk.csv；论文说明1Hz及室外误差风险 | 本地未找到两日原始RTK；时间/质量/覆盖未认证 |
| RTK质量字段 | Table7有fix mode及卫星数，mode2是经纬度可用、3含高度 | 不等于RTK fixed/float认证或误差上界；gps_rtk_err来自SLAM比较，不能作为独立质量证书 |
| 外参 | Table4给T_body_vel和T_body_imu等，RTK杆臂[-.24,0,-1.24]m | 官方定义存在，不是本轮实物精度认证；评价应预测天线点，不用未知姿态强扣杆臂 |
| 点时间/IMU | Sec7.3: raw packet UTIME是最后一次firing，包时长552.96us；IMU UTIME为epoch微秒、gyro rad/s | 原始每点未保存独立firing时间；packet近瞬时近似须披露，不能冒充精确逐点时间。velodyne_sync已去畸变且在body系，不可当原始雷达点 |
| 同步 | Sec5.5:共同记录电脑；多数传感器用Olson同步，KVH按到达时间 | 不宣称会话中每字段已验证 |
| 是否需共同SLAM | 官方发布参考含跨会话激光约束统一世界系 | 发布高频GT不独立；独立方案须建图日RTK地理注册，不沿用共同图当独立证据 |
| 重复结构场景 | 数据集一般包含室内hallway | 未认证固定两日期内有对应目标重复结构区间 |
| 存储 | 实测可用30534504448B；页面两压缩包约8GB+3GB | 仅压缩包+保守20GiB余量已约32474836480B，缺约1940332032B，尚未计解压/缓存。页面GB是近似，非实际包大小认证 |

本地有界清单只有R1论文/页面缓存，没有两会话原始数据。本轮不以下载来补证据，不删除旧资产换空间。
未来如果需要独立位置评价：应直接比较估计天线位置p_M_B+R_M_B*p_B_RTK与合法原始RTK，质量筛选及时间配对先注册；不能从联合SLAM误差挑选“好RTK”。没有独立姿态真值时不得宣称独立6DoF通过。

本预筛不影响A线继续。它既不是NCLT正式准入，也不是已证明NCLT不可能使用。后续由总控决定，当前不扩展到其他日期或其他数据集。
