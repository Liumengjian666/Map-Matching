# Prior map and frame contract — NOT established

已检查官方根目录→groundtruth→map：
<https://drive.google.com/drive/folders/1WjJ5RpZg1IsiYpbwBvn0wBVXsPuUoSdX>。
本次公开列表仅有`stairs.las`，ID `1hxAQe_rPNWHD1WaEjrKEB6r5EeI0-lmR`。
它不能仅因同属GEODE就被当成Shield_tunnel1_gamma的隧道地图。
没有下载这个不匹配场景文件，未认证其实际LAS header或XYZ坐标。

“本次检查没有找到”不等于证明作者从未提供任何隧道地图。
需要能够追溯到同一物理场景、覆盖测试窗口的地图资产，以及地图XYZ坐标定义。

| 关系 | 当前状态 |
|---|---|
| M=官方隧道先验地图系 | UNCONFIRMED_NO_TUNNEL_MAP_ASSET |
| G=参考轨迹世界系 | 未取得实际轨迹，不能认证与M相同 |
| T_map_gt | UNCONFIRMED；不通过全段GT拟合 |
| T_map_lidar(t0) | UNCONFIRMED；没有合法起点初始化 |
| T_imu_lidar | 配置字段可读，但当前gamma设备和IMU适用性未批准 |
| T_map_imu(t0) | UNCONFIRMED；不能由未知链补出 |

只有在所有矩阵有来源时才可使用：
`T_map_lidar = T_map_imu * T_imu_lidar`。
该公式是方向约定，不表示本轮已获得可用矩阵。

官方README说明Metro_Tunnels的Leica参考主要为位置，且跨设备需要参考点转换；
不能假设Shield_tunnel1.txt天然就是Avia完整6DoF位姿，也不能默认参考轨迹与地图同系。

未同序列建图，未使用测试GT建图，未计算ATE或旋转精度，未GT重定位。
如未来必须同序列建图，必须先冻结地图并标注工程实验依赖，不能宣称独立跨遍次定位。
