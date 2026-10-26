# 外部先验地图导入与冻结

正式输入为已有 FAST-LIVO2 PCD：

`/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd`

7346648 bytes，SHA256 `016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca`。二进制 PCD，FIELDS=`x y z rgb`，SIZE=4/4/4/4、TYPE=F/F/F/U，459154 点。原生 PointXYZI load 发出缺 intensity 的格式提示，但成功读取 XYZ，未另造强度、未转换或重写原 PCD；本连续配准只用几何，不用 intensity 做权重。

研究配置 `loop2_identity_smoke.yaml` 保持作者几何参数，设置 `lio.relocation.update_map=false`、`lio.map.save_map=false`。作者 map path 带 ROOT 前缀，因此在独立上游 clone 的 map 目录建立 `p63_loop2_prior.pcd` 符号链接指向上述文件；没有向原数据目录写入。

## 源码闭合

1. SuperLIOReLoc::init 调用派生 map_init；加载 PCD 后一次性 `ivox_->insert(point_map_v3)`。
2. 进入 inherited stateWaitMapInit 时仍虚派发到派生 map_init，static pcd_loaded 已为 true，不插入当前扫描。
3. 连续 UpdateMap 虚派发到派生实现，开头即 `if(!g_update_map)return`，唯一跟踪插入语句在其后。
4. 基类 mapping 实例的 map_init／UpdateMap 确有 insert，但不走本 relocation 派发路径。
5. 初始化后 `point_map_->clear()` 清除原始加载临时云；不清除 OctVox 的固定目标。原 ROS 全局地图已经复制。
6. OctVox resetMap 定义无本路径调用；近邻查询不修改几何。搜索状态／缓存变化不等于目标地图变更。
7. save_map=false 使 cache/save 路径提前返回；作者保存地图函数含清理保存目录代码，本轮未进入。

## 运行验证

| 证据 | 输入／动作 | 实际结果 |
|---|---|---|
| `MAP_FREEZE_PROBE.json` | 真实 PCD 加载一次，600 次关闭状态的非空 synthetic UpdateMap，1000 次原生 KNN 查询 | 151401 目标代表点；995 查询至少 4 邻居；前后摘要相同 |
| `raw/loop2_smoke_01/OBSERVER_RECEIPT.json.gz` | 真实 107 扫描已发布、95 跟踪输出后失败退出 | 同样 151401 点，前后摘要相同；没有增量建图 |
| `FINAL_DELIVERY_CHECKS.json` | 完成后重新读取小型原始地图 SHA | 与输入 SHA 相同 |

两次运行目标 float XYZ 数组 SHA256 均为 `57e4bc2d68c072123aa7e0f490b05a637389ad80dd5211d93c4a341cb50f9745`。459154→151401 是原生 OctVox 初始化紧凑化，不是后续地图丢失或扫描插入。摘要针对本机原生顺序的 float 几何；不声称封存了每个非几何缓存字节。

**MAP_FREEZE_PASS**：配置足以冻结，无需作者源码修复。该判定与位姿是否正确、外参是否标定、地图是否重力对齐分开。本轮定位失败不影响这个局部工程事实。
