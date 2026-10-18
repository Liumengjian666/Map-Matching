# P61 bounded entry protocol

基线：869f476904130917ea2bc342f19ee0862a135765。
优先且本轮唯一处理Shield_tunnel1_gamma。Shield_tunnel5_gamma只保留为未执行备选。

1. 独立worktree，只操作系统内盘；不访问HIKVISION或修改受保护工作区。
2. 先检查现有内盘资产、容量和官方发布目录，再进行小型资源访问。
3. 官方Git资料固定到真实HEAD，保存原字节、来源URL、HTTP状态和本地SHA。
4. 仅解析Google Drive公开网页已经提供的文件列表，不使用认证API、第三方镜像或猜测ID。
5. 目标bag正常大文件确认最多一次；访问探针最多保留64KiB，不能作为完整bag。
   非bag magic／HTML／配额／权限失败立即停止，不自动重试、不续接不同来源。
6. 只有完整真实LiDAR/IMU、外参、时间、地图和初始化全部通过才允许20帧融合Smoke。
7. 不修改生产NDT、EKF、Deskew或Coupled数学，不运行旧Ground-Challenge FoV实验。
8. 地图必须认证同一物理场景及实际XYZ；没有隧道地图不能套用stairs地图。
9. 如采用同序列地图，必须先冻结且标注工程依赖；本轮未建图。
10. 不用GT拟合坐标变换、初始化后重定位或替代NDT结果。
11. 任一关键输入合同不成立即停止，使用GEODE_INPUT_CONTRACT_BLOCKED，不能误报算法失败。

实际本轮在第5项取得明确配额HTML，未进入第6项。
后续标定和地图审计只整理已经取得的官方小型证据，不再发出网络下载。

测试入口（无网络、无ROS、无NDT）：

```bash
python3 -m unittest discover -s docs/p61_geode_native_tunnel_entry -p 'test_*.py' -v
```

工具不是生产数据发布器，不存在本轮roslaunch验收命令。
