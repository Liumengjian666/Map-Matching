# P61 — GEODE Native Tunnel Entry

正式结果：`GEODE_INPUT_CONTRACT_BLOCKED`。

**唯一立即阻塞：没有取得完整官方原始bag。**
目标下载入口的大文件确认页可以访问，但正常确认后返回HTTP200的Quota exceeded HTML，
不是rosbag。没有运行任何定位或退化实验；这不是NDT失败证据。

## 六个必须回答的问题

| 问题 | 实际结果 |
|---|---|
| Shield_tunnel1_gamma是否可获取？ | 官方发布且可列目录，bag公布大小4,500,527,001bytes；本轮正常下载确认后配额受限，未取得完整bag。不能宣布数据已恢复。 |
| 真实LiDAR/IMU输入是什么？ | 官方资料声明Avia、`/livox/lidar`、`livox_ros_driver/CustomMsg`；外置Xsens `/imu/data`，另有BMI088 `/livox/imu`。因无完整bag，实际消息、frame_id、逐点时间、IMU覆盖仍未认证。 |
| 官方外参是否适用？ | 未批准。官方Git仓库gamma_config含Ouster话题、16线配置、IMU话题不一致及重复键，不能直接套用；这也不等于已证明矩阵数值错误。 |
| 是否存在合法独立先验地图？ | 本次检查的官方`groundtruth/map`只列出`stairs.las`；未找到可认证的同场景隧道地图。不能把stairs地图用作隧道Target。 |
| 是否真正运行融合NDT Smoke？ | 否。Deskew、NDT align、native jet、EKF/OOSM更新全部0；没有启动ROS。 |
| 是否具备真实退化定位实验条件？ | 尚不具备。先解决完整bag获取；其后仍须独立闭合标定、点时间、隧道地图和初始化合同。 |

## Git与隔离

- 仓库：<https://github.com/Liumengjian666/Map-Matching>
- worktree：`/home/jian/livox_ws/p61_geode_native_tunnel_ws`
- 分支：`research/p61-geode-native-tunnel-entry`
- START_SHA：`869f476904130917ea2bc342f19ee0862a135765`
- CODE_SHA（元数据／受控访问工具）：`0ca33208b15d42a879ed0ad31226ba7407b68ea2`
- END_SHA：本轮最终回执及内盘`/home/jian/geode_p61_data/git_receipt.json`记录提交后的完整SHA，避免提交自引用。
- REMOTE_HEAD：查询时P61分支不存在；P60-R5远端为START_SHA。
- PUSH_EXECUTED：`false`。未获本轮push授权，提供命令但不执行。
- 未改生产`src/`、NDT/EKF/Coupled数学或历史R4/R5归档。
- 未改稳定机器狗工作区；原工作区既有大量未提交内容未暂存、覆盖或回滚。
- 所有新证据均写入内盘，未访问HIKVISION。

## 数据来源与访问事实

GEODE官方Git仓库固定于`c6e930623d4fed450d7fc50e16e3ffe0288b692b`。
官网列Shield_tunnel1_gamma为gamma手持、约542秒、691.85m、Hard；这些是公布值，不是本轮实测。

内盘检索：`/home/jian`最大深度7，按Shield/tunnel名称查找，排除cache/local/node_modules，
不跟随符号链接；未发现可用bag副本。不是全磁盘健康／完整资产审计。
开始内盘约74GiB可用，归档时约73GiB；没有空间阻塞。

从官网Google Drive根目录逐级获得真实ID，未猜测文件ID，也未下载整套GEODE：

- 目标bag：`11Z4tmdEXGMuiLDKazd9MoALYJEbiGpE9`
- 目标序列目录：`1lxiNFxb208r4_7YgKDFQKJ_z5Vx5Xp37`
- 对应轨迹`Shield_tunnel1.txt`：`1wTdSus7r1CTLnr7G0s5Q0wYDXkqifTYd`
- 共享gamma配置：`1GheigvWF1YTTw-2y4XrN4f-5i5wh2m4s`
- 官方地图目录：`1WjJ5RpZg1IsiYpbwBvn0wBVXsPuUoSdX`

bag初始请求返回2,472bytes病毒扫描确认页，**当时不是配额失败**。
仅正常提交该页给出的同一文件官方确认参数一次，请求Range `bytes=0-65535`。
没有认证登录、captcha破解或配额绕过，没有重试：

```text
HTTP 200
Content-Type: text/html; charset=utf-8
Content-Length: 2040
Quota exceeded
rosbag magic: FAIL
complete bag: false
```

2,040bytes错误体SHA256：
`c6727431d8666abf04ec3177f46fdd7cad3329c6d8ca3ea0b3223af27b76921c`。
**该SHA只是HTML证据的哈希，不是bag SHA。** 完整bag SHA为null／未取得。
共享标定和GT下载也各返回一次Quota exceeded HTML；Git固定版本标定可以读取。
没有继续下载Shield_tunnel5_gamma，保持本轮仅处理一条序列；没有探测第三方镜像。

## 标定和地图后续门槛

详见CALIBRATION_CONTRACT.md和MAP_FRAME_CONTRACT.md。
`gamma_config.yaml`读取成功，4,731bytes，SHA256：
`19cda4d48f5b185d93293393846a387a2b093c67dc9ec3a646ed8f826b587dbe`。

该文件第48行imu_topic=`/imu/imu`，第59行pointcloud_topic=`/os1_cloud_node1/points`，
第65行VERT_RES=16；与官方gamma Avia话题表不匹配。
第109和119行重复`T_LiDAR_CamL`，相机字段也有重复；没有用普通YAML字典静默覆盖。
只按文本提取唯一`T_IMU_LiDAR`作数值审计，没有将其用于运行时。

官方公开地图目录本次仅见`stairs.las`，未下载错误场景地图。
隧道地图XYZ、`T_map_lidar`、`T_map_imu`、`T_map_gt`和合法初始化均未确认。
没有同序列建图、GT拟合、在线GT反馈或绝对ATE评价。

## 验证和运行资源

7项元数据／证据测试通过：公开列表解析、失败关闭、同一文件确认限制、拒绝外盘目标、
拒绝非官方初始源、不覆盖旧文件，以及HTTP200配额HTML不被提升为传感器输入。
检查确认仅新增本目录工具和归档；生产和R5归档diff为空。

根据code-review-and-quality技能进行了本地正确性、边界、可读性、安全和资源检查；
没有宣称外部CLI或跨模型复核。根据debugging-and-error-recovery技能保存了HTTP原文、
状态和SHA，区分确认页／配额页／真实数据，没有猜测下载成功。

没有定位进程，因此NDT/EKF VmHWM、align耗时、Gaussian Grid实测、Hessian弱方向、
地图覆盖和OOSM比例均为**未测量**，不是0ms或100%通过。

唯一NEXT：由科研总控／用户通过官方授权渠道取得该**单一序列**的完整bag并提供内盘位置，
然后另行批准后续输入和地图合同核验。本轮已停止，不自动重试、不开始完整回放。
