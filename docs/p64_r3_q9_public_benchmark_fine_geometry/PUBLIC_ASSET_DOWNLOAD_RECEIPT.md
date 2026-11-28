# 公开资产下载及定位准入

## Corridor02

对官方预览、标准下载 URL 和确认下载入口执行了限制在开头 64 KiB 的探测。预览返回文件名页面。ZIP、PCD 的第一次下载返回大文件无法扫描的确认页。完成标准确认后，Google Drive 返回 Quota exceeded，参考轨迹、外参和内参也返回同一配额页。该响应是服务端公开下载配额，当前没有证据表明文件不存在、客户端不支持或用户需要额外授权。探测回执见 drive_probe.csv。未尝试绕过限制。

| 官方文件 | 历史核验规格 | 官方入口 |
| --- | ---: | --- |
| corridor02.zip | 15,589,213,681 B | [录包](https://drive.google.com/file/d/1fbQIjza6zCVZ719VvXfNhAONDZflqGnf/view?usp=sharing) |
| corridor02.pcd | 66,056,109 B | [地图](https://drive.google.com/file/d/1d5vv4kfrTyntZw-82fGeTyih3H7DWxg5/view?usp=sharing) |
| corridor02_gt.txt | 475,805 B | [轨迹](https://drive.google.com/file/d/1hb3IVu1OqyPZA61kBZG1raGTBcTv_h3H/view?usp=sharing) |
| corridor02_extrinsics.yaml | 860 B | [外参](https://drive.google.com/file/d/1FzepVzxan_9GjS0Rg_3f1LIjjwm2VZrR/view?usp=sharing) |
| corridor02_Intrinsics.yaml | 478 B | [内参](https://drive.google.com/file/d/1gCjmOVcwhm55Rwosel79FMxG_WzvnV8g/view?usp=sharing) |
| corridor02.yaml | 1,011 B | [初值目录](https://drive.google.com/drive/folders/1WZsyEYyU-_8ps1CUqRq3YO7IaARMdGNm?usp=sharing) |

如果官方网页可以下载，原文件放在 /home/jian/p64_r3_q9_ws/run/assets/Corridor02/。ZIP 本身需要约 14.52 GiB，地图和辅助文件约 63 MiB。建议保留至少 40 GiB 供归档与解包，确切解包大小用 ZIP 清单确认。本机最初有约 62 GiB，运行中的 Corridor01 另占空间。以下命令核查 ZIP 完整性和历史匹配的两个辅助资产。

    cd /home/jian/p64_r3_q9_ws/run/assets/Corridor02 && stat -c '%s %n' corridor02.zip corridor02.pcd corridor02_gt.txt && unzip -t corridor02.zip && printf '%s\n' '6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228  corridor02.pcd' 'cddb6739230ded86c57412e768e071e5cd6d62a5de5ff310618032ca2e38be0a  corridor02_gt.txt' | sha256sum -c -

尚缺作者明确说明的 T_M_G、参考物理原点及初始化时间绑定。取得文件后仍需闭合这些定义。

## Corridor01 及后续备选

Corridor01 公开附件确认后返回 ZIP 字节及 3,903,086,488 B 总长。保存于 run/assets/Corridor01。历史归档完整 ZIP SHA-256 为 6f83a45868aa5fa7299690bab78b782642295074d14388dd37707a2cc693aa05。历史未压缩录包约 6.47 GB。四个静态官方文件从本机历史公开资产副本复制并逐个校验。Cave01 和 Cave02 的确认入口同样返回配额页。FusionPortable Corridor 的 PALoc 官方定位接口使用 20220216_corridor_day_ref.bag。该线路的地图关联轨迹来源仍需核查，故未把它作为独立真值，也未开启第二套大规模下载。[官方定位库](https://github.com/JokerJohn/PALoc)

Corridor01 本轮给冻结短窗使用的是官方 ZIP 的连续前缀。下载探测证实标准范围返回 206，并检查各段的 Content-Range。共取得 1,789,626,224 B，其中连续前缀 1,621,511,419 B。逐块解压并按标准 ROS bag 记录重封包，得到 **94,070,120 B** 的完整 95 s 传感器子包（19,916 条），SHA-256 为 c1628d92a8b93aedc75a574d3720d806236cd241ad5f03a2424c67f9ea2e961d。完成短窗后停止剩余传输，保留可继续下载的分段。完整 ZIP CRC 和 SHA 未进行本轮验证。归档文件名/长度及四个静态对象分别核对。

PALoc 的在用入口确认为 paloc/launch/fp_os128_corridor_loc.launch，固定版本 6dc6722。配置指定 /os_cloud_node/points、/stim300/imu/data_raw 和自定义外部 prior_map_directory。未检索到本地合法完整发布三件套。已取得可用于短窗的 Corridor01 后，未再下载其他序列。
