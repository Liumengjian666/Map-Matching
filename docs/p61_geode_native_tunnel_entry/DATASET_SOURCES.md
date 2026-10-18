# 官方来源与逐级身份

## 发布资料

- 官方仓库：<https://github.com/PengYu-Team/GEODE_dataset>
- 固定README：<https://github.com/PengYu-Team/GEODE_dataset/blob/c6e930623d4fed450d7fc50e16e3ffe0288b692b/README.md>
- 官网：<https://thisparticle.github.io/geode/>
- 固定标定：<https://github.com/PengYu-Team/GEODE_dataset/blob/c6e930623d4fed450d7fc50e16e3ffe0288b692b/cali/gamma_config.yaml>
- 官方ROS话题图：<https://raw.githubusercontent.com/PengYu-Team/GEODE_dataset/c6e930623d4fed450d7fc50e16e3ffe0288b692b/README.assets/image-20240717154348844.png>

GitHub README和YAML原文、话题图已归档official/；未改写原始配置。
官网／README的“天然退化”描述不是本轮实测Hessian或定位失败证据。

## 数据入口

- 根目录：<https://drive.google.com/drive/folders/1hEn3sBAvQhSdUFnGMZCCv-W0Ynj2rWBs>
- sensor_data：<https://drive.google.com/drive/folders/1w3mi2iUHVHQq3kytuJV3p4BPFYN2V2gL>
- Metro_tunnel：<https://drive.google.com/drive/folders/1Hww_-msqO1PTvgAtXRG6x9L2eqDbM51N>
- Shield_tunnel1_gamma目录：<https://drive.google.com/drive/folders/1lxiNFxb208r4_7YgKDFQKJ_z5Vx5Xp37>
- **bag**：<https://drive.google.com/file/d/11Z4tmdEXGMuiLDKazd9MoALYJEbiGpE9/view>
- GT目录：<https://drive.google.com/drive/folders/18AObN_YJUIbnAA9yw_7rd6DetTn1tPv1>
- **Shield_tunnel1.txt**：<https://drive.google.com/file/d/1wTdSus7r1CTLnr7G0s5Q0wYDXkqifTYd/view>
- 标定目录：<https://drive.google.com/drive/folders/14VR6U0d4u6yN-qIiUpbH4bk9NRHV17QG>
- 共享gamma配置：<https://drive.google.com/file/d/1GheigvWF1YTTw-2y4XrN4f-5i5wh2m4s/view>
- 官方地图目录：<https://drive.google.com/drive/folders/1WjJ5RpZg1IsiYpbwBvn0wBVXsPuUoSdX>

所有ID来自已读取的官方页面／目录，不把另一个文件拼到目标数据中。
bag目录还列Camera、imu、LiDAR文件夹；未下载它们绕过bag失败，不扩张恢复路线。
source_manifest.json保存公开列表、实际大小字段、HTTP证据和原始内盘证据SHA。
完整网页只留内盘`/home/jian/geode_p61_data/official_metadata/`，不是大型传感器数据。

README另公布作者百度分享入口：
<https://pan.baidu.com/s/19t2WhhUvNnNqKX0zLDhRIA?pwd=sysu>（提取码sysu）。
本轮没有访问或下载该入口，没有将授权入口存在误报为文件已获取。
如科研总控决定人工恢复，可由用户通过合法浏览器操作选择单一序列。
