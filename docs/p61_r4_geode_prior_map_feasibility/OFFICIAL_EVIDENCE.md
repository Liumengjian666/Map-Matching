# 有界官方证据和来源

1. [GEODE官方README](https://github.com/PengYu-Team/GEODE_dataset/blob/c6e930623d4fed450d7fc50e16e3ffe0288b692b/README.md)
   当前GitHub提交仍为c6e930623d4fed450d7fc50e16e3ffe0288b692b，树未截断。
   Localization Evaluation说明Leica仅位置、alpha评价参考及gamma结果转换。
2. [原论文v2](https://arxiv.org/html/2409.04961v2)：
   4.1.3隧道前进/返回路线；4.4.1明确RTC360地图用于楼梯；4.4.2/4.5解释Leica位置与参考点转换。
   6.2/6.3说明采集期间标定及时钟的现实限制。
3. [官方地图目录](https://drive.google.com/drive/folders/1WjJ5RpZg1IsiYpbwBvn0wBVXsPuUoSdX)
   本次正常公开目录页面200，仅见stairs.las，未下载地图大文件。
   原HTML保留内盘，official/map_listing.json及source_manifest.json保留列表与来源身份。
4. [Shield_tunnel1位置参考](https://drive.google.com/file/d/1wTdSus7r1CTLnr7G0s5Q0wYDXkqifTYd/view)
   正常官方uc下载一次即得到86413字节有效文本，未登录/绕过权限，未重复重试。
5. [官方Drive gamma标定](https://drive.google.com/file/d/1GheigvWF1YTTw-2y4XrN4f-5i5wh2m4s/view)
   下载一次得到4682字节；不同于旧Git4731字节版本。原文保存在official/gamma_drive.yaml。
6. [真实大小写GT转换脚本](https://github.com/PengYu-Team/GEODE_dataset/blob/c6e930623d4fed450d7fc50e16e3ffe0288b692b/script/gamma2GT_leica.py)
   旧小写URL失败不能证明该脚本缺失；本轮使用树中实际路径，取得200原文。
7. [作者评价仓库](https://github.com/thisparticle/GEODE_Evaluation/tree/1f008a7249e36393a1752622de50660b77b5b7f4)
   由官网链接进入；默认分支master，正确树完整返回。
   曾对main查询一次404，作为分支元数据错误保留；未将其当数据缺失。
   只获取FAST_LIO carol配置、launch、测试脚本、laserMapping.cpp和根README共5个小文件。
   全部仅作源码参考，不编译、不执行、不移植到生产；未来集成需单独检查依赖/许可证。

小文件HTTP状态、最终官方URL、bytes和本地SHA均在source_manifest.json。
作者未发布SHA的资产不声称通过作者校验。网页/论文事实与实际文件数值分别标注。
没有请求其他序列原始数据、下载新bag或访问第三方地图。
