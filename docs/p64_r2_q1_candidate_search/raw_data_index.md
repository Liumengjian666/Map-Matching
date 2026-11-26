# 原始评分、地图输入及可视化

- 固定网格的完整输出为 [candidate_search_results.csv.gz](candidate_search_results.csv.gz)，包含 58,212 个真实位姿的两个评分、支持、概率与变换矩阵。解压用 gzip -dc。
- 原始逐点明细放在同一独立工作区 /home/jian/p64_r2_q1_ws/run/raw_archive。分为 Gate 1 点贡献、Gate 1 体素贡献、精修前后点贡献、精修前后体素贡献，以及原生体素均值、协方差和源点成员。使用 xz -dc 解压单表，tar -xJf 将对应压缩包解包到新的路径。大体积逐点文件以本地证据目录交付。Git 提交承载文件身份、完整网格、统计和可视化。
- 对原始逐点文件和受检输入、代码、输出作逐文件校验，见 [原始文件哈希](raw_artifact_sha256.txt)、[候选封存](online_seal.json)、[输入复核](identity.json)。原始运行日志及未压缩证据留于独立工作区 run 目录。
- 三份 raw309 PCD（点云数据文件）用灰色表示地图，青色表示实测扫描。可用本机点云查看器验证，例如 pcl_viewer /home/jian/p64_r2_q1_ws/run/evaluation/figures/raw309_original_overlay.pcd。Git 内的版本为 gzip 压缩。PNG 包含离线参考位置标记。

Git 内的已读 CSV 副本统一使用 LF 行尾，行字段逐项一致。封存哈希以 run 中的原始文件为准。
