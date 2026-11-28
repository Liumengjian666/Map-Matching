# 结构与对应的真实点云证据

三处展示为西侧平台第 55 帧、西侧转角第 235 帧、东侧廊道第 680 帧。图中灰色是完整 E57 先验裁剪，蓝色是满足距离与粗糙度门槛的查询支持，黑箭头在西侧平台表示最弱六维模态的旋转分量，后两处表示受控平移轴。每处分别保留点云地图、查询扫描、配对目标、方向选择与删除点集。箭头表示测得最小方向；是否退化仍按各方法冻结门槛判定。

候选微小结构通过 0.055 m 图上的可支持局部面、周边 0.12–0.30 m 环带平面和有界有符号位差筛选。仅西侧转角的第 225 帧在 A 输出附近得到 6 个匹配观测，全部映射到同一地图点。点位约 (101.184, 29.979, 0.754) m，法向约 (0.947, −0.320, −0.003)，相对环带拟合的有符号位差约 0.060 m。相符同面邻域只剩该点，面片尺寸不可估，其他固定邻近帧在该点 0.08 m 内无重复支撑。将其保留为未认证的结构假设。未将普通大平面、边缘、NDT 体素边界或单一疑似点记作真实厘米结构。

G/H/I 每组均尝试构造实际对应并调用各段协议的同一解算器，14/14 帧每组均低于 12 个最小合法细节对应而回到父姿态。删去候选后的全局部点面 F 对照基本不变。下述图与点云展示了真正使用的对应和控制面，而不是以误差改善推断结构存在。

- [候选坐标、法向和重复可见性](PHYSICAL_STRUCTURE_EVIDENCE.csv)
- [西侧平台](visuals/Exp18_55_patches.png)
- [西侧转角](visuals/Exp18_235_patches.png)
- [东侧廊道](visuals/Exp18_680_patches.png)

```bash
pcl_viewer /home/jian/p64_r3_q10_ws/docs/p64_r3_q10_b_real_failure_fine_geometry/visuals/Exp18_55_map.pcd /home/jian/p64_r3_q10_ws/docs/p64_r3_q10_b_real_failure_fine_geometry/visuals/Exp18_55_scan.pcd /home/jian/p64_r3_q10_ws/docs/p64_r3_q10_b_real_failure_fine_geometry/visuals/Exp18_55_paired_target.pcd
```

衍生点云使用 Hilti Research 的公开 Hilti-Oxford 2022 资产，按其 CC BY-NC-SA 3.0 许可归档。
