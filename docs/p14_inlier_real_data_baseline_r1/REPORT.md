# P14 — InLiER真实单帧基线

TASK: P14-INLIER-REAL-DATA-BASELINE-R1.
FINAL_RESULT: **INLIER_CPP_FUNCTIONAL_SINGLE_SCAN_GEOMETRY_NOT_CERTIFIED**.
NEXT（唯一）: **DECIDE_CAUSAL_ACCUMULATION_INPUT_PROTOCOL**.

作者原版C++核心编译、API烟测及坐标方向测试通过；真实389参考扫描和
24查询扫描完成一次固定实验。单帧结构支持不足，480次Verify全部在对应点
数量门限前退出，没有取得真实相对位姿。不能认定已替代DCReg，也不能据此
启动定位反馈。结束本轮，不调描述子参数、不自行优化作者算法。

## Git与源码

- Repository: Liumengjian666/Map-Matching。
- Branch: research/p9-r4-heldout-visual-evidence。
- START_SHA及实际重新查询的REMOTE_HEAD:
  `f3a996da317a08bad0886f14dc499c0d24031ce6`。
- 实际编译运行CODE_SHA: `294e6fd540922793540bcd2d4120d727c8973d89`。
- END_SHA为随后归档提交的完整SHA，在文本交接中回传；不自引用提交哈希。
- PUSH_EXECUTED=NO；本地提交不是远端上传。隔离副本：
  `/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`。
- 只新增`src/dog_prior_map_localization/scripts/p14_inlier_baseline/`及本目录。
  NDT/EKF/OOSM/SOLiD/DCReg/R6、主论文脏工作区及稳定机器狗工作区均未修改。

直接链接外部作者库，未复制或重新实现MINT、BEAM、Token、RANSAC及几何数学：

```cpp
encoded = encoder.Encode(points);
matcher.Add(reference_id, encoded.second.token_id);
matcher.Finalize();
mint = matcher.Shortlist(query_tokens, mintConfig(), -1, -1);
beam = matcher.BeamScore(query_tokens, mint.ids, beamConfig(), -1, -1);
v = inlier::Verify(query_tokens, query_keypoints,
                   reference_tokens, reference_keypoints,
                   beam.best_shifts[i], grid, verifyConfig());
```

上游https://github.com/LTU-RAI/InLiER，实际获取SHA:
`515759c00fe1e2f6d4d4a005e8c3a6b3f71d17ac`，MIT，外部工作树clean。
README声明v1.1.0，根CMake项目版本0.2.0；以实际commit为准。
使用`cpp/inlier_core/src/{encoder,matcher,verify,plane,shape_pca}.cpp`和
原始头文件。DIRECT_AUTHOR_CPP_STATIC_LIBRARY，无Python算法fallback。
GICP=NOT_RUN，rerank=NOT_RUN，完整点云精配准调用=0。

## 冻结协议与测试

使用作者官方default.yaml，不使用更宽松的库Verify结构默认值。
0.5m首点体素、保留原顺序、float32、去全零/非有限点；不用NDT1400点裁剪。
MINT=前10%（39/389），BEAM=20，Verify=20；32对应点、16RANSAC内点、
8关键点内点、1m阈值、500次RANSAC，seed0，OpenMP1线程。未改门限。
首批固定16/45/124/708/1285/1891；1891出现正MINT和BEAM，因此按事前
规则继续其余18帧，没有替换样本。389参考只编码一次。

Release build PASS；作者合成场景API烟测PASS；原版预处理逐点/顺序对照PASS
（边界9→4点，实际reference0为17699→559点）；确定种子Verify重复PASS。
合成变换方向误差`1.7431442023888599e-15`。

保留了两项开发错误记录，不隐藏：

1. 系统相机SDK的LD_LIBRARY_PATH优先加载旧libusb，PCL链接缺少
   `libusb_set_option`。独立程序显式链接系统libusb，仅该进程使用系统库路径，
   未升级ROS/Python或修改作者代码。原失败和修复后构建日志均保留。
2. 最初烟测错误地要求原Token自匹配严格等于单位阵。作者使用首索引对应，
   重复Token会产生多对一关系；原自匹配Verify成功但矩阵误差0.004921117。
   保留此数值。精确方向单元测试改用唯一(h,r,a)合成对应，门限仍为1e-8；
   此筛选只用于合成单元测试，真实输入、Token和验证规则完全不变。

这只是API烟测及必要接口单元测试，不是作者完整pytest套件通过。

## 真实数据结果

Loop2独立建图参考389帧，Loop3原P12固定查询24帧；所有实际输入与历史
SHA一致。PCD逐帧流式读取。未重新提取、下载数据或播放ROS bag。

| 指标 | 实测 |
|---|---:|
| 参考非空Token帧 |315/389|
| 参考空Token帧 |74/389|
| 参考Token总数 |2265|
| 查询非空Token帧 |19/24|
| 查询空Token帧 |5/24|
| 查询Token总数 |195|
| 查询Token数量达到32 |1/24|
| 同时有正MINT和BEAM分数的查询 |5/24|
| MINT返回ID / 正分候选 |936 /195|
| BEAM返回ID / 正分候选 |480 /323|
| 正MINT来源且正BEAM的候选 |100|
| Verify调用 / 成功 |480 /0|
| 实际进入RANSAC或几何拟合 |0|
| 真实合法相对位姿 |0|

5个有效检索查询ID：1891、1994、2011、4691、4881。
BEAM在18帧有正分，但其中部分来自MINT零分填充ID，不能把18/24写成
有效位置识别率。这里的5/24也只是有正分检索证据，不是正确检索率。

原始查询点数均值20008，作者体素后均值877.5（370–2895），Token中位数4、
最大51；参考Token中位数3、最大60。查询高度活跃层中位数1，官方MINT需要
3层；这是19帧MINT全零的直接门限证据。所有候选对应点0–16，不满足32。
因此480/480 fail_stage=1，RANSAC没有启动，几何残差不可用。
失败的默认单位阵不是位姿估计，Inf残差是作者失败哨兵，不能计为成功。
无非有限成功位姿；成功位姿为0，不据此宣称成功配准数值稳定性已验证。

真实ground_Rzz存在较大变化（参考最小0.001535、查询最小0.480108），但没有
独立地面标签，不能证明作者地面拟合错误或传感器轴错误。本轮不改地面算法。
数据在作者0–20m高度/1m局部高度变化准入下结构支持不足；不能外推为所有
MID-360输入必然不适用。未运行多帧累积，也未伪造合法连续累积姿态。

## 成本与内存

单位ms，24个查询，单线程；P95按numpy线性百分位计算。

| 阶段 | mean | P95 | max |
|---|---:|---:|---:|
| PCD读取 |0.416455|0.917892|1.146462|
| 原版等价预处理 |0.917591|2.067490|3.115665|
| Encoder |0.903163|1.741731|3.871988|
| MINT |0.033251|0.112055|0.126833|
| BEAM |6.510499|8.838251|11.906717|
| Verify（每查询20次总和） |0.043538|0.079970|0.087163|
| 计算阶段总和 |8.408042|12.998464|15.781218|
| 完整查询链（含读盘/诊断，不含CSV写入） |8.989249|13.535352|17.026507|
| 完整查询CPU |8.960250|13.506050|17.004000|

启动前哈希读入使OS文件缓存变暖；这些不是冷盘I/O基准。离线389参考数据库
生成13.853071s；索引构建0.740607ms；完整子进程含离线建库14.088787s。
参考PCD为ASCII而查询为binary，其I/O成本不同。
无本轮GICP耗时可报，GICP=NOT_RUN。

峰值RSS21648KiB=21.140625MiB；序列化Token/关键点DB128512字节
（0.122559MiB）。同进程驻留RSS增量5244KiB=5.121094MiB，含编码器工作区及
分配器保留，不能当作精确专属数据库分配大小。Matcher索引RSS增量2108KiB，
仅其稠密直方图就占2178400字节；序列化文件大小不等于在线内存。
原始389扫描未同时驻留。机器AMD Ryzen7 5800H，Ubuntu20.04.6，GCC9.4，
CMake3.16.3，PCL1.10，作者依赖nanoflann v1.6.2。

P12/P12B三候选DCReg430–455ms只能参考，不能计算严格加速比：不同候选池、
点数处理、参考对象（整张地图 vs参考扫描关键点）、失败阶段都不同。本轮
Verify全部早退；8.99ms不是成功几何验证耗时，也不是低成本替代证明。

## 科研判断

- 原版核心能运行，单帧能产生少量Token及部分检索候选。
- 原版配置在本固定MID-360单帧样本上未形成任何真实几何验证结果。
- 没有独立正确性标签，不计算位置正确率、精度或RMSE。
- 不推荐直接替代DCReg或接入EKF；只能保留为可执行的负基线。
- 唯一下一步建议：科研总控决定是否值得冻结一套小规模、合法因果多帧
  输入协议，以检验“单帧结构支持不足”这一具体原因；本轮不自动执行。

GT_LOADED=NO；ACCURACY_GAIN_NOT_EVALUATED。所有实验已结束。
