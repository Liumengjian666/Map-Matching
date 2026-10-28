# 源码、构建与真实运行身份

- Repository: https://github.com/Liumengjian666/Map-Matching
- Branch: `research/p63-r10-hdl-imu-full-hard`
- Worktree: `/home/jian/livox_ws/p63_r10_imu_full_hard_ws`
- START: `6e2b67a6bffa81eb4d70667fdf6e604c56ae4959`；执行前`git ls-remote`核对R9远端分支一致。
- 实际在线CODE: **`4d0ba895314eb80aae191c3bc63a37fa28861b1d`**。输入身份、完整种类/范围、预算、RPE配对与合同均在唯一正式运行前提交。
- 封存后评价CODE: **`d8e211f00c29fb9327227051b6f5f305475bcfd7`**。评价工具提交后首次读取测试参考位姿。
- END为最后本地归档提交，以`git -C /home/jian/livox_ws/p63_r10_imu_full_hard_ws rev-parse HEAD`和最终交接完整SHA为准；文件不包含自身commit哈希。没有push。

## 原版来源与实际二进制

|库|上游Git SHA|
|---|---|
|hdl_localization|de2dc7769aa2412876d05ae6161408d3295b6a3c|
|ndt_omp|5495fd9214945afcb4b35d5a1da385e405c52bf9|
|fast_gicp|0e7ec1441c99f7be453db2ea216d5de029387417|
|hdl_global_localization|a6a6b69929ac371d9531d0dcdef3c1593fd49245|

四个上游源码和6份原生ELF均沿用R9，核心改动0、重编译0。`BUILD.json`包含实际路径、全部SHA、历史Release命令及许可证身份；不要将其中继承的R9 trace编译命令误认为本轮重编译。

本轮只编译零align的`kernel_preflight.cpp`，Release `-O3 -DNDEBUG`，仅把R8输入期望160扩为2375，物理过滤/体素化不变。编译日志`logs/build_preflight.log`；预处理日志`logs/native_preprocess.log`，没有NDT对象/align。其二进制SHA `aa6fee25c8910277984af76861edc4c7341397977f4276456d8b5b2bb17a8d97`。

R9只读trace原文件`docs/p63_r9_hdl_real_imu_prediction/tools/read_only_trace.cpp`、实际ELF`/home/jian/livox_ws/p63_r9_real_imu_prediction_data/libp63_r9_trace.so`完全复用；ELF SHA `e87b8e26cc2d8eb1390291b603a81a7e015c47fc784a523cc8b1b0ee7f7a4fe6`。它调用原IMU callback、原predict、原align及points callback，读取状态/输入/耗时，不改变原UKF或NDT。原NDT ELF SHA `e274416abe31236910b5aa588e095cd3100dd77f348163ffddd29618bb8b82ff`。

## 本轮修改位置与范围

- `tools/imu_geometry.py`：字节等同R9，SHA `19adb542534ebff22ba08ee98a84854dbc809d2295f601e049e49fe9f6570739`。R_L_I、单位、因果角加速度及杆臂公式全部不变。
- `tools/preflight.py`：全段原CDR/PCD/IMU身份、原时钟模型、参考时间戳资格、预冻结RPE配对；不读取测试参考位姿，不执行NDT。
- `tools/run_full_once.py`：R9监督器扩至raw2379，私有ROS master11650、新输出路径；逐帧追加CSV代替反复重写，增加监督周期计时/进度和全段截图。IMU顺序、实参转换、发布握手、原生预测和UKF反馈保持不变。文件存在即拒绝覆盖，正式运行只有一次。
- `research/p63_hdl_imu_full_hard/launch/p63_r10_full_hard.launch`：包含R9原launch，参数/初值不变。绝不能使用云点单独player冒充IMU-on。
- `tools/evaluate_full.py`：在线文件SHA核验后调用冻结R29/R9评价、3帧失锁规则及预注册精确1sRPE；无ROS、NDT、预测或反馈。
- `tools/archive_run.py`、`plot_results.py`、`final_checks.py`：封存、展示和必要证据检查，新增align0。

## 已实际执行的命令

在R10工作区，零NDT预检/编译/目标核验按`logs/preflight.log`、`BUILD.json`及`logs/native_preprocess.log`记录。完整在线命令：

```bash
cd /home/jian/livox_ws/p63_r10_imu_full_hard_ws
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}
/usr/bin/python3 docs/p63_r10_hdl_imu_full_hard/tools/run_full_once.py
```

真实执行使用`> docs/p63_r10_hdl_imu_full_hard/logs/full_01.log 2>&1`。该命令已消耗唯一正式预算，不能再次执行；已存在输出目录会拒绝重跑。实际节点启动完整argv见`raw/full_01/COMMAND.json`。

封存之后执行：

```bash
/usr/bin/python3 docs/p63_r10_hdl_imu_full_hard/tools/evaluate_full.py
/usr/bin/python3 docs/p63_r10_hdl_imu_full_hard/tools/archive_run.py
/usr/bin/python3 docs/p63_r10_hdl_imu_full_hard/tools/plot_results.py
/usr/bin/python3 docs/p63_r10_hdl_imu_full_hard/tools/final_checks.py
```

原生NDT调用逐项清单为`raw/full_01/NATIVE_NDT.csv.gz`的2375行；原IMU调用为`IMU_PREDICT.csv.gz`的44267行，其中41000行active；原callback2375行。所有原始文件gzip为无损封存，外部原文件仍保留，`RAW_MANIFEST.json`记录压缩前后SHA。

## RViz实测与复现边界

以下launch在正式实验中已经由监督器实际启动，地图、当前点云、原UKF Path/TF和RViz自动加载：

```bash
ROS_MASTER_URI=http://127.0.0.1:11650 ROS_IP=127.0.0.1 ROS_PACKAGE_PATH=/home/jian/livox_ws/p63_r10_imu_full_hard_ws/research:$ROS_PACKAGE_PATH roslaunch p63_hdl_imu_full_hard p63_r10_full_hard.launch output_dir:=/home/jian/livox_ws/p63_r10_imu_full_hard_data/full_01 rviz:=true
```

该行展示实际已执行调用，**不要手动重新用封存目录启动**。真实IMU+扫描输入由上面的监督器按原时间顺序握手发送；单独launch不会自动产生输入。新的完整实时回放需下一轮授权及新的输出目录，不属于本轮交付验收。

`raw/full_01/LIVE_raw164.png`、`LIVE_raw1204.png`、`LIVE_raw2379.png`是真实在线RViz截屏，后两张显示严重发散后的失败状态。只提供封存轨迹图不等于重新在线定位，更不等于定位成功。未用GT/候选拼接轨迹。

## 必要核验与提交前源码检查

`logs/imu_regression.log`：旋转方向、无符号反转、向心/切向杆臂项、输入不变和非法dt拒绝测试通过，NDT0/UKF0。
`FINAL_CHECKS.json`及`logs/final_checks.log`：23项必要检查通过，包括正式调用与身份、2373个非空IMU批的最后原生状态确实成为NDT初值、预检时钟与实际消费一致、参考缺口保留、RPE配对冻结、R9前160轨迹逐帧复现、归档解压内容一致、原库/地图/根工作区原444项改动均未被改写、20GiB余量。

提交前检查R9→R10 supervisor差异：仅范围、日志追加方式、输出路径/ROS master、合同校验及计时/截图；IMU转换、原生参数和状态更新无改变。记录/评价/图表与算法隔离；没有merge到生产分支，也没有push。评价脚本固定为本次已完成运行的审计工具，不能拿它替代另一次未完成运行的错误分类。

对代码的正确性、可读性、模块边界、数据保护和成本作有限自查：参考仅在SHA封存之后打开；数据输入及库有固定哈希；输出独占创建；完整失败状态不删；在线采用追加CSV避免随帧数二次方重写；未作无关重构或增加新的依赖/网络服务。只读工具保留冻结研究脚本的紧凑风格，不扩大为通用执行框架。
