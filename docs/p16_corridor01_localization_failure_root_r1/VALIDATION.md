# 验证与审查记录

## 已执行

1. Python几何/代数测试：PASS。覆盖0.5m/5deg原限幅、90deg外参旋转及
   杠杆臂方向、下一初值正反方向、5cm修正导致10cm初值变化及0.3m/s
   速度差分变化的解析案例。
2. `python3 -m py_compile`：PASS。
3. 冻结数据离线分析：退出码0。396个完整事件组/分支，692个评价扫描
   行、9个反馈行；两条实际high-rate主窗口均6396行，RMSE与历史重现
   差<1e-9m。用到的GT/外参SHA与原归档一致。
4. 实际used IMU位姿与校正节点值相符，但分别按自身时间评价。
5. 下一NDT初值重建max差：Control 5.031845355e-8m，Weak-only
   5.546384622e-8m。归一化CSV四元数和原PCL float矩阵的表示差已说明；
   仅使用float表示保护，不改原Control/Shadow重复性容差。
6. CSV/JSON结构、非有限值和本轮必要artifact哈希检查：见audit.json。
7. `git diff --check`在提交前通过。

## 独立只读审查

`p16_time_frame_review`发现的可操作项：

- NDT初值平移来自自身上一增量，不是EKF平移预测。
- Weak-only raw→used含候选替换，不能全算step limit。
- NDT_CORRECTION_APPLIED日志先于速度混合，不能假称已观测postblend v。
- 395/396 rollback节点不等于测量时刻，最大4.981041ms。
- 缺少完整release时刻pre-state，不能从callback早期状态替代。
- 两条反馈历史319/396点云及初值分化，不是逐帧同输入消融。

全部已落实为阶段/时间标签、同历史代数比较、结论边界；没有改定位代码。

`p16_analysis_review`核对坐标/限幅/velocity_blend=.6、z_apply_ratio=1
及下一初值公式和实际归档，未发现实质公式错误或越界运行。

用户已选择“仅当前独立审查，继续执行”。没有运行任何外部模型CLI。
收尾文本核对发现audit.json引用尚待生成及一处五位小数舍入差；均在
最终提交前补齐/修正。没有追加实验或改变结论。

## 本轮自身分析错误与修复

- 第一次将启动GT覆盖假定得过宽，校验拒绝最早启动样本；修复为只分析
  原0–35秒评价范围，不插值到覆盖外，不改变定位输入。
- 初值重建曾用1e-8m过严表示断言；核对PCL float矩阵→CSV归一化旋转
  后，用float32机器精度派生保护并保留实测差值。不是放宽实验容差。
- 首次commit因隔离副本缺少Git身份失败；只对本次commit采用起点提交
  已有的`Codex Execution AI <codex-execution@local.invalid>`，未修改全局配置。

不需要Release重建或ROS测试：定位源码、CMake、算法与运行配置没有修改。
本轮配准/ROS调用为零。没有用文档验证替代新的算法实验；任务本身是
对既有真实实验的有界根因分析。
