# P50 协议与停止记录

## 冻结目标

任务仅验证 SuperLoc Floor01 上冻结的生产 innovation trigger 与 Anchor 资格；不做任何候选精修、GT 评价或定位精度判断。触发门槛维持 `translation > 0.12 m OR rotation > 3 deg`。

## 输入准入优先级

1. 确认实际 Floor01 raw bag、固定地图、标定和已有合法初始化来源。
2. 验证原始 Velodyne packet 连续序列及 decoder/time 合同；确认可提供算法所需点级 firing time。
3. 验证 IMU 输入时间覆盖、坐标系和外参来源。
4. 只有以上合同通过，才运行 20 帧 Smoke。
5. Smoke 后执行 CONTROL Record-Off 与 Record-Only parity；通过后才允许一次最多 1,500 帧正式记录。

## 本轮实际执行

输入检查在第 2 项失败：原始 bag 和 timed-point 索引/二进制均缺失。P6 归档的 XYZ 二进制只有 XYZ 三个 float32 分量/点，虽然另有每扫描 timestamp，但没有每点 firing time；不能重建 VLP-16 packet 解码和点内运动去畸变合同。按预注册停止规则，本轮未继续 Smoke。

## 非执行项目

- Smoke：0 帧；正式采集：0 帧。
- CONTROL parity：未运行。
- 运行时预测、Anchor 与 trigger：未测量；CSV 保持表头，无伪造事件行。
- GT：未读取。
- NDT align、extra align、nativeJet、dynamicScore、Weak-only、Coupled、Equal-Budget、Recovery、候选反馈：本轮均未调用。
- 源码、配置、输入资产：均未修改；P6 资产仅只读核验。

## 源码兼容性待办（不是本轮算法改动）

当前 P49 Record-Only 构造检查要求 Local Target manager；Floor01 本轮批准的开发链允许 Global Target。恢复输入后，需要最小兼容性处理，并保留同一 CONTROL 测量输出合同。另需在 P50 专用配置中启用已有 IMU Deskew；不能沿用 R9B 文件中显式关闭 Deskew 的设置来宣称满足本任务。该代码/配置工作需先做独立定向测试，再进入 P50 Smoke。
