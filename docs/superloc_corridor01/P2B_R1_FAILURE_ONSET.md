# PAPER-P2B-R1 prefix-anchored failure onset

## Definition

误差序列使用 PREFIX_10S 的固定 SE(3) 对齐，时间原点为评估起点 `1517157224.188979`。超过阈值并连续至少 5 s 才记为 persistent crossing。NDT 是主序列，EKF 结论一致。

| threshold | first crossing (s) | first persistent (s) | duration above (s) | max after (m) |
|---:|---:|---:|---:|---:|
| 0.5 m | 11.2375 | 11.2375 | 263.5312 | 184.6813 |
| 1.0 m | 13.1537 | 13.1537 | 261.6150 | 184.6813 |
| 2.0 m | 16.4819 | 16.4819 | 258.2868 | 184.6813 |
| 5.0 m | 25.1553 | 25.1553 | 248.5040 | 184.6813 |

以 2 m persistent crossing 作为主候选 onset：约评估起点后 `16.48 s`（sensor time 约 `1517157240.67`）。对应最近采样：

- onset 前约 5 s：translation error `0.5668 m`，rotation `3.1876 deg`，NDT fitness `6.56394`，iterations `1`，converged `1`。
- onset 前约 2 s：translation error `1.3234 m`，rotation `5.0055 deg`，NDT fitness `0.015804`，iterations `1`，converged `1`。
- onset 采样：translation error `2.0061 m`，rotation `5.8573 deg`，NDT fitness `0.012892`，iterations `1`，converged `1`。

fitness 单独不能作为正确性的判据：错误轨迹样本仍可能有较小 fitness，说明“收敛”与“落在正确地图位置”不是同一件事。

## Initial tracking check

PREFIX_10S 在评估开始后的前 10 s：mean `0.1424 m`、P95 `0.2929 m`、max `0.3490 m`；前 5 s max `0.2091 m`。因此初始化后的短时 tracking 是可信的。之后约 11.24 s 开始越过 0.5 m，并持续增长；这不是 full-trajectory alignment 造成的起点假象。

作为对照，PREFIX_30S 对齐在前 10 s 就有 mean `1.4995 m`，表明它已被长序列污染，不能用于 onset。

## Classification

`INPUT_DETERMINISM=PASS`，`NDT_DETERMINISM=PASS`，PREFIX-10S 又显示前 10 s 正常、随后出现持续增长。因此当前证据支持：

**FAILURE CLASSIFICATION = C — frozen baseline starts valid, then naturally degrades on Corridor01.**

这只说明 frozen baseline 的真实长时定位失败已被可信时间/输入协议暴露；它不说明原因已经是某个具体算法模块，也不授权本阶段修改初始化或 NDT。
