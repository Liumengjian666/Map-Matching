# 复现与审计

在 `/home/jian/p64_r3_q5_ws` 执行下列单行命令。脚本读取既有传感资产及本地固定的参考源码，结果写入全新目录。

```bash
bash /home/jian/p64_r3_q5_ws/docs/p64_r3_q5_integrated_baseline_v1/run_experiment.sh /home/jian/p64_r3_q5_ws/run/reproduction_01
```

实际运行按脚本同顺序分条执行。最终运行原始目录 `run/final`，构建在 `run/final_build`；逐组执行采用 `run_v1.sh`，验证后才开启主动 `ACTIVE`。单独核对实际结果使用以下命令。

```bash
python3 /home/jian/p64_r3_q5_ws/docs/p64_r3_q5_integrated_baseline_v1/check_gate.py /home/jian/p64_r3_q5_ws/run/final/w2_A /home/jian/p64_r3_q5_ws/run/final/w2_SHADOW
```

评估读取簿记及封存的定位输出后独立读取参考。实际工作站为 ROS1 Noetic、系统 PCL 1.10、CloudCompare 2.10.3。编译和外部执行参数均在 `build.sh`、`build_reference.sh`、`run_v1.sh`、`run_experiment.sh`；参考来源单列。图 `trajectory_comparison.png` 来自实际轨迹，狗窗没有参考轨迹线。原始目录含每帧完整 UKF 协方差、地图观测前后一致时标与候选 CSV。复现依赖的大型栅格地图和真实扫描仍由本机实验资产保管。
