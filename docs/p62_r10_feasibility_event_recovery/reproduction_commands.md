# R10 reproduction boundaries

## 已完成的一次真实运行（历史命令，不要再次执行）

```bash
python3 /home/jian/livox_ws/p62_r10_feasibility_recovery_ws/docs/p62_r10_feasibility_event_recovery/build.py
python3 /home/jian/livox_ws/p62_r10_feasibility_recovery_ws/docs/p62_r10_feasibility_event_recovery/run_experiment.py
```

冷构建实际源码591c7fe0081dfaea0e3db19a07d1a8df5cf65ae1，运行233152227ab9d0614cb98b4ec9e091ebd60c17e5。实际argv在raw/run_freeze.json；运行输出目录已存在，程序拒绝覆盖。不要删除运行令牌、修改输出路径或重跑。没有roslaunch/rosbag play命令，因为本轮是独立C++ Shadow，不是ROS定位。

## 允许的纯离线重算（不调用NDT/EKF）

```bash
cd /home/jian/livox_ws/p62_r10_feasibility_recovery_ws
python3 docs/p62_r10_feasibility_event_recovery/analyze.py --self-test
python3 docs/p62_r10_feasibility_event_recovery/analyze.py
cd docs/p62_r10_feasibility_event_recovery
sha256sum -c checksums.sha256
```

analyze.py仅读取本次冻结输出及固定R7输入身份，不重新读取旧R9实验归档。重算会更新自身派生results/局部表，需在对应分析源码提交上执行以保持analysis_CODE_SHA一致；不要把新派生文件冒充原生运行结果。seal.py执行最终输入/生产文件身份复核及清单生成，不运行算法。checksums不包含自身，analysis_manifest不自哈希；最终END_SHA由Git归档历史确定。
