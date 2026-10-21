# R11 reproduction

## 已执行的历史命令（不要再次运行原生实验）

```bash
python3 /home/jian/livox_ws/r10_stable_preservation_20261021/object_export_retry_01/retry.py
python3 /home/jian/livox_ws/p62_r11_cause_aware_recovery_ws/docs/p62_r11_cause_aware_recovery/control.py build
python3 /home/jian/livox_ws/p62_r11_cause_aware_recovery_ws/docs/p62_r11_cause_aware_recovery/control.py run --resume-preflight
```

实际argv、来源和二进制SHA在raw/freeze.json及BUILD_PROVENANCE.json。
run原先因20GiB预检失败，尚未调用原生程序；其STARTED与PRE_NATIVE_BLOCKED已保留。
--resume-preflight只接受当时严格的零调用状态，现在目录已有formal/freeze/log/continuation，
因此会拒绝再次运行。不要删除令牌、改目录或重新调用原生程序。
构建历史：warm重编译、frozen_build_01（预检前）、最终独占frozen_build_02；均只有零-align测试。

## 封存后允许的只读结果复核

```bash
cd /home/jian/livox_ws/p62_r11_cause_aware_recovery_ws
python3 docs/p62_r11_cause_aware_recovery/analyze.py --self-test
cd docs/p62_r11_cause_aware_recovery
sha256sum -c checksums.sha256
```

封存前曾执行 `python3 docs/p62_r11_cause_aware_recovery/analyze.py`，只读取本次冻结结果及固定R7输入身份，不执行NDT/EKF；更新派生results与两份诊断CSV。
实际数值分析版本见results.json的analysis_CODE_SHA。封存后不要直接执行这个写出模式覆盖本归档；复算需在独立输出副本中进行，并保留原件。
最终封存seal.py仅哈希、比较和归档，不能被当作再次运行算法。
新工作树稀疏配置保留全部依赖，不需下载或检出其他历史大文件。
没有roslaunch/rosbag play命令，因为本轮不是ROS定位验收。
