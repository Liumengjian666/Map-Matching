# 实际必要验证

## 构建

独立CMake配置和Release编译均退出0。
GNU9.4、PCL1.10、Eigen3.3.7；只构建annotated_test，不构建或运行定位节点。
CMake已有VTK工具缺失及可选PCAP/PNG警告不影响本PCD离线程序成功链接和执行。

## 合成守卫自测

实际执行一次：

```bash
env PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 src/dog_prior_map_localization/scripts/p32_annotated_pose/run_once.py --self-test
```

退出0，执行端观察到的stdout如下（本段是观察记录，不冒充重定向生成的原始日志文件）：

```text
ANNOTATED_GUARD_TEST_PASS first_full_block=PASS no_rotation_mix=PASS homogeneous=PASS proper_rotation=PASS finite=PASS
```

合成夹具验证首个完整注释块优先、旋转/平移不混用，拒绝错误齐次行、
反射、非正交与NaN。它不读取GT/真实点云，不增加实验候选或NN调用。

## 真实单次运行

`runs/annotated_once/execution.json`退出0；ledger初始NN1、地图ICP1。
matrix_audit原矩阵合法；initial_nn预筛通过；map_icp/final_nn最终准入失败。
独立验证、地图归一化及定位没有执行。无额外数据重建或参数修正。
最终旋转容差未通过被如实保留，不修改输出矩阵或阈值。
