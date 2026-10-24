# R23 必要验证与真实限制

- Release实际构建成功；原始编译日志在logs。现有causal_pose_prediction_test及frame_conversion_contract_test通过，launch XML与参数展开通过。
- 独立源码/结果复核已核对：真实IMU来源、anchor边界、SE3杆臂转换、因果TP、原生PCL align、拒绝不发布测量、真实OOSM、参考后置。未开展额外候选搜索或参数试错。
- 20帧Smoke、10条正式运行及20帧GUI的原始日志/收据保存；正式170份封存产物逐项SHA复核通过。正式C++、冻结策略头及三份二进制仍匹配FORMAL_CODE_FREEZE。
- 现有EKF/传播/Deskew/地图管理源文件未改，稳定工程快照与起始逐项相等；未修改旧研究归档。
- 保留三个限制：历史TP落后最近地图校正一个revision；W1/C3优化前几何门拒绝全部60帧并产生IMU漂移；首次RViz EOL弹窗人工关闭，最终抑制变量重启试验没有完成回执。
- 曾出现输入准备脚本预期稳定工程存在local_map_tiles.cpp，实际该路径缺失；原失败保存在input_prepare_failed_01，修正为记录受保护缺失状态，没有生成或修改稳定工程文件。
- Smoke输入完成后监督进程没有自动退出，已明确人工仅停止自身ROS会话并保留回执；后续监督器检测输入完成并仅关闭自己创建的ROS树，正式十条无需重跑。
- 以上通过只能支持受限工程闭环，不能将D未触发描述为其安全/创新性通过。参考为条件性Base、地图配准Benchmark，Coverage不是独立留出。

原始ROS stdout和rosnode info保留尾随空格/末尾空行，因此全归档`git diff --check`有原始日志格式告警；不为消除告警篡改原始输出。研究源码与launch的diff检查通过。
