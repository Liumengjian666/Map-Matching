# 实测运行时间与内存

统计对象为唯一400扫描试验。NDT computeTransformation由只读ELF截获测时；完整callback包括原生云转换／下采样、预测、align、UKF correct、status和odom等实际执行路径，以及额外诊断；不隐藏仪表开销。外层callback计时器自己的CSV写入在计时窗口外；输入解码、桥接和显示不在这个回调窗口内，分别或在总墙钟中记账。主机为Ubuntu20.04.6、AMD Ryzen7 5800H，具体软件身份见HOST_MESSAGE_IDENTITY.json和构建冻结。

|范围(ms)|Mean|P95|Max|
|---|---:|---:|---:|
|原生NDT computeTransformation|33.827908|90.752905|166.655089|
|完整points_callback（含诊断）|50.659627|105.994585|181.915786|
|额外fitness／target检查诊断|7.694401|11.340716|17.595438|
|桥接转换／SHA／记录|19.466510|31.876048|39.346040|

NDT compute计时不含首次target准备、Registration::align外层少量开销和诊断NN fitness；不能把它说成完整定位周期。完整callback对这次实际执行路径计时；原live RViz被提示框遮挡，因此不能承诺当时全程存在aligned_points的可视化订阅。第一帧target快照也包含在其callback中。

原始帧的源时间范围40.000008499s，总监督墙钟88.318191524s（含启动、bag解码、Python/ROS输入、同步、截图和结束），约4.52908扫描/墙钟秒。采用逐帧握手保证400帧不丢，不能将其称实时10Hz回放。Mean不足以保证deadline；**实测callback P95未低于100ms，周期筛选FAIL**。不估算“去掉诊断即可达标”，因为未测量该新运行。

|内存范围|KiB|MiB|
|---|---:|---:|
|定位nodelet manager保守实测峰值|153928|150.320313|
|RViz峰值|302496|295.406250|
|完整自有实验树同时峰值|865516|845.230469|

manager包含localizer、map server、UKF/NDT和只读target快照，采样RSS与每次getrusage取较大者；满足暂定200MiB进程筛选。整个实验树包含输入/记录器父进程、ROS master/launch、桥、RViz等，50ms采样同时求和，不是逐进程峰值相加。原试验RViz被提示框遮挡，其内存不是后续完整渲染预览的资源测量。纯ROS子树（排除输入父进程）没有独立字段，不伪造该数字，也不称整机内存。采样峰值可能漏过更短瞬时峰；定位自身getrusage补充在NDT_RESULTS每行。

这只是本机40s数据的资源筛选，不是跨硬件速度优势或论文低内存结论。冻结二进制保留于独立catkin devel，未提交大型编译产物到Git。当前磁盘及保护见evidence/FINAL_CHECKS.json；没有删文件腾空间。
