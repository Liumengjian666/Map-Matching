# 数学与输出语义边界

本轮无原创数学。作者`Encoder::Encode`进行地面对齐、高度切片关键点提取
和Token生成；`Matcher::Shortlist`采用MINT，`Matcher::BeamScore`采用BEAM；
`inlier::Verify`按Token对应进行2点RANSAC及2D刚体SVD、z残差中位数估计，
再由作者地面对齐变换返回传感器相对SE(3)。之后的原版GICP属于单独阶段。

作者源码明确：

```text
T_sensor = inverse(T_ground_reference) * T_aligned * T_ground_query
p_reference = T_sensor * p_query
```

不是`map_T_lidar`。本轮不使用历史map_T_reference、未闭合物理外参或GT去
补足地图位姿。原始扫描各自在sensor frame，参考关键帧来自独立Loop2建图
采集，查询来自Loop3；不把它们变换进地图后再错误地当作本地扫描。

默认参数：高度0–20m、10层，因此每层2m。作者关键点高度变化门限
`0.5*(z_max-z_min)/N_h`=1m；每1m BEV单元需满足该门限及局部极大值。
这个公式未修改。稀疏/低结构单帧可有大量原始点但很少结构关键点。
查询19/24 MINT没有正分、所有480 Verify对应点不足，是本次数据条件下的
实测失败机制，不是全局配准方法在所有传感器上的否定。

非空Token只证明编码产生数据；MINT正分不证明位置正确；BEAM航向估计
来自相似性移位，不是独立真实航向标签；Verify成功也不等于定位正确。
失败单位阵与Inf残差必须保留失败状态，不能把有限矩阵误计为成功位姿。
本轮没有真实Verify成功，不能评价成功几何精度或以失败早退时间证明加速。

多帧输入不是本轮已经完成的能力。若下一轮获准，必须仅用当前及此前扫描、
真实合法相对位姿和明确时间参考，不能沿用作者离线未来帧累积去冒充在线。
不实现这一新输入协议、不开发描述子、不改NDT或滤波器。
