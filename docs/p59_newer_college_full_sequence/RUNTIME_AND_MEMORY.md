# 运行性能与内存

P59未启动ROS定位，因此NDT/EKF/Publisher VmHWM、CPU时间、对齐耗时、Deskew耗时、Tile更新、ACK延迟、积压及实时性均为 `NOT_MEASURED`。转存耗时不能用作定位运行性能。

## 构建身份

在P59独立worktree基于HEAD `a93d41d46ac60adc04552d76a09861e08f7ed87d` 编译 `dog_prior_map_localization` 成功。NDT、EKF和Publisher二进制SHA256：

| Binary | P59 build | P46 frozen run |
|---|---|---|
| NDT | `f872f6cd317c1e0ae0790987479aad4245badc6e7a39ab5468282708a2d36dda` | `c5e028637a38fb8877486c9c7ef783e40de071e61c981b874d82db8bfcbb266c` |
| EKF | `fa699364c28589e298e24e8d1aaf3963544ee0f9ea02ba5b0c5e42aeb3a18401` | `930a3ca96e178d63afd58874ca3974dadcbd87908e58e39a3d15c85807d25d2d` |
| sequence publisher | `c4fe6e00a1f2ce3f2402b6d2d50526f1b19a8810f4d566de605c8fb1cd4b4fe3` | `024b8f679c16a325acb73bf6b4e91698f4a7a025dd47ed90de3cfb6fef565e66` |

三者二进制都与P46不相同。P59没有运行，也没有声称这些重编译二进制与P46逐位或逐帧等价。P46只作为冻结参考输入；前1800帧正式输出比较未执行。

构建环境需注意：先source `/opt/ros/noetic/setup.bash`，再source `/home/jian/livox_ws/devel/setup.bash` 的只读Livox依赖overlay，再在P59 worktree构建。第一次未source overlay的构建因缺少 `livox_ros_driver2` 失败；补source overlay后目标包构建成功。launch节点名展开通过：NDT、EKF、Newer College sequence publisher。没有启动节点。
