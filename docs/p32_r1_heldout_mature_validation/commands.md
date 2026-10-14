# 实际检查的复现命令（不是未执行的F/G命令）

仅只读检查；所有timeout用于限制外盘元数据/小文件读取。
P32没有roscore、roslaunch、rosbag play或NDT align命令。

```bash
git -C /home/jian/livox_ws/p10_corridor01_oosm_r1_ws status --short
git -C /home/jian/livox_ws/p10_corridor01_oosm_r1_ws rev-parse HEAD
git -C /home/jian/livox_ws/p10_corridor01_oosm_r1_ws branch --show-current
git -C /home/jian/livox_ws/p10_corridor01_oosm_r1_ws ls-remote origin refs/heads/research/p9-r4-heldout-visual-evidence
timeout 15s find '/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01' '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02' -maxdepth 2 -type f -printf '%p\t%s bytes\n'
timeout 15s find '/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/raw' -maxdepth 4 -type f -printf '%p\t%s bytes\n'
sha256sum /home/jian/livox_ws/p6_i1_recovery_assets_20260927/floor01_h1_map_p5_frozen.pcd
source /opt/ros/noetic/setup.bash
timeout 30s rosbag info --yaml '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02.bag'
```

实际首条消息检查：

```bash
source /opt/ros/noetic/setup.bash
timeout 25s python3 - <<'PY'
import json,rosbag
p='/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/raw/corridor02.bag'
with rosbag.Bag(p) as b:
 seen=set()
 for topic,msg,t in b.read_messages(topics=['/camera_1/image_raw','/imu/data','/velodyne_points']):
  if topic in seen: continue
  seen.add(topic)
  r=dict(topic=topic,record_ns=t.to_nsec(),header_ns=msg.header.stamp.to_nsec(),frame=msg.header.frame_id)
  if topic=='/camera_1/image_raw': r.update(width=msg.width,height=msg.height,encoding=msg.encoding,bytes=len(msg.data))
  if topic=='/velodyne_points': r.update(points=msg.width*msg.height,fields=[dict(name=f.name,offset=f.offset,type=f.datatype) for f in msg.fields])
  print(json.dumps(r),flush=True)
  if len(seen)==3: break
PY
```

标定/初始文件哈希（已执行）：

```bash
timeout 15s sha256sum '/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml' '/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_intrinsics.yaml' '/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/initial_pose/multi-floor01.yaml' '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/calibration/corridor02_extrinsics.yaml' '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/calibration/corridor02_intrinsics.yaml' '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor02/initial_pose/corridor02.yaml'
```

归档检查：

```bash
python3 /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p32_r1_heldout_mature_validation/verify_archive.py --seal
git -C /home/jian/livox_ws/p10_corridor01_oosm_r1_ws diff --check
```

没有已通过准入的场景，不提供假“完整复现实验回放命令”。
外部CLI精确命令/限定输入已经单独确认，实际仅执行一次并退出0。
精确命令及文件哈希见external_review/execution.json。输入与输出原文保留；
不与ROS实验混称。该记录不是新的调用授权，未经新授权不得再次执行。
