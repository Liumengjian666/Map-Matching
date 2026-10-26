#!/usr/bin/env python3
import os,subprocess,json,time,socket
from pathlib import Path
P=Path(__file__).resolve().parents[1];U=Path('/home/jian/livox_ws/p63_r1_super_lio_data/Super-LIO');D=U.parent/'map_probe_01'
D.mkdir(exist_ok=False);env=os.environ.copy();env.update(ROS_MASTER_URI='http://127.0.0.1:11631',ROS_IP='127.0.0.1',ROS_HOME=str(D/'ros_home'),ROS_LOG_DIR=str(D/'ros_logs'))
with socket.socket() as s:assert s.connect_ex(('127.0.0.1',11631))!=0
cmd=['bash','--noprofile','--norc','-c','source /opt/ros/noetic/setup.bash && source '+str(U/'devel/setup.bash')+' && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && exec roslaunch p63_super_lio_baseline map_freeze_probe.launch receipt:='+str(D/'PROBE.json')]
(D/'COMMAND.json').write_text(json.dumps(cmd,indent=2)+'\n')
with (D/'ROSLAUNCH.log').open('x') as f:
 p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 try:code=p.wait(timeout=90)
 except subprocess.TimeoutExpired:
  os.killpg(p.pid,2);p.wait(timeout=20);raise
assert code==0 and (D/'PROBE.json').exists();receipt=json.loads((D/'PROBE.json').read_text());assert receipt['target_before_sha256']==receipt['target_after_sha256']
(P/'evidence/MAP_FREEZE_PROBE.json').write_text(json.dumps(receipt,indent=2)+'\n');print(receipt)
