#!/usr/bin/env python3
"""One conditional original-framework smoke; owned processes only, no GT evaluation."""
import json,subprocess,sys,os,time,socket,signal,csv
from pathlib import Path
P=Path(__file__).resolve().parents[1];R=P.parents[1];U=Path('/home/jian/livox_ws/p63_r1_super_lio_data/Super-LIO');D=U.parent/'loop2_smoke_01'
sys.path.insert(0,str(R/'docs/p62_r31_observation_identity/tools'));from run_sequence import OwnedTree
D.mkdir(exist_ok=False);env=os.environ.copy();env.update(ROS_MASTER_URI='http://127.0.0.1:11631',ROS_IP='127.0.0.1',ROS_HOME=str(D/'ros_home'),ROS_LOG_DIR=str(D/'ros_logs'),DISPLAY=os.environ.get('DISPLAY',':0'),LIBGL_ALWAYS_SOFTWARE='1')
with socket.socket() as s:assert s.connect_ex(('127.0.0.1',11631))!=0
assert env['DISPLAY'], 'actual GUI display required for this RViz smoke'
setup='source /opt/ros/noetic/setup.bash && source '+str(U/'devel/setup.bash')+' && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && '
argv=['roslaunch','p63_super_lio_baseline','p63_loop2_frozen_map.launch','sensor_contract:='+str(P/'config/SENSOR_CONTRACT.json'),'output_dir:='+str(D),'allow_historical_identity_smoke:=true','rviz:=true']
cmd=['bash','--noprofile','--norc','-c',setup+'exec '+' '.join(argv)];(D/'RUN_COMMAND.json').write_text(json.dumps({'argv':cmd,'git_HEAD':subprocess.check_output(['git','rev-parse','HEAD'],cwd=R,text=True).strip(),'GT_online':False,'sensor_contract':str(P/'config/SENSOR_CONTRACT.json')},indent=2)+'\n')
log=(D/'ROSLAUNCH.log').open('x');proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);owned=OwnedTree(proc.pid);player=None;start=time.monotonic();peak=rviz_peak=tree_peak=0;stop=None;reason='';snap=False
with (D/'PROCESS_MEMORY.csv').open('x',newline='') as f:
 w=csv.writer(f,lineterminator='\n');w.writerow(['wall_s','observer_RSS_KiB','rviz_RSS_KiB','owned_total_RSS_KiB'])
 try:
  while time.monotonic()-start<180:
   now=time.monotonic();live=owned.collect();obs=sum(v['rss_KiB'] for v in live.values() if v['exe_name']=='p63_original_observer');rv=sum(v['rss_KiB'] for v in live.values() if v['exe_name']=='rviz');total=sum(v['rss_KiB'] for v in live.values());peak=max(peak,obs);rviz_peak=max(rviz_peak,rv);tree_peak=max(tree_peak,total);w.writerow([now-start,obs,rv,total]);f.flush()
   if proc.poll() is not None:reason='LOCALIZATION_PROCESS_EXITED';break
   if (D/'TARGET_BEFORE.json').exists() and player is None:
    player_log=(D/'INPUT.log').open('x');playcmd=['bash','--noprofile','--norc','-c',setup+'exec /usr/bin/python3 '+str(P/'tools/replay_prefix.py')+' --bag /home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag --output '+str(D)];player=subprocess.Popen(playcmd,env=env,stdout=player_log,stderr=subprocess.STDOUT,start_new_session=True)
   if not snap and any(v['exe_name']=='rviz' for v in live.values()) and (D/'ORIGINAL_TRAJECTORY.csv').exists() and sum(1 for _ in (D/'ORIGINAL_TRAJECTORY.csv').open())>250:
    subprocess.run(['/usr/bin/python3',str(P/'tools/capture_rviz.py'),'--pid',str(next(pid for pid,v in live.items() if v['exe_name']=='rviz')),'--output',str(D/'RVIZ.png')],env=env,timeout=15,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);snap=True
   if player is not None and player.poll() is not None and stop is None:
    if player.returncode!=0:reason='INPUT_FAILURE';break
    stop=now;reason='COMPLETED_FIXED_PREFIX_DRAIN'
   if stop is not None and now-stop>8:break
   if os.statvfs(D).f_bavail*os.statvfs(D).f_frsize<20*1024**3:reason='DISK_RESERVE';break
   time.sleep(.05)
  else:reason='TIMEOUT'
 finally:
  if player is not None and player.poll() is None:os.killpg(player.pid,signal.SIGINT);player.wait(timeout=20)
  if proc.poll() is None:owned.signal(signal.SIGINT)
  try:proc.wait(timeout=20)
  except subprocess.TimeoutExpired:owned.signal(signal.SIGTERM);proc.wait(timeout=10)
  log.close()
result={'reason':reason,'wall_s':time.monotonic()-start,'roslaunch_exit':proc.returncode,'observer_peak_RSS_KiB':peak,'rviz_peak_RSS_KiB':rviz_peak,'owned_tree_peak_RSS_KiB':tree_peak,'one_prefix_no_retry':True,'reference_evaluation':'NOT_EVALUABLE_NO_INDEPENDENT_GT','extrinsic_status':'USER_AUTHORIZED_HISTORICAL_IDENTITY_SMOKE_ONLY'}
(D/'RUN_RECEIPT.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
