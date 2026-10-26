#!/usr/bin/env python3
"""Single registered prefix through pristine HDL, with external input/measurement only."""
import csv,hashlib,io,json,math,os,signal,socket,subprocess,sys,threading,time
from pathlib import Path
import numpy as np,psutil,rosbag,rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg
from hdl_localization.msg import ScanMatchingStatus
A=Path(__file__).resolve().parents[1];W=A.parents[1]
C=Path('/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws');D=C.parent/'trial_01'
D.mkdir(exist_ok=False)
identity=json.loads((A/'WINDOW_IDENTITY.json').read_text())
assert os.statvfs(D).f_bavail*os.statvfs(D).f_frsize>=20*1024**3
with socket.socket() as s:assert s.connect_ex(('127.0.0.1',11633))!=0
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11633';os.environ['ROS_IP']='127.0.0.1'
env=os.environ.copy();env.update(ROS_HOME=str(D/'ros_home'),ROS_LOG_DIR=str(D/'ros_logs'),LIBGL_ALWAYS_SOFTWARE='1')
setup='source /opt/ros/noetic/setup.bash && source '+str(C/'devel/setup.bash')+' && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && '
cmd=['bash','--noprofile','--norc','-c',setup+'exec roslaunch p63_hdl_candidate p63_hdl_loop2.launch output_dir:='+str(D)+' rviz:=true']
(D/'COMMAND.json').write_text(json.dumps({'argv':cmd,'CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'GT_online':False,'scan_handshake':'one original scan,wait full prior callback,then next scan'},indent=2)+'\n')
log=(D/'ROSLAUNCH.log').open('x');launch=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
start=time.monotonic();samples=[];stop_sample=threading.Event();manager_peak=tree_peak=rviz_peak=0
# Includes this input/recorder supervisor and every owned ROS descendant, conservatively.
def sample():
 global manager_peak,tree_peak,rviz_peak
 own=psutil.Process();f=(D/'MEMORY.csv').open('x',newline='');w=csv.writer(f,lineterminator='\n');w.writerow(['wall_s','manager_KiB','rviz_KiB','whole_owned_tree_KiB'])
 while not stop_sample.wait(.05):
  procs=[own]+own.children(recursive=True);total=manager=rviz=0
  for p in procs:
   try:
    rss=p.memory_info().rss//1024;args=p.cmdline();total+=rss
    if any('p63_hdl_manager' in x for x in args) and 'manager' in args:manager+=rss
    if p.name()=='rviz':rviz+=rss
   except (psutil.NoSuchProcess,psutil.AccessDenied):pass
  manager_peak=max(manager_peak,manager);rviz_peak=max(rviz_peak,rviz);tree_peak=max(tree_peak,total)
  w.writerow([time.monotonic()-start,manager,rviz,total]);f.flush()
 f.close()
thread=threading.Thread(target=sample,daemon=True);thread.start()
odoms={};statuses={};maps=[];reason='UNFINISHED';sent=0;valid_count=0;bad_run=0;bad_max=0;jump_run=0;records=[];capture=None
rawf=(D/'ROS_RECEIPTS.csv').open('x',newline='');rw=csv.writer(rawf,lineterminator='\n');rw.writerow(['stamp_ns','kind','values'])
def odom(m):
 odoms[m.header.stamp.to_nsec()]=m
 p=m.pose.pose.position;q=m.pose.pose.orientation
 rw.writerow([m.header.stamp.to_nsec(),'odom',json.dumps([p.x,p.y,p.z,q.x,q.y,q.z,q.w])]);rawf.flush()
def status(m):
 statuses[m.header.stamp.to_nsec()]=m
 rw.writerow([m.header.stamp.to_nsec(),'status',json.dumps([m.has_converged,m.matching_error,m.inlier_fraction])]);rawf.flush()
def globalmap(m):
 offsets={f.name:f.offset for f in m.fields};dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in ('x','y','z')],'itemsize':m.point_step})
 a=np.frombuffer(m.data,dtype=dtype,count=m.width*m.height);xyz=np.stack([a[k] for k in ('x','y','z')],axis=1)
 maps.append({'points':len(a),'xyz_sha256':hashlib.sha256(xyz.tobytes()).hexdigest(),'frame_id':m.header.frame_id,'xyz_min':xyz.min(0).tolist(),'xyz_max':xyz.max(0).tolist()})
 (D/'PUBLISHED_MAP.json').write_text(json.dumps(maps,indent=2)+'\n')
def rows(path):
 if not path.exists():return []
 with path.open() as f:return list(csv.DictReader(f))
def stop_owned():
 if launch.poll() is None:
  os.killpg(launch.pid,signal.SIGINT)
  try:launch.wait(timeout=20)
  except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
try:
 deadline=time.monotonic()+30
 while not rospy.core.is_initialized():
  try:
   import rosgraph
   if rosgraph.Master('/p63_controller').is_online():rospy.init_node('p63_trial_recorder',disable_signals=True);break
  except Exception:pass
  if time.monotonic()>deadline:raise RuntimeError('ROS_STARTUP_TIMEOUT')
  time.sleep(.1)
 rospy.Subscriber('/odom',Odometry,odom,queue_size=100);rospy.Subscriber('/status',ScanMatchingStatus,status,queue_size=100);rospy.Subscriber('/globalmap',PointCloud2,globalmap,queue_size=1,buff_size=33554432)
 publisher=rospy.Publisher('/livox/lidar',CustomMsg,queue_size=1)
 while not maps or publisher.get_num_connections()!=1:
  if time.monotonic()>deadline or launch.poll() is not None:raise RuntimeError('MAP_OR_BRIDGE_NOT_READY')
  time.sleep(.05)
 params={n:rospy.get_param('/'+n) for n in ('hdl_localization_nodelet','globalmap_server_nodelet','p63_livox_bridge')}
 assert params['hdl_localization_nodelet']['use_imu'] is False
 assert params['hdl_localization_nodelet']['use_global_localization'] is False
 assert params['hdl_localization_nodelet']['enable_robot_odometry_prediction'] is False
 assert params['globalmap_server_nodelet']['convert_utm_to_local'] is False
 (D/'EFFECTIVE_PARAMS.json').write_text(json.dumps(params,indent=2)+'\n')
 previous=np.array([0,0,0,0,0,0,1.],float)
 with rosbag.Bag(identity['bag'],'r') as bag:
  for topic,m,record in bag.read_messages(topics=['/livox/lidar']):
   if sent>=400:break
   expected=identity['frames'][sent];ns=m.header.stamp.to_nsec();b=io.BytesIO();m.serialize(b)
   assert ns==expected['stamp_ns'] and hashlib.sha256(b.getvalue()).hexdigest()==expected['raw_message_sha256']
   publisher.publish(m);sent+=1;deadline_frame=time.monotonic()+30
   while True:
    nr=rows(D/'NATIVE_NDT.csv');cr=rows(D/'CALLBACKS.csv')
    if len(nr)>=sent and len(cr)>=sent and ns in odoms and ns in statuses:break
    if launch.poll() is not None:raise RuntimeError('LOCALIZER_EXITED')
    if time.monotonic()>deadline_frame:raise RuntimeError('CALLBACK_OR_TRACE_TIMEOUT')
    time.sleep(.01)
   n=nr[sent-1];c=cr[sent-1];s=statuses[ns];o=odoms[ns]
   assert int(n['stamp_ns'])==ns and int(c['stamp_ns'])==ns and int(c['ndt_calls_so_far'])==sent
   p=o.pose.pose.position;q=o.pose.pose.orientation;v=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
   finite=bool(np.isfinite(v).all() and .99<np.linalg.norm(v[3:])<1.01)
   raw=np.array([float(n[k]) for k in ('tx','ty','tz','qx','qy','qz','qw')]);finite=finite and bool(np.isfinite(raw).all())
   valid=finite and n['converged']=='1' and s.has_converged and math.isfinite(float(n['fitness_all_nn_m2'])) and math.isfinite(s.matching_error) and s.inlier_fraction>=.50
   valid_count+=int(valid);bad_run=0 if valid else bad_run+1;bad_max=max(bad_max,bad_run)
   dt=float(np.linalg.norm(v[:3]-previous[:3])) if finite else float('nan')
   dr=float(np.degrees(2*np.arccos(np.clip(abs(v[3:]@previous[3:])/(np.linalg.norm(v[3:])*np.linalg.norm(previous[3:])),0,1)))) if finite else float('nan')
   jump_run=jump_run+1 if finite and (dt>2 or dr>20) else 0
   records.append({'ordinal_1based':sent,'stamp_ns':ns,'finite':finite,'valid_update_screen':bool(valid),'bad_run':bad_run,'translation_step_m':dt,'rotation_step_deg':dr,'output_pose_xyzw':v.tolist(),'raw_ndt_pose_xyzw':raw.tolist(),'inlier_fraction':float(s.inlier_fraction),'native_matching_error':float(s.matching_error)})
   previous=v
   if n['map_frozen']!='1':reason='TARGET_MUTATION';break
   if not finite:reason='NONFINITE_POSE';break
   if bad_run>=20:reason='TWENTY_CONSECUTIVE_INVALID_UPDATES';break
   if jump_run>=3:reason='THREE_CONSECUTIVE_LARGE_JUMPS';break
   if os.statvfs(D).f_bavail*os.statvfs(D).f_frsize<20*1024**3:reason='DISK_RESERVE';break
   if sent==20:
    # Own RViz window only; failure to capture is recorded, never fabricated.
    rvs=[p for p in psutil.Process(launch.pid).children(recursive=True) if p.name()=='rviz']
    if rvs:
     cap=Path('/home/jian/livox_ws/p63_r1_super_lio_baseline_ws/docs/p63_r1_mature_prior_map_baseline/tools/capture_rviz.py')
     capture=subprocess.run(['/usr/bin/python3',str(cap),'--pid',str(rvs[0].pid),'--output',str(D/'RVIZ.png')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=10).returncode
  else:pass
  if sent==400 and reason=='UNFINISHED':reason='COMPLETED_FIXED_400'
except Exception as exc:
 reason='EXECUTION_ERROR:'+repr(exc)
finally:
 stop_owned();stop_sample.set();thread.join(timeout=2);rawf.close();log.close()
 receipt={'reason':reason,'published_scans':sent,'completed_records':len(records),'valid_update_screen_count':valid_count,'longest_invalid_run':bad_max,'wall_s':time.monotonic()-start,'manager_peak_RSS_KiB_sampled':manager_peak,'rviz_peak_RSS_KiB':rviz_peak,'whole_owned_tree_peak_RSS_KiB_including_input_recorder':tree_peak,'RViz_capture_exit':capture,'map_messages':maps,'records':records,'GT_read':False,'IMU_published_or_used':False,'formal_attempt':1}
 (D/'RUN_RECEIPT.json').write_text(json.dumps(receipt,indent=2,allow_nan=True)+'\n')
 print(json.dumps({k:v for k,v in receipt.items() if k not in ('records','map_messages')},indent=2))
