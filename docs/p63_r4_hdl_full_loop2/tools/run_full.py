#!/usr/bin/env python3
"""One frozen full-loop2 trial. Reuses every R3 native binary and exact input bridge."""
import csv,hashlib,io,json,math,os,signal,socket,subprocess,threading,time
from pathlib import Path
import numpy as np,psutil,rosbag,rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg
from hdl_localization.msg import ScanMatchingStatus
from contract import CsvTail,assess
A=Path(__file__).resolve().parents[1];W=A.parents[1]
C=Path('/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws');D=Path('/home/jian/livox_ws/p63_r4_hdl_full_loop2_data/full_trial_01')
identity=json.loads((A/'INPUT_IDENTITY.json').read_text());frames=list(csv.DictReader((A/'INPUT_FRAMES.csv').open()))
assert identity['status']=='PASS_FRESH_ALL_LIDAR_PREFLIGHT' and len(frames)==10905
assert hashlib.sha256((A/'INPUT_FRAMES.csv').read_bytes()).hexdigest()==identity['frames_manifest_sha256']
assert os.statvfs(D.parent).f_bavail*os.statvfs(D.parent).f_frsize>=20*1024**3
for p,x in json.loads((W/'docs/p63_r3_hdl_ndt_candidate/BUILD_AND_SOURCE_FREEZE.json').read_text())['binaries'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==x['sha256']
D.mkdir(exist_ok=False)
with socket.socket() as s:assert s.connect_ex(('127.0.0.1',11644))!=0
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11644',ROS_IP='127.0.0.1')
env=os.environ.copy();env.update(ROS_HOME=str(D/'ros_home'),ROS_LOG_DIR=str(D/'ros_logs'),LIBGL_ALWAYS_SOFTWARE='1')
setup='source /opt/ros/noetic/setup.bash && source '+str(C/'devel/setup.bash')+' && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && export ROS_PACKAGE_PATH='+str(W/'research')+':${ROS_PACKAGE_PATH:-} && '
command=['bash','--noprofile','--norc','-c',setup+'exec roslaunch p63_hdl_full_loop2 p63_r4_hdl_loop2.launch output_dir:='+str(D)+' rviz:=true']
(D/'COMMAND.json').write_text(json.dumps({'argv':command,'R4_diagnostic_CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'R3_native_CODE_SHA':'eb6ff37ab791193ff1a803952d79e148b9ecea6c','target_frames':10905,'GT_online':False,'selected_OMP_environment':{k:os.environ.get(k) for k in ('OMP_NUM_THREADS','OMP_DYNAMIC','OMP_SCHEDULE')},'no_timing_thread_override':True},indent=2)+'\n')
start=time.monotonic();log=(D/'ROSLAUNCH.log').open('x');launch=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
stop_sample=threading.Event();peaks={'manager_KiB':0,'rviz_KiB':0,'owned_descendants_KiB':0,'whole_owned_tree_KiB':0};sample_errors=[]
def sample():
    own=psutil.Process();f=(D/'MEMORY.csv').open('x',newline='');w=csv.writer(f,lineterminator='\n');w.writerow(['wall_s',*peaks,'input_recorder_KiB'])
    while not stop_sample.wait(.05):
        try:
            root_rss=own.memory_info().rss//1024;desc=manager=rviz=0
            for p in own.children(recursive=True):
                try:
                    rss=p.memory_info().rss//1024;args=p.cmdline();desc+=rss
                    if any('p63_hdl_manager' in x for x in args) and 'manager' in args:manager+=rss
                    if p.name()=='rviz':rviz+=rss
                except (psutil.NoSuchProcess,psutil.AccessDenied):pass
            values=[manager,rviz,desc,desc+root_rss]
            for k,v in zip(peaks,values):peaks[k]=max(peaks[k],v)
            w.writerow([time.monotonic()-start,*values,root_rss]);f.flush()
        except Exception as e:sample_errors.append(repr(e));return
    f.close()
thread=threading.Thread(target=sample,daemon=True);thread.start()
mutex=threading.Lock();odoms={};statuses={};maps=[];map_errors=[]
rosf=(D/'ROS_RECEIPTS.jsonl').open('x')
def log_ros(kind,ns,payload):
    with mutex:rosf.write(json.dumps({'kind':kind,'stamp_ns':ns,'payload':payload},allow_nan=True)+'\n');rosf.flush()
def odom(m):
    p=m.pose.pose.position;q=m.pose.pose.orientation;v=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    with mutex:odoms[m.header.stamp.to_nsec()]=v
    log_ros('odom',m.header.stamp.to_nsec(),{'frame_id':m.header.frame_id,'child_frame_id':m.child_frame_id,'pose_xyzw':v})
def status(m):
    errors={}
    for label,tf in zip(m.prediction_labels,m.prediction_errors):
        errors[label.data]=[tf.translation.x,tf.translation.y,tf.translation.z,tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w]
    payload={'converged':bool(m.has_converged),'matching_error':float(m.matching_error),'inlier_fraction':float(m.inlier_fraction),'prediction_errors':errors}
    with mutex:statuses[m.header.stamp.to_nsec()]=payload
    log_ros('status',m.header.stamp.to_nsec(),payload)
def globalmap(m):
    offsets={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in ('x','y','z')],'itemsize':m.point_step});v=np.frombuffer(m.data,dtype=dt,count=m.width*m.height);xyz=np.stack([v[k] for k in ('x','y','z')],axis=1)
    entry={'points':len(v),'XYZ_sha256':hashlib.sha256(xyz.tobytes()).hexdigest(),'frame_id':m.header.frame_id}
    with mutex:maps.append(entry)
    if entry!={'points':424323,'XYZ_sha256':'d4ae17e75e22b2c660df1ca08140ad3675479c79f214da6253706c239ca4325e','frame_id':'map'}:map_errors.append(entry)
    (D/'PUBLISHED_MAP.json').write_text(json.dumps(maps,indent=2)+'\n')
nt=CsvTail(D/'NATIVE_NDT.csv');ct=CsvTail(D/'CALLBACKS.csv');at=CsvTail(D/'ADAPTER.csv')
output=(D/'CAUSAL_RESULTS.csv').open('x',newline='');writer=csv.writer(output,lineterminator='\n')
columns=['ordinal_1based','stamp_ns','relative_s','native_converged','UKF_output_receipt','finite','external_valid','invalid_run','jump_run','step_translation_m','step_rotation_deg','native_fitness_m2','native_inlier_error_m2','inlier_fraction','UKF_pred_to_NDT_translation_m','UKF_pred_to_NDT_rotation_deg','pred_delta_finite','px','py','pz','qx','qy','qz','qw','raw_tx','raw_ty','raw_tz','raw_qx','raw_qy','raw_qz','raw_qw','ndt_compute_ms','callback_ms','diagnostic_ms','bridge_ms','iterations','source_points','manager_getrusage_peak_KiB']
writer.writerow(columns);output.flush()
sent=completed=valid_count=converged=bad_run=bad_max=jump_run=0;previous=[0,0,0,0,0,0,1];reason='UNFINISHED';captures=[];first_pose=last_pose=None;input_wall_ms=[]
# GUI screenshots read only our owned RViz. Failures do not alter estimator state or validity.
def capture(tag):
    try:
        p=next(p for p in psutil.Process(launch.pid).children(recursive=True) if p.name()=='rviz')
        helper='/home/jian/livox_ws/p63_r1_super_lio_baseline_ws/docs/p63_r1_mature_prior_map_baseline/tools/capture_rviz.py'
        result=subprocess.run(['/usr/bin/python3',helper,'--pid',str(p.pid),'--output',str(D/(tag+'.png'))],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=10)
        captures.append({'tag':tag,'returncode':result.returncode,'output':result.stdout.decode(errors='replace')[:300]})
    except Exception as e:captures.append({'tag':tag,'error':repr(e)})
try:
    deadline=time.monotonic()+30
    import rosgraph
    while not rosgraph.Master('/p63_r4_controller').is_online():
        if time.monotonic()>deadline:raise RuntimeError('ROS_STARTUP_TIMEOUT')
        time.sleep(.1)
    rospy.init_node('p63_r4_recorder',disable_signals=True)
    rospy.Subscriber('/odom',Odometry,odom,queue_size=100);rospy.Subscriber('/status',ScanMatchingStatus,status,queue_size=100);rospy.Subscriber('/globalmap',PointCloud2,globalmap,queue_size=1,buff_size=33554432)
    pub=rospy.Publisher('/livox/lidar',CustomMsg,queue_size=1)
    while not maps or pub.get_num_connections()!=1:
        if launch.poll() is not None or time.monotonic()>deadline:raise RuntimeError('MAP_OR_BRIDGE_NOT_READY')
        time.sleep(.05)
    if map_errors:raise RuntimeError('R3_TARGET_IDENTITY_MISMATCH')
    params={k:rospy.get_param('/'+k) for k in ('hdl_localization_nodelet','globalmap_server_nodelet','p63_livox_bridge')}
    old=json.loads(Path('/home/jian/livox_ws/p63_r3_hdl_data/trial_01/EFFECTIVE_PARAMS.json').read_text())
    assert params['hdl_localization_nodelet']==old['hdl_localization_nodelet'] and params['globalmap_server_nodelet']==old['globalmap_server_nodelet']
    (D/'EFFECTIVE_PARAMS.json').write_text(json.dumps(params,indent=2)+'\n')
    with rosbag.Bag(identity['bag'],'r') as bag:
        for _,m,_ in bag.read_messages(topics=['/livox/lidar']):
            frame_start=time.monotonic()
            if sent>=10905:raise RuntimeError('UNREGISTERED_EXTRA_INPUT')
            exp=frames[sent];ns=m.header.stamp.to_nsec();payload=io.BytesIO();m.serialize(payload)
            if ns!=int(exp['stamp_ns']) or hashlib.sha256(payload.getvalue()).hexdigest()!=exp['raw_sha256']:raise RuntimeError('INPUT_IDENTITY_MISMATCH')
            pub.publish(m);sent+=1;frame_deadline=time.monotonic()+30;nr=cr=ar=None
            while True:
                if nr is None:nr=nt.poll()
                if cr is None:cr=ct.poll()
                if ar is None:ar=at.poll()
                with mutex:ready=ns in odoms and ns in statuses
                if nr is not None and cr is not None and ar is not None and ready:break
                if launch.poll() is not None:raise RuntimeError('LOCALIZER_EXITED')
                if time.monotonic()>frame_deadline:raise RuntimeError('CALLBACK_OR_TRACE_TIMEOUT')
                if sample_errors:raise RuntimeError('MEMORY_MONITOR_FAILED')
                if os.statvfs(D).f_bavail*os.statvfs(D).f_frsize<20*1024**3:raise RuntimeError('DISK_RESERVE')
                time.sleep(.01)
            if not all(int(x['stamp_ns'])==ns for x in (nr,cr,ar)) or int(cr['ndt_calls_so_far'])!=sent:raise RuntimeError('DIAGNOSTIC_IDENTITY_MISMATCH')
            if ar['raw_message_sha256']!=exp['transport_seq_sha256'] or ar['xyz_sha256']!=exp['xyz_sha256'] or int(ar['points'])!=int(exp['points']):raise RuntimeError('ADAPTER_IDENTITY_MISMATCH')
            with mutex:v=odoms.pop(ns);s=statuses.pop(ns)
            raw=[float(nr[k]) for k in ('tx','ty','tz','qx','qy','qz','qw')]
            finite,valid,trans,rot=assess(v,raw,nr['converged']=='1',s['converged'],float(nr['fitness_all_nn_m2']),s['matching_error'],s['inlier_fraction'],previous)
            error=s['prediction_errors'].get('motion_model');pred_finite=error is not None and bool(np.isfinite(error).all())
            pred_t=float(np.linalg.norm(error[:3])) if pred_finite else float('nan')
            pred_R=float(np.degrees(2*np.arccos(np.clip(abs(error[6])/np.linalg.norm(error[3:]),0,1)))) if pred_finite and np.linalg.norm(error[3:])>0 else float('nan')
            completed+=1;valid_count+=int(valid);converged+=int(nr['converged']=='1');bad_run=0 if valid else bad_run+1;bad_max=max(bad_max,bad_run);jump_run=jump_run+1 if finite and (trans>2 or rot>20) else 0
            rel=(ns-identity['first_lidar_header_ns'])/1e9
            writer.writerow([sent,ns,rel,nr['converged'],1,finite,valid,bad_run,jump_run,trans,rot,nr['fitness_all_nn_m2'],s['matching_error'],s['inlier_fraction'],pred_t,pred_R,pred_finite,*v,*raw,nr['ndt_compute_ms'],cr['callback_ms'],nr['diagnostic_ms'],ar['bridge_ms'],nr['iterations'],nr['source_points'],nr['peak_RSS_KiB']]);output.flush()
            previous=v;first_pose=v if first_pose is None else first_pose;last_pose=v;input_wall_ms.append((time.monotonic()-frame_start)*1000)
            if nr['map_frozen']!='1' or nr['target_xyz_sha256']!='d4ae17e75e22b2c660df1ca08140ad3675479c79f214da6253706c239ca4325e' or int(nr['target_points'])!=424323 or map_errors:raise RuntimeError('TARGET_IDENTITY_CHANGED')
            if not finite:reason='NONFINITE_STATE';break
            if bad_run>=20:reason='TWENTY_CONSECUTIVE_EXTERNAL_INVALID';break
            if jump_run>=3:reason='THREE_CONSECUTIVE_LARGE_STEPS';break
            if error is None or not pred_finite or not math.isfinite(pred_R):raise RuntimeError('PREDICTION_DIAGNOSTIC_INVALID')
            if sample_errors:raise RuntimeError('MEMORY_MONITOR_FAILED')
            if os.statvfs(D).f_bavail*os.statvfs(D).f_frsize<20*1024**3:raise RuntimeError('DISK_RESERVE')
            if completed%250==0 or bad_run>=5:
                print('P63_R4_PROGRESS',json.dumps({'frame':completed,'relative_s':rel,'converged':converged,'valid':valid_count,'bad_run':bad_run,'fitness':float(nr['fitness_all_nn_m2']),'inlier':s['inlier_fraction'],'position':v[:3],'wall_s':time.monotonic()-start},allow_nan=True),flush=True)
            if sent in (20,6201,10905):capture('LIVE_'+str(sent))
        if sent==10905 and reason=='UNFINISHED':reason='COMPLETED_FULL_10905'
        elif reason=='UNFINISHED':raise RuntimeError('PREMATURE_INPUT_EOF')
except Exception as e:
    reason='EXECUTION_CONTRACT_ERROR:'+repr(e)
finally:
    # Capture a failure view without replaying a scan or altering estimator state.
    if reason!='COMPLETED_FULL_10905' and completed:capture('STOP_'+str(completed))
    if launch.poll() is None:
        os.killpg(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
    stop_sample.set();thread.join(timeout=2)
    for t in (nt,ct,at):t.close()
    output.close();rosf.close();log.close()
    if reason=='COMPLETED_FULL_10905':verdict='FULL_SEQUENCE_ENGINEERING_PASS'
    elif reason in ('NONFINITE_STATE','TWENTY_CONSECUTIVE_EXTERNAL_INVALID','THREE_CONSECUTIVE_LARGE_STEPS'):verdict='FULL_SEQUENCE_TRACKING_FAIL'
    else:verdict='EXECUTION_CONTRACT_FAIL'
    receipt={'verdict':verdict,'reason':reason,'target_frames':10905,'published_scans':sent,'complete_callbacks':completed,'native_converged':converged,'external_valid':valid_count,'longest_external_invalid_run':bad_max,'first_output':first_pose,'last_output':last_pose,'wall_s':time.monotonic()-start,'RSS_peaks':peaks,'monitor_errors':sample_errors,'captures':captures,'map_messages':maps,'map_errors':map_errors,'new_imu_calls':0,'formal_trials':1,'GT_read':False,'last_processed_relative_s':((int(frames[completed-1]['stamp_ns'])-identity['first_lidar_header_ns'])/1e9 if completed else None),'per_frame_handshake_wall_ms':input_wall_ms}
    (D/'RUN_RECEIPT.json').write_text(json.dumps(receipt,indent=2,allow_nan=True)+'\n')
    print('P63_R4_FINAL',json.dumps({k:v for k,v in receipt.items() if k not in ('per_frame_handshake_wall_ms','map_messages','captures')},allow_nan=True),flush=True)
