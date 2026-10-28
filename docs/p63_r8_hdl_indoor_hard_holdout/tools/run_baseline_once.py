#!/usr/bin/env python3
"""One original HDL Hard trial, raw5–164. Low quality never controls native feedback."""
import csv,hashlib,json,os,signal,socket,subprocess,sys,threading,time
from pathlib import Path
import numpy as np,psutil,rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from hdl_localization.msg import ScanMatchingStatus
from data import message,sha
A=Path(__file__).resolve().parents[1];W=A.parents[1]
sys.path.insert(0,str(W/'docs/p63_r4_hdl_full_loop2/tools'))
from contract import CsvTail
D=Path('/home/jian/livox_ws/p63_r8_indoor_hard_holdout_data/baseline_01')
F=json.loads((A/'INPUT_FREEZE.json').read_text());B=json.loads((A/'BASELINE_BUILD.json').read_text())
assert [f['raw_ordinal'] for f in F['frames']]==list(range(5,165))
assert sha(F['map_path'])==F['map_sha256'] and sha(B['trace_binary'])==B['trace_binary_sha256']
assert sha(B['trace_source'])==B['trace_source_sha256']
for p,v in B['native_binaries'].items():assert sha(p)==v['sha256']
assert os.statvfs(D.parent).f_bavail*os.statvfs(D.parent).f_frsize>=20*1024**3
D.mkdir(exist_ok=False)
with socket.socket() as sock:assert sock.connect_ex(('127.0.0.1',11648))!=0
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11648',ROS_IP='127.0.0.1')
env=os.environ.copy();env.update(ROS_HOME=str(D/'ros_home'),ROS_LOG_DIR=str(D/'ros_logs'),LIBGL_ALWAYS_SOFTWARE='1')
setup='source /opt/ros/noetic/setup.bash && source /home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/devel/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && export ROS_PACKAGE_PATH='+str(W/'research')+':${ROS_PACKAGE_PATH:-} && '
command=['bash','--noprofile','--norc','-c',setup+'exec roslaunch p63_hdl_indoor_hard p63_r8_indoor_hard.launch output_dir:='+str(D)+' rviz:=true autoplay:=false']
(D/'COMMAND.json').write_text(json.dumps({'argv':command,'CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'input_manifest_sha256':sha(A/'INPUT_FREEZE.json'),'trace_binary_sha256':sha(B['trace_binary']),'test_GT_access':False,'oracle_anchor':4,'target_raw':[5,164]},indent=2)+'\n')
start=time.monotonic();log=(D/'ROSLAUNCH.log').open('x');launch=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
mutex=threading.Lock();odoms={};statuses={};inputs={};maps=[];errors=[];captures=[]
expected={f['stamp_ns']:f for f in F['frames']}
raw=(D/'ROS_RECEIPTS.jsonl').open('x')
def record(kind,ns,payload):
    with mutex:raw.write(json.dumps({'kind':kind,'stamp_ns':ns,'payload':payload},allow_nan=True)+'\n');raw.flush()
def odom(m):
    p=m.pose.pose.position;q=m.pose.pose.orientation;v=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    with mutex:odoms[m.header.stamp.to_nsec()]=v
    if m.header.frame_id!='map' or m.child_frame_id!='depth_camera_link':errors.append('OUTPUT_FRAME_MISMATCH')
    record('odom',m.header.stamp.to_nsec(),v)
def status(m):
    value={'converged':bool(m.has_converged),'inlier_fraction':float(m.inlier_fraction),'matching_error':float(m.matching_error)}
    with mutex:statuses[m.header.stamp.to_nsec()]=value
    record('status',m.header.stamp.to_nsec(),value)
def points(m):
    try:
        ns=m.header.stamp.to_nsec();frame=expected[ns]
        xyz=np.frombuffer(m.data,dtype='<f4').reshape(-1,4)[:,:3]
        assert m.header.frame_id==frame['frame_id'] and m.width*m.height==frame['points']
        assert hashlib.sha256(xyz.tobytes()).hexdigest()==frame['xyz_sha256']
        with mutex:inputs[ns]=True
    except Exception as e:errors.append('INPUT_RECEIPT_ERROR:'+repr(e))
def globalmap(m):
    offsets={f.name:f.offset for f in m.fields}
    dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in ('x','y','z')],'itemsize':m.point_step})
    array=np.frombuffer(m.data,dtype=dt,count=m.width*m.height);xyz=np.column_stack([array[k] for k in ('x','y','z')])
    entry={'points':len(xyz),'xyz_sha256':hashlib.sha256(xyz.tobytes()).hexdigest(),'frame':m.header.frame_id}
    if m.header.frame_id!='map' or not 0<len(xyz)<=F['map_points']:errors.append('MAP_INPUT_INVALID')
    maps.append(entry);(D/'PUBLISHED_MAP.json').write_text(json.dumps(maps,indent=2)+'\n')
stop=threading.Event();peaks={'manager_KiB':0,'whole_owned_tree_KiB':0}
def monitor():
    own=psutil.Process()
    with (D/'MEMORY.csv').open('x',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['wall_s','manager_KiB','whole_owned_tree_KiB'])
        while not stop.wait(.05):
            try:
                manager=0;total=own.memory_info().rss//1024
                for p in own.children(recursive=True):
                    try:
                        rss=p.memory_info().rss//1024;total+=rss;args=p.cmdline()
                        if 'manager' in args and any('p63_hdl_manager' in a for a in args):manager+=rss
                    except (psutil.NoSuchProcess,psutil.AccessDenied):pass
                peaks['manager_KiB']=max(peaks['manager_KiB'],manager);peaks['whole_owned_tree_KiB']=max(peaks['whole_owned_tree_KiB'],total)
                writer.writerow([time.monotonic()-start,manager,total]);stream.flush()
            except Exception as e:errors.append('MONITOR_ERROR:'+repr(e));return
thread=threading.Thread(target=monitor,daemon=True);thread.start()
native=CsvTail(D/'NATIVE_NDT.csv');callback=CsvTail(D/'CALLBACKS.csv')
sent=completed=0;reason='UNFINISHED';records=[]
def capture(tag):
    try:
        p=next(p for p in psutil.Process(launch.pid).children(recursive=True) if p.name()=='rviz')
        helper='/home/jian/livox_ws/p63_r1_super_lio_baseline_ws/docs/p63_r1_mature_prior_map_baseline/tools/capture_rviz.py'
        r=subprocess.run(['/usr/bin/python3',helper,'--pid',str(p.pid),'--output',str(D/(tag+'.png'))],capture_output=True,timeout=10)
        captures.append({'tag':tag,'returncode':r.returncode})
    except Exception as e:captures.append({'tag':tag,'error':repr(e)})
try:
    import rosgraph
    deadline=time.monotonic()+30
    while not rosgraph.Master('/p63_r8').is_online():
        if time.monotonic()>deadline:raise RuntimeError('ROS_STARTUP_TIMEOUT')
        time.sleep(.1)
    rospy.init_node('p63_r8_recorder',disable_signals=True)
    rospy.Subscriber('/odom',Odometry,odom,queue_size=20);rospy.Subscriber('/status',ScanMatchingStatus,status,queue_size=20)
    rospy.Subscriber('/p63/koide_points',PointCloud2,points,queue_size=1,buff_size=16777216)
    rospy.Subscriber('/globalmap',PointCloud2,globalmap,queue_size=1,buff_size=16777216)
    pub=rospy.Publisher('/p63/koide_points',PointCloud2,queue_size=1)
    while not maps or pub.get_num_connections()<2:
        if time.monotonic()>deadline or launch.poll() is not None:raise RuntimeError('MAP_OR_SUBSCRIBER_STARTUP_FAILED')
        time.sleep(.05)
    assert not errors and maps[0]['points']==F['target_points'] and maps[0]['xyz_sha256']==F['target_xyz_sha256']
    params={name:rospy.get_param('/'+name) for name in ('hdl_localization_nodelet','globalmap_server_nodelet')}
    p=params['hdl_localization_nodelet'];g=params['globalmap_server_nodelet']
    assert p['reg_method']=='NDT_OMP' and p['ndt_resolution']==1.0 and p['ndt_neighbor_search_method']=='DIRECT7'
    assert not p['use_imu'] and not p['use_global_localization'] and not p['enable_robot_odometry_prediction']
    assert p['odom_child_frame_id']=='depth_camera_link' and p['downsample_resolution']==.1
    assert not g['convert_utm_to_local'] and g['downsample_resolution']==.1 and g['globalmap_pcd']==F['map_path']
    (D/'EFFECTIVE_PARAMS.json').write_text(json.dumps(params,indent=2)+'\n')
    time.sleep(.5)  # Let the latched map callback finish; original sensor stamps are unchanged.
    for f in F['frames']:
        begin=time.monotonic();m=message(f);load_ms=(time.monotonic()-begin)*1000;ns=f['stamp_ns']
        pub.publish(m);sent+=1;nr=cr=None;deadline=time.monotonic()+30
        while True:
            if nr is None:nr=native.poll()
            if cr is None:cr=callback.poll()
            with mutex:ready=ns in odoms and ns in statuses and ns in inputs
            if ready and nr is not None and cr is not None:break
            if errors or launch.poll() is not None or time.monotonic()>deadline:raise RuntimeError('RECEIPT_FAILURE:'+repr(errors))
            time.sleep(.01)
        assert int(nr['stamp_ns'])==int(cr['stamp_ns'])==ns and int(cr['ndt_calls_so_far'])==sent
        assert nr['map_frozen']=='1' and nr['target_xyz_sha256']==maps[0]['xyz_sha256'] and int(nr['target_points'])==maps[0]['points'] and nr['source_xyz_sha256']==f['source_xyz_sha256'] and int(nr['source_points'])==f['source_points']
        with mutex:v=odoms.pop(ns);s=statuses.pop(ns);inputs.pop(ns)
        guess=[float(nr['guess_'+str(i)+str(j)]) for i in range(4) for j in range(4)]
        final=[float(nr[k]) for k in ('tx','ty','tz','qx','qy','qz','qw')]
        finite=bool(np.isfinite(v+guess+final).all())
        row={'raw_ordinal':f['raw_ordinal'],'stamp_ns':ns,'relative_s':(ns-F['anchor_stamp_ns'])/1e9,
             'input_points':f['points'],'source_points':int(nr['source_points']),'converged':nr['converged'],
             'fitness_all_nn_m2':nr['fitness_all_nn_m2'],'native_probability':nr['probability'],
             'inlier_fraction':s['inlier_fraction'],'native_inlier_error_m2':s['matching_error'],
             'iterations':nr['iterations'],'ndt_compute_ms':nr['ndt_compute_ms'],'callback_ms':cr['callback_ms'],
             'diagnostic_ms':nr['diagnostic_ms'],'input_load_convert_ms':load_ms,'peak_RSS_KiB':nr['peak_RSS_KiB'],
             'source_xyz_sha256':nr['source_xyz_sha256'],'target_xyz_sha256':nr['target_xyz_sha256'],'target_points':nr['target_points'],'core_finite':finite,**dict(zip(('px','py','pz','qx','qy','qz','qw'),v)),
             **{'raw_'+k:nr[k] for k in ('tx','ty','tz','qx','qy','qz','qw')},
             **{k:nr[k] for k in nr if k.startswith('guess_')}}
        records.append(row);completed+=1
        with (D/'ONLINE_RESULTS.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader();writer.writerows(records)
        if not finite:reason='NONFINITE_CORE_STATE';break
        if errors:raise RuntimeError(repr(errors))
        if os.statvfs(D).f_bavail*os.statvfs(D).f_frsize<20*1024**3:raise RuntimeError('DISK_RESERVE')
        if sent in (58,72,127,160):print('R8_PROGRESS',json.dumps({k:row[k] for k in ('raw_ordinal','converged','inlier_fraction','fitness_all_nn_m2')}),flush=True)
        if sent in (151,160):capture('LIVE_raw'+str(f['raw_ordinal']))
    if reason=='UNFINISHED':reason='COMPLETED_160'
except Exception as e:reason='ENGINEERING_EXECUTION_ERROR:'+repr(e)
finally:
    if reason!='COMPLETED_160' and completed:capture('STOP_raw'+str(records[-1]['raw_ordinal']))
    if launch.poll() is None:
        os.killpg(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
    stop.set();thread.join(timeout=2);native.close();callback.close();raw.close();log.close()
    receipt={'reason':reason,'published':sent,'completed':completed,'target':160,'last_raw':records[-1]['raw_ordinal'] if records else None,
             'wall_s':time.monotonic()-start,'peaks':peaks,'errors':errors,'captures':captures,'maps':maps,
             'test_GT_access':False,'GT_access_online':'Previously frozen raw4 anchor only','formal_trials':1,
             'map_file_sha256_after':sha(F['map_path'])}
    (D/'RUN_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n')
    names=['ONLINE_RESULTS.csv','NATIVE_NDT.csv','CALLBACKS.csv','ROS_RECEIPTS.jsonl','MEMORY.csv','PUBLISHED_MAP.json','EFFECTIVE_PARAMS.json','COMMAND.json','RUN_RECEIPT.json']
    seal={name:sha(D/name) for name in names if (D/name).exists()}
    (D/'ONLINE_SEAL.json').write_text(json.dumps({'files':seal,'test_reference_values_read':False},indent=2)+'\n')
    print('R8_SEALED',json.dumps(receipt),flush=True)
