#!/usr/bin/python3
"""Stream one frozen return window through existing ROS localization nodes."""
import csv,hashlib,json,math,os,shutil,sys,threading,time,xmlrpc.client
from pathlib import Path
import rosbag,rosgraph,rospy,yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath
from sensor_msgs.msg import Imu,PointCloud2
from prepare_inputs import ROOT,REPO,sha
from terminal_contract import terminal
sys.path.insert(0,str(REPO/'docs/p61_r7_geode_continuous_baseline'))
from replay_twenty import connected_edges,new_corrected_at


class GrowingCsv:
    def __init__(self,path):self.path=path;self.offset=0;self.tail='';self.header=None;self.rows=[]
    def poll(self):
        if not self.path.exists():return []
        with self.path.open() as f:f.seek(self.offset);text=f.read();self.offset=f.tell()
        lines=(self.tail+text).split('\n');self.tail=lines.pop();new=[]
        for line in lines:
            fields=next(csv.reader([line]))
            if self.header is None:self.header=fields
            else:
                if len(fields)!=len(self.header):raise RuntimeError('CSV row conflict '+str(self.path))
                row=dict(zip(self.header,fields));self.rows.append(row);new.append(row)
        return new


def main():
    inp=ROOT/'input';run=ROOT/'formal'
    expected=(inp/'manifest.sha256').read_text().strip();assert sha(inp/'manifest.json')==expected
    m=json.loads((inp/'manifest.json').read_text())
    assert m['scan_count']==2441 and m['first_index']==2986 and m['last_index']==5426
    for name,key in [('full_return.bag','bag_sha256'),('scans.csv','scans_sha256'),('imu.csv','imu_sha256'),('initialization.yaml','config_sha256')]:assert sha(inp/name)==m[key],name
    scans=list(csv.DictReader((inp/'scans.csv').open()));imu_manifest=list(csv.DictReader((inp/'imu.csv').open()))
    assert [int(x['scan_index']) for x in scans]==list(range(2986,5427))
    identity=json.loads((ROOT/'preflight_identity.json').read_text())
    assert expected==identity['manifest_sha256']=='4444883c441f0c3c08650bf629c05812d34b1a069c7102d565053a47b976c76d'
    assert sha(REPO/'src/dog_prior_map_localization/config/geode_r7_baseline.yaml')==identity['algorithm_config_sha256']=='9d2e32ebf5bd89230d46b7af7b5d0cf9115cbe72d600634899337eab09eba0d3'
    for path,digest in m['inputs'].items():assert sha(path)==digest,path
    for path,digest in identity['binaries'].items():assert sha(path)==digest
    assert shutil.disk_usage('/home/jian').free>20*1024**3
    rospy.init_node('geode_r8_full_input');lock=threading.RLock();logging=[True]
    assert str(Path(rospy.get_param('/output/geode_fusion_csv')).parent)==str(run)
    def params(d,p=''):
        for k,v in d.items():
            key=p+'/'+k
            if isinstance(v,dict):params(v,key)
            else:assert rospy.get_param(key)==v,key
    params(yaml.safe_load((REPO/'src/dog_prior_map_localization/config/geode_r7_baseline.yaml').read_text()))
    params(yaml.safe_load((inp/'initialization.yaml').read_text()))
    fpose=open(run/'live_trajectories.csv','x',buffering=1);pw=csv.writer(fpose,lineterminator='\n')
    pw.writerow(['stream','stamp_ns','receive_monotonic','px','py','pz','qx','qy','qz','qw','vx','vy','vz'])
    fperf=open(run/'ndt_performance.csv','x',buffering=1);tw=csv.writer(fperf,lineterminator='\n')
    keys=['ndt_converged','align_time_ms','preprocess_time_ms','localization_time_ms','scan_points','map_points','fitness_score','iterations']
    tw.writerow(['stamp_ns']+keys)
    latest_high=[None];bad_state=[False];corrected={};predicted={};used={};diagnosed={}
    prediction_path=RosPath();prediction_path.header.frame_id='camera_init'
    fusion_path=RosPath();fusion_path.header.frame_id='camera_init'
    pp=rospy.Publisher('/geode/prediction_path',RosPath,queue_size=1,latch=True)
    fp=rospy.Publisher('/geode/r8_fused_path',RosPath,queue_size=1,latch=True)
    def record(msg,kind):
        with lock:
            if not logging[0]:return
            p=msg.pose if kind=='PREDICTION' else msg.pose.pose
            xyz=[getattr(p.position,k) for k in 'xyz'];q=[getattr(p.orientation,k) for k in ('x','y','z','w')]
            v=[0,0,0] if kind=='PREDICTION' else [getattr(msg.twist.twist.linear,k) for k in 'xyz']
            good=all(math.isfinite(x) for x in xyz+q+v) and abs(sum(x*x for x in q)-1)<1e-5
            if not good:bad_state[0]=True
            ns=msg.header.stamp.to_nsec();pw.writerow([kind,ns,time.monotonic()]+xyz+q+v)
            if kind=='HIGH_RATE':latest_high[0]=msg
            elif kind=='NDT_USED':used[ns]=msg
            elif kind=='PREDICTION':
                predicted[ns]=msg;prediction_path.header=msg.header;prediction_path.poses.append(msg)
                if len(prediction_path.poses)%10==0 or len(prediction_path.poses)==1:pp.publish(prediction_path)
            elif kind=='CORRECTED':
                corrected[ns]=time.monotonic()
                ps=PoseStamped();ps.header=msg.header;ps.pose=p;fusion_path.header=msg.header;fusion_path.poses.append(ps)
                if len(fusion_path.poses)%10==0 or len(fusion_path.poses)==1:fp.publish(fusion_path)
    def diag(msg):
        with lock:
            if not logging[0]:return
            for s in msg.status:
                if s.name=='dog_prior_map_ndt':
                    d={x.key:x.value for x in s.values};diagnosed[msg.header.stamp.to_nsec()]=d
                    tw.writerow([msg.header.stamp.to_nsec()]+[d.get(k,'') for k in keys])
    subs=[rospy.Subscriber(t,typ,lambda msg,k=k:record(msg,k),queue_size=1000) for t,typ,k in
      [('/dog_livo/odom_high_rate',Odometry,'HIGH_RATE'),('/dog_livo/odom_corrected',Odometry,'CORRECTED'),
       ('/dog_livo/ndt_odom',Odometry,'NDT_USED'),('/geode/scan_end_prediction',PoseStamped,'PREDICTION')]]
    subs.append(rospy.Subscriber('/dog_livo/diagnostics',DiagnosticArray,diag,queue_size=100))
    ip=rospy.Publisher('/imu/data',Imu,queue_size=1000);cp=rospy.Publisher('/geode/raw_points',PointCloud2,queue_size=1)
    def wait(test,label,seconds=30):
        deadline=time.monotonic()+seconds
        while not test():
            if rospy.is_shutdown() or bad_state[0] or time.monotonic()>deadline:raise RuntimeError(label)
            time.sleep(.005)
    wait(lambda:ip.get_num_connections()>=2 and cp.get_num_connections()>=1,'input readiness',180)
    master=rosgraph.Master('/geode_r8_full_input');pids={'publisher':os.getpid()};apis={}
    for node,key in [('/dog_prior_map_ndt','ndt'),('/dog_prior_map_ekf','ekf'),('/geode_r8_rviz','rviz'),('/geode_frozen_map_visualization','map_visualization')]:
        api=xmlrpc.client.ServerProxy(master.lookupNode(node));pids[key]=api.getPid('/geode_r8_full_input')[2]
        if key in ('ndt','ekf'):
            actual=str(Path('/proc/%d/exe'%pids[key]).resolve())
            assert actual in identity['binaries'],actual
            assert sha(actual)==identity['binaries'][actual]
        if key in ('ndt','ekf'):apis[node]=api
    (run/'pids.json').write_text(json.dumps(pids,indent=2))
    edges=[('/dog_prior_map_ekf','/geode/scan_end_prediction','/dog_prior_map_ndt'),
      ('/dog_prior_map_ekf','/geode/deskewed_points','/dog_prior_map_ndt'),
      ('/dog_prior_map_ndt','/dog_livo/ndt_odom','/dog_prior_map_ekf')]
    for node,topic in [('/dog_prior_map_ekf','/geode/scan_end_prediction'),('/dog_prior_map_ekf','/dog_livo/odom_high_rate'),
      ('/dog_prior_map_ekf','/dog_livo/odom_corrected'),('/dog_prior_map_ndt','/dog_livo/ndt_odom'),('/dog_prior_map_ndt','/dog_livo/diagnostics')]:edges.append((node,topic,'/geode_r8_full_input'))
    def ready():
        info={n:a.getBusInfo('/geode_r8_full_input')[2] for n,a in apis.items()}
        ok=connected_edges(info,edges)
        if ok:(run/'connection_readiness.json').write_text(json.dumps(info,indent=2))
        return ok
    wait(ready,'complete connection readiness',60)
    fm=open(run/'memory.csv','x',buffering=1);mw=csv.writer(fm,lineterminator='\n');mw.writerow(['wall_monotonic','process','pid','rss_kib','hwm_kib','utime_ticks','stime_ticks'])
    stop=threading.Event()
    def monitor():
        while not stop.is_set():
            for key,pid in pids.items():
                try:
                    lines=Path('/proc/%d/status'%pid).read_text().splitlines();values={s.split(':')[0]:s.split(':')[1].split()[0] for s in lines if s.startswith(('VmRSS:','VmHWM:'))}
                    stat=Path('/proc/%d/stat'%pid).read_text().rsplit(')',1)[1].split()
                    mw.writerow([time.monotonic(),key,pid,values['VmRSS'],values['VmHWM'],stat[11],stat[12]])
                except FileNotFoundError:bad_state[0]=True
            stop.wait(.01)
    thread=threading.Thread(target=monitor);thread.start()
    readers={n:GrowingCsv(run/(n+'.csv')) for n in ('deskew','ndt_diagnostics','fusion','oosm','ekf_lineage')}
    last_events={};imu_cursor=0;sent_count=0;done=0;started=time.monotonic();failure=None
    active=None;active_written=False;sent=0.;watermark=None
    fr=open(run/'frame_terminals.csv','x',buffering=1);fw=csv.writer(fr,lineterminator='\n')
    fw.writerow(['scan_index','header_ns','end_ns','imu_watermark_ns','imu_sent','terminal','fatal','ack_ms'])
    try:
        with rosbag.Bag(str(inp/'full_return.bag')) as bag:
            imus=[msg for _,msg,_ in bag.read_messages(topics=['/imu/data'])]
            assert len(imus)==len(imu_manifest)==m['imu_count']
            for msg,row in zip(imus,imu_manifest):assert msg.header.stamp.to_nsec()==int(row['stamp_ns'])
            for ordinal,(_,cloud,_),row in zip(range(1,2442),bag.read_messages(topics=['/geode/raw_points']),scans):
                active=row;active_written=False;watermark=None
                end=int(row['end_ns']);sent=0.;status=None;fatal=False
                if ordinal%100==1:assert shutil.disk_usage('/home/jian').free>20*1024**3
                assert cloud.header.stamp.to_nsec()==int(row['stamp_ns']) and sha_data(cloud.data)==row['adapter_sha256']
                while imu_cursor<len(imus):
                    ip.publish(imus[imu_cursor]);imu_cursor+=1
                    if imus[imu_cursor-1].header.stamp.to_nsec()>=end:break
                watermark=imus[imu_cursor-1].header.stamp.to_nsec()
                assert watermark>=end
                wait(lambda:latest_high[0] is not None and latest_high[0].header.stamp.to_nsec()>=watermark,'IMU watermark')
                sent=time.monotonic();cp.publish(cloud);sent_count+=1;deadline=sent+30
                while status is None:
                    for name,reader in readers.items():
                        new=reader.poll()
                        if name=='ekf_lineage':
                            for x in new:last_events[x['event_type']]=x
                    d=readers['deskew'].rows[ordinal-1] if len(readers['deskew'].rows)>=ordinal else None
                    n=readers['ndt_diagnostics'].rows[ordinal-1] if len(readers['ndt_diagnostics'].rows)>=ordinal else None
                    o=next((x for x in reversed(readers['oosm'].rows) if abs(float(x['ndt_stamp'])-end/1e9)<1e-6),None)
                    f=next((x for x in reversed(readers['fusion'].rows) if int(x['measurement_ns'])==end),None)
                    times=True
                    if d is not None:times=d['post_scan_end_imu_used']=='0'
                    if o is not None and o['oosm_result']=='APPLIED':
                        event=last_events.get('NDT_CORRECTION_APPLIED')
                        times=times and event is not None and abs(float(event['ndt_measurement_stamp'])-end/1e9)<1e-6 and float(event['state_stamp'])==float(event['ndt_measurement_stamp'])
                    with lock:
                        decision=terminal(state_valid=not bad_state[0],prediction_valid=end in predicted,
                            deskew=d['status'] if d else None,ndt=(n['ndt_has_converged']=='1') if n and end in diagnosed and end in predicted else None,
                            oosm=o['oosm_result'] if o else None,fusion_applied=(f['update_applied']=='1') if f else None,
                            fresh_corrected=new_corrected_at(corrected,watermark,sent) and end in used,times_valid=times)
                    status,fatal=decision
                    if status is None:
                        if rospy.is_shutdown() or time.monotonic()>deadline:status='TIMEOUT';fatal=True
                        else:time.sleep(.005)
                fw.writerow([row['scan_index'],row['stamp_ns'],end,watermark,imu_cursor,status,int(fatal),(time.monotonic()-sent)*1000])
                active_written=True
                done+=1
                (run/'progress.json').write_text(json.dumps({'completed':done,'scan_index':int(row['scan_index']),'end_ns':end,'terminal':status}))
                if done%100==0 or status!='APPLIED':print(done,row['scan_index'],status,flush=True)
                if fatal:raise RuntimeError(status)
            assert done==sent_count==2441
    except Exception as e:
        failure=repr(e);print('STOP',failure,flush=True)
        if active is not None and not active_written:
            # This is an observed STOP, not a successful ACK or generated pose.
            fw.writerow([active['scan_index'],active['stamp_ns'],active['end_ns'],watermark,imu_cursor,
                         'STOP_BEFORE_TERMINAL:'+failure,1,(time.monotonic()-sent)*1000 if sent else ''])
    finally:
        stop.set();thread.join();fm.close();fr.close()
        for sub in subs:sub.unregister()
        with lock:
            logging[0]=False;fpose.close();fperf.close();pp.publish(prediction_path);fp.publish(fusion_path)
        (run/'replay_summary.json').write_text(json.dumps({'sent':sent_count,'completed_terminals':done,'imu_sent':imu_cursor,
            'wall_sec':time.monotonic()-started,'failure':failure,'gt_read':False,'candidate_feedback':0},indent=2))
    print('FINISHED; no further input; kept live for screenshot',flush=True)
    rospy.spin()


def sha_data(data):return hashlib.sha256(data).hexdigest()
if __name__=='__main__':main()
