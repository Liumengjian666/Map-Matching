#!/usr/bin/env python3
"""One isolated author FAST-LIO run on an already frozen mapping-only input."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import xmlrpc.client

import numpy as np
import rosbag
import rospy
from livox_ros_driver.msg import CustomMsg
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt64
import yaml

ROOT = Path('/home/jian/livox_ws/p61_r5_geode_runs')
AUTHOR = ROOT / 'author_ws/src/FAST_LIO'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def proc_stats(pid):
    result = {'wall_monotonic': time.monotonic(), 'pid': pid}
    try:
        for line in Path(f'/proc/{pid}/status').read_text().splitlines():
            if line.startswith(('VmRSS:', 'VmHWM:')):
                k, value, _ = line.split(); result[k.rstrip(':')+'_kib'] = int(value)
        result['cpu_ticks'] = sum(map(int, Path(f'/proc/{pid}/stat').read_text().split()[13:15]))
    except FileNotFoundError:
        pass
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', choices=['smoke_100', 'formal_mapping'], required=True)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    output = ROOT / args.name
    output.mkdir(exist_ok=False)
    if shutil.disk_usage(ROOT).free < 20*1024**3:
        raise RuntimeError('20 GiB reserve not available')
    manifest = json.loads((ROOT/'mapping_only.json').read_text())
    bag_path = Path(manifest['bag'])
    if sha(bag_path) != manifest['bag_sha256']:
        raise RuntimeError('filtered input changed')
    config = yaml.safe_load(args.config.read_text())
    if config['common']['imu_topic'] != '/imu/data' or config['mapping']['extrinsic_est_en']:
        raise RuntimeError('external Xsens / fixed extrinsics required')
    config.update(p61_output_directory=str(output), use_sim_time=False)
    (output/'mapping_config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:11395', ROS_HOME=str(output/'ros_home'),
               ROS_LOG_DIR=str(output/'ros_logs'), ROS_IP='127.0.0.1', OMP_NUM_THREADS='3',
               LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0')
    os.environ.update({k:v for k,v in env.items() if k.startswith('ROS_')})
    master_log = (output/'master.log').open('x')
    mapper_log = (output/'mapper.log').open('x')
    master = subprocess.Popen(['roscore','-p','11395'], env=env, stdout=master_log, stderr=subprocess.STDOUT)
    mapper = None
    samples = []
    try:
        deadline = time.monotonic()+20
        while True:
            try:
                xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/p61_wait')
                break
            except OSError:
                if time.monotonic()>deadline: raise RuntimeError('isolated master unavailable')
                time.sleep(.1)
        rospy.init_node('p61_gated_replay', anonymous=False, disable_signals=True)
        for k,v in config.items(): rospy.set_param('/'+k,v)
        mapper = subprocess.Popen([str(ROOT/'author_ws/devel/lib/fast_lio/fastlio_mapping')],
                                  env=env,stdout=mapper_log,stderr=subprocess.STDOUT)
        done = [0]
        condition = threading.Condition()
        def ack(msg):
            with condition:
                done[0]=msg.data; condition.notify_all()
        subscriber = rospy.Subscriber('/p61_mapping_ack',UInt64,ack,queue_size=10)
        lidar_pub = rospy.Publisher('/livox/lidar',CustomMsg,queue_size=1)
        imu_pub = rospy.Publisher('/imu/data',Imu,queue_size=1000)
        deadline = time.monotonic()+30
        while not(lidar_pub.get_num_connections() and imu_pub.get_num_connections()):
            if mapper.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('mapper not subscribed')
            time.sleep(.05)
        with rosbag.Bag(str(bag_path)) as bag:
            imus = [m for _,m,_ in bag.read_messages(topics=['/imu/data'])]
            cursor=0
            expected = 100 if args.name=='smoke_100' else manifest['verified_counts']['/livox/lidar']
            began=time.monotonic()
            with (output/'replay.csv').open('x', newline='') as stream:
                writer=csv.writer(stream,lineterminator='\n')
                writer.writerow(['sequence','scan_start_ns','point_end_ns','imu_published','publisher_to_ack_ms'])
                for i,(_,msg,_) in enumerate(bag.read_messages(topics=['/livox/lidar']),1):
                    if i>expected: break
                    start=msg.header.stamp.to_nsec(); end=start+max(p.offset_time for p in msg.points)
                    if not manifest['window_start_ns']<=start<=end<=manifest['window_end_ns']:
                        raise RuntimeError('sensor time gate violated')
                    tick=time.monotonic(); sent=0
                    # Publish exactly through the first measured Xsens sample after scan end.
                    while cursor<len(imus):
                        m=imus[cursor]; imu_pub.publish(m); cursor+=1; sent+=1
                        if m.header.stamp.to_nsec()>end: break
                    if imus[cursor-1].header.stamp.to_nsec()<end:
                        raise RuntimeError('no real IMU tail bracket')
                    lidar_pub.publish(msg)
                    with condition:
                        deadline=time.monotonic()+30
                        while done[0]<i:
                            samples.append(proc_stats(mapper.pid))
                            if mapper.poll() is not None or time.monotonic()>deadline:
                                raise RuntimeError(f'missing real consumed-scan ACK {i}')
                            condition.wait(.05)
                    if done[0]!=i: raise RuntimeError('unexpected ACK count')
                    writer.writerow([i,start,end,sent,(time.monotonic()-tick)*1000]); stream.flush()
                    if i%100==0: print('ACK',i,'of',expected,flush=True)
                    if shutil.disk_usage(ROOT).free<20*1024**3: raise RuntimeError('disk reserve lost')
            if done[0]!=expected: raise RuntimeError('incomplete mapping')
            runtime=time.monotonic()-began
        samples.append(proc_stats(mapper.pid))
        mapper.send_signal(signal.SIGINT)
        if mapper.wait(timeout=60)!=0: raise RuntimeError('mapper exited with failure')
        shutil.copytree(AUTHOR/'Log', output/'author_logs')
        rows=list(csv.DictReader((output/'mapping_trajectory.csv').open()))
        if len(rows)!=expected: raise RuntimeError('trajectory missing consumed scans')
        for row in rows:
            for prefix in ['T_M_I','T_I_L','T_M_L']:
                t=np.array([float(row[f'{prefix}_{r}{c}']) for r in range(4) for c in range(4)]).reshape(4,4)
                if not np.isfinite(t).all() or np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))>1e-4 or abs(np.linalg.det(t[:3,:3])-1)>1e-4:
                    raise RuntimeError('non-SE3 state')
        binary=ROOT/'author_ws/devel/lib/fast_lio/fastlio_mapping'
        result=dict(completed=True,input_scans=expected,wall_sec=runtime,
                    binary_sha256=sha(binary),bag_sha256=manifest['bag_sha256'],
                    config_sha256=sha(output/'mapping_config.yaml'),
                    trajectory_sha256=sha(output/'mapping_trajectory.csv'),map_sha256=sha(output/'frozen_map.pcd'),
                    process_samples=samples,gt_read=False,test_scans_delivered=0,
                    use_sim_time=False,sensor_timestamps_modified=False)
        (output/'run_freeze.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='process_samples'},indent=2))
    finally:
        for proc in (mapper,master):
            if proc is not None and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try: proc.wait(timeout=15)
                except subprocess.TimeoutExpired: print('PROCESS STILL RUNNING',proc.pid,flush=True)
        mapper_log.close(); master_log.close()


if __name__=='__main__': main()
