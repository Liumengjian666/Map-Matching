#!/usr/bin/env python3
"""Fresh native LIO bridge with a hard R5-state gate and causal terminal IMU step."""
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
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
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation
import yaml
from prepare_bridge import ROOT, MAP, MAP_SHA, TEST, sha

HERE=Path(__file__).resolve().parent
AUTHOR=ROOT/'author_ws/src/FAST_LIO'


def transform(row,prefix='T_M_L'):
    return np.array([float(row[f'{prefix}_{r}{c}']) for r in range(4) for c in range(4)]).reshape(4,4)


def parity(row):
    original=Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/mapping_trajectory.csv')
    with original.open() as stream:reference=list(csv.DictReader(stream))[-1]
    a,b=transform(row),transform(reference)
    dt=float(np.linalg.norm(a[:3,3]-b[:3,3]))
    dr=float(np.degrees(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude()))
    same_time=abs(float(reference['scan_end_sec'])-float(row['scan_end_sec']))<=1e-6
    return dict(pass_gate=dt<=.2 and dr<=2. and same_time,same_scan_end_time=same_time,
        translation_difference_m=dt,rotation_difference_deg=dr,
        reference_trajectory_sha256=sha(original),scan_sequence=2441,
        reference_native_end_sec=float(reference['scan_end_sec']),new_native_end_sec=float(row['scan_end_sec']),
        R5_T_M_L=b.tolist(),bridge_T_M_L=a.tolist(),alignment_applied=False)


def main():
    output=ROOT/'bridge_run';output.mkdir(exist_ok=False)
    assert sha(MAP)==MAP_SHA
    manifest=json.loads((ROOT/'bridge_input.json').read_text())
    bag_path=Path(manifest['bag']);assert sha(bag_path)==manifest['bag_sha256']
    assert manifest['test_start_ns']==TEST and manifest['counts']['/livox/lidar']==2984
    if shutil.disk_usage(ROOT).free<20*1024**3:raise RuntimeError('disk reserve unavailable')
    config=yaml.safe_load((HERE.parent/'p61_r5_geode_frozen_prior_map/mapping_config.yaml').read_text())
    config.update(p61_output_directory=str(output),use_sim_time=False,
        r6_target_ns=str(TEST),r6_last_imu_ns=str(manifest['last_imu_ns']),r6_gate_passed=False)
    (output/'config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',11396))==0:raise RuntimeError('private master port already in use')
    env=os.environ.copy();env.update(ROS_MASTER_URI='http://127.0.0.1:11396',ROS_IP='127.0.0.1',
        ROS_HOME=str(output/'ros_home'),ROS_LOG_DIR=str(output/'ros_logs'),
        LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0',OMP_NUM_THREADS='3')
    os.environ.update({k:v for k,v in env.items() if k.startswith('ROS_')})
    logs=[(output/'master.log').open('x'),(output/'mapper.log').open('x')]
    master=subprocess.Popen(['roscore','-p','11396'],env=env,stdout=logs[0],stderr=subprocess.STDOUT)
    mapper=None;reader_stream=None;began=time.monotonic();done=[0];condition=threading.Condition()
    try:
        deadline=time.monotonic()+20
        while True:
            try:
                xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/r6_start');break
            except OSError:
                if master.poll() is not None or time.monotonic()>deadline:raise RuntimeError('master not ready')
                time.sleep(.05)
        rospy.init_node('r6_causal_bridge_replay',disable_signals=True)
        for k,v in config.items():rospy.set_param('/'+k,v)
        binary=ROOT/'author_ws/devel/lib/fast_lio/fastlio_mapping'
        mapper=subprocess.Popen([str(binary)],env=env,stdout=logs[1],stderr=subprocess.STDOUT)
        def ack(msg):
            with condition:done[0]=msg.data;condition.notify_all()
        subscriber=rospy.Subscriber('/p61_mapping_ack',UInt64,ack,queue_size=10)
        lidar=rospy.Publisher('/livox/lidar',CustomMsg,queue_size=1)
        imu=rospy.Publisher('/imu/data',Imu,queue_size=1000)
        deadline=time.monotonic()+30
        while not(lidar.get_num_connections() and imu.get_num_connections()):
            if mapper.poll() is not None or time.monotonic()>deadline:raise RuntimeError('subscription unavailable')
            time.sleep(.05)
        with rosbag.Bag(str(bag_path)) as bag:
            imus=[m for _,m,_ in bag.read_messages(topics=['/imu/data'])];cursor=0;prefix_imu_count=None
            with (output/'replay.csv').open('x',newline='') as stream:
                writer=csv.writer(stream,lineterminator='\n')
                writer.writerow(['sequence','scan_start_ns','raw_point_end_ns','imu_sent','publisher_to_ack_ms'])
                for i,(_,msg,_) in enumerate(bag.read_messages(topics=['/livox/lidar']),1):
                    start=msg.header.stamp.to_nsec();end=start+max(p.offset_time for p in msg.points)
                    if not manifest['start_ns']<=start<=end<=TEST:raise RuntimeError('LiDAR crosses cutoff')
                    tick=time.monotonic();sent=0
                    while cursor<len(imus):
                        m=imus[cursor]
                        if m.header.stamp.to_nsec()>TEST:raise RuntimeError('future IMU')
                        imu.publish(m);cursor+=1;sent+=1
                        if m.header.stamp.to_nsec()>end:break
                    if imus[cursor-1].header.stamp.to_nsec()<end:raise RuntimeError('no measured scan tail bracket')
                    lidar.publish(msg)
                    with condition:
                        deadline=time.monotonic()+30
                        while done[0]<i:
                            if mapper.poll() is not None or time.monotonic()>deadline:raise RuntimeError(f'ACK missing at {i}')
                            condition.wait(.05)
                    if done[0]!=i:raise RuntimeError('ACK count conflict')
                    if reader_stream is None:
                        reader_stream=(output/'mapping_trajectory.csv').open();reader=csv.DictReader(reader_stream)
                    row=next(reader)
                    if int(row['sequence'])!=i:raise RuntimeError('trajectory/ACK conflict')
                    for p in ['T_M_I','T_I_L','T_M_L']:
                        t=transform(row,p)
                        if not np.isfinite(t).all() or np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))>1e-4:
                            raise RuntimeError(f'illegal SE3 at {i}')
                    if abs(float(row['scan_start_sec'])-start*1e-9)>1e-6:raise RuntimeError('scan time conflict')
                    writer.writerow([i,start,end,sent,(time.monotonic()-tick)*1000]);stream.flush()
                    if i==2441:
                        gate=parity(row);gate['imu_delivered_at_gate']=cursor
                        (output/'r5_state_consistency.json').write_text(json.dumps(gate,indent=2)+'\n')
                        print('R5_STATE_GATE',json.dumps(gate),flush=True)
                        if not gate['pass_gate']:raise RuntimeError('MAP_FRAME_INCONSISTENT')
                        prefix_imu_count=cursor;rospy.set_param('/r6_gate_passed',True)
                    if i%100==0:print('BRIDGE_ACK',i,flush=True)
                    if shutil.disk_usage(ROOT).free<20*1024**3:raise RuntimeError('disk reserve lost')
            if done[0]!=2984:raise RuntimeError('bridge incomplete')
            while cursor<len(imus):
                m=imus[cursor]
                if m.header.stamp.to_nsec()>TEST:raise RuntimeError('future terminal IMU')
                imu.publish(m);cursor+=1
        rospy.wait_for_service('/r6_finalize_init',timeout=5)
        terminal=rospy.ServiceProxy('/r6_finalize_init',Trigger);deadline=time.monotonic()+5
        while True:
            response=terminal()
            if response.success:break
            if response.message!='WAIT_IMU_DELIVERY' or time.monotonic()>deadline:
                raise RuntimeError('terminal propagation failed: '+response.message)
            time.sleep(.01)
        state=json.loads((output/'initial_state.json').read_text())
        covariance=np.loadtxt(output/'initial_covariance.csv',delimiter=',')
        t=np.array(state['T_M_L']).reshape(4,4)
        assert state['target_stamp_ns']==TEST and state['last_imu_ns']<=TEST
        assert np.isfinite(t).all() and np.isfinite(covariance).all() and covariance.shape==(23,23)
        assert abs(np.linalg.det(t[:3,:3])-1)<1e-4
        result=dict(initial_state=state,position_m=t[:3,3].tolist(),
            roll_pitch_yaw_deg=Rotation.from_matrix(t[:3,:3]).as_euler('xyz',degrees=True).tolist(),
            source='fresh native FAST-LIO followed by native IMU-only integration; no test LiDAR',
            original_map_sha256=MAP_SHA,map_frame='R5 camera_init (direct boundary consistency checked)',
            input_bag_sha256=manifest['bag_sha256'],binary_sha256=sha(binary),config_sha256=sha(output/'config.yaml'),
            bridge_scans=543,total_scans=2984,imu_delivered_total=cursor,
            imu_delivered_after_r5_state=cursor-prefix_imu_count,
            last_lidar_start_ns=start,last_lidar_raw_end_ns=end,
            initial_state_sha256=sha(output/'initial_state.json'),covariance_sha256=sha(output/'initial_covariance.csv'),
            covariance_shape=[23,23],covariance_symmetry_error=float(np.max(np.abs(covariance-covariance.T))),
            covariance_min_eigenvalue=float(np.linalg.eigvalsh((covariance+covariance.T)*.5).min()),
            raw_test_lidar_used_for_initialization=False,gt_used=False,
            initialization_frozen_wall_unix=time.time(),bridge_wall_sec=time.monotonic()-began)
        with (output/'initialization_freeze.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
        print('INITIALIZATION_FROZEN',json.dumps(result),flush=True)
        mapper.send_signal(signal.SIGINT)
        if mapper.wait(timeout=60)!=0:raise RuntimeError('mapper shutdown failed')
        shutil.copytree(AUTHOR/'Log',output/'author_logs')
        assert sha(MAP)==MAP_SHA
    finally:
        if reader_stream is not None:reader_stream.close()
        for proc in [mapper,master]:
            if proc is not None and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:print('PROCESS STILL RUNNING',proc.pid,flush=True)
        for log in logs:log.close()


if __name__=='__main__':main()
