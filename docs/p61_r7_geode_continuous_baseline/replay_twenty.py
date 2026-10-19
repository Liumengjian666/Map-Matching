#!/usr/bin/python3
"""Bounded live-input driver/recorder. Does not compute localization states."""
import argparse
import csv
import hashlib
import json
import os
import threading
import time
import xmlrpc.client
from pathlib import Path

import rosbag
import rosgraph
import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, PointCloud2


def rows(path):
    try:
        with open(path) as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []


def connected_edges(info, edges):
    return all(any(b[1]==peer and b[2]=='o' and b[3]=='TCPROS' and b[4]==topic and b[5]
                   for b in info[node]) for node,topic,peer in edges)


def new_corrected_at(corrected, watermark, sent):
    return any(abs(ns-watermark)<1000 and received>=sent
               for ns,received in corrected.items())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input-dir', type=Path, default=Path('/home/jian/livox_ws/p61_r7_geode_runs/input'))
    ap.add_argument('--run-dir', type=Path, default=Path('/home/jian/livox_ws/p61_r7_geode_runs/formal'))
    args = ap.parse_args()
    assert hashlib.sha256((args.input_dir / 'manifest.json').read_bytes()).hexdigest() == \
        'dda2ac02550f6ee5f294261094ff0b42c5494c5c5b0b3c5bc6ca56ed2a880cb4', 'input manifest is not frozen R7 identity'
    manifest = json.loads((args.input_dir / 'manifest.json').read_text())
    assert manifest['scan_count']==len(manifest['scans'])==20
    assert [int(x['scan_index']) for x in manifest['scans']]==list(range(2986,3006))
    assert hashlib.sha256((args.input_dir / 'initialization.yaml').read_bytes()).hexdigest()==manifest['config_sha256']
    assert hashlib.sha256((args.input_dir / 'twenty_scans.bag').read_bytes()).hexdigest() == manifest['bag_sha256']
    clouds, imus = [], []
    with rosbag.Bag(str(args.input_dir / 'twenty_scans.bag')) as bag:
        for topic, msg, _ in bag.read_messages():
            (imus if topic == '/imu/data' else clouds).append(msg)
    assert len(clouds) == 20 and [m.header.seq for m in clouds] == list(range(2986,3006))
    for cloud,row in zip(clouds,manifest['scans']):
        assert cloud.header.stamp.to_nsec()==int(row['stamp_ns'])
        assert hashlib.sha256(cloud.data).hexdigest()==row['adapter_sha256']
    rospy.init_node('geode_r7_bounded_input')
    lock = threading.RLock()
    logging_open = [True]
    latest_high = [0]
    corrected = {}
    predicted = {}
    used = {}
    diagnosed = set()
    run = args.run_dir
    run.mkdir(parents=True, exist_ok=True)
    poses_file = open(run / 'live_trajectories.csv', 'x', buffering=1)
    pose_writer = csv.writer(poses_file)
    pose_writer.writerow(['stream','stamp_ns','receive_monotonic','px','py','pz','qx','qy','qz','qw','vx','vy','vz'])
    path_pub = rospy.Publisher('/geode/prediction_path', RosPath, queue_size=2, latch=True)
    path = RosPath(); path.header.frame_id = 'camera_init'
    perf_file = open(run / 'ndt_performance.csv', 'x', buffering=1)
    perf_writer = csv.writer(perf_file)
    perf_keys = ['ndt_converged','align_time_ms','preprocess_time_ms','localization_time_ms',
                 'scan_points','map_points','fitness_score','iterations']
    perf_writer.writerow(['stamp_ns']+perf_keys)

    def performance(msg):
        with lock:
            if not logging_open[0]: return
            for status in msg.status:
                if status.name == 'dog_prior_map_ndt':
                    d = {kv.key:kv.value for kv in status.values}
                    perf_writer.writerow([msg.header.stamp.to_nsec()]+[d.get(k,'') for k in perf_keys])
                    diagnosed.add(msg.header.stamp.to_nsec())

    def record(msg, kind):
        with lock:
            if not logging_open[0]: return
            p = msg.pose if kind == 'PREDICTION' else msg.pose.pose
            v = [0,0,0] if kind == 'PREDICTION' else [msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]
            pose_writer.writerow([kind,msg.header.stamp.to_nsec(),time.monotonic(),p.position.x,p.position.y,p.position.z,
                p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]+v)
            if kind == 'HIGH_RATE': latest_high[0] = max(latest_high[0],msg.header.stamp.to_nsec())
            if kind == 'CORRECTED': corrected[msg.header.stamp.to_nsec()] = time.monotonic()
            if kind == 'NDT_USED': used[msg.header.stamp.to_nsec()] = True
            if kind == 'PREDICTION':
                predicted[msg.header.stamp.to_nsec()] = True
                path.header = msg.header; path.poses.append(msg); path_pub.publish(path)
    subs = [rospy.Subscriber('/dog_livo/odom_high_rate', Odometry, lambda m: record(m,'HIGH_RATE'),queue_size=1000),
            rospy.Subscriber('/dog_livo/odom_corrected',Odometry,lambda m: record(m,'CORRECTED'),queue_size=100),
            rospy.Subscriber('/dog_livo/ndt_odom',Odometry,lambda m: record(m,'NDT_USED'),queue_size=100),
            rospy.Subscriber('/geode/scan_end_prediction',PoseStamped,lambda m: record(m,'PREDICTION'),queue_size=100)]
    subs.append(rospy.Subscriber('/dog_livo/diagnostics',DiagnosticArray,performance,queue_size=100))
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=1000)
    cloud_pub = rospy.Publisher('/geode/raw_points', PointCloud2, queue_size=1)

    def wait(test, description, seconds=30):
        deadline = time.monotonic()+seconds
        while not test():
            if rospy.is_shutdown() or time.monotonic()>deadline:
                raise RuntimeError('bounded wait failed: '+description)
            time.sleep(.005)
    wait(lambda: imu_pub.get_num_connections()>=2 and cloud_pub.get_num_connections()>=1,'real NDT + EKF subscribers',180)
    repo = Path(__file__).resolve().parents[2]
    assert hashlib.sha256((repo/'docs/p61_r2_geode_real_data_run/scan_ledger.csv').read_bytes()).hexdigest() == \
        '21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb'
    for name,expected in manifest['inputs'].items():
        if name.endswith('frozen_map.pcd'):
            assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==expected
    def check_params(config,prefix=''):
        for key,value in config.items():
            path=prefix+'/'+key
            if isinstance(value,dict): check_params(value,path)
            else: assert rospy.get_param(path)==value, 'runtime parameter mismatch: '+path
    check_params(yaml.safe_load((repo/'src/dog_prior_map_localization/config/geode_r7_baseline.yaml').read_text()))
    check_params(yaml.safe_load((args.input_dir/'initialization.yaml').read_text()))
    master = rosgraph.Master('/geode_r7_bounded_input')
    pids = {'publisher':os.getpid()}
    for name,key in [('/dog_prior_map_ndt','ndt'),('/dog_prior_map_ekf','ekf'),('/geode_r7_rviz','rviz')]:
        try:
            uri = master.lookupNode(name)
            pids[key] = xmlrpc.client.ServerProxy(uri).getPid('/geode_r7_bounded_input')[2]
        except Exception:
            if key != 'rviz': raise
    (run / 'pids.json').write_text(json.dumps(pids,indent=2))
    apis={name:xmlrpc.client.ServerProxy(master.lookupNode(name))
          for name in ('/dog_prior_map_ekf','/dog_prior_map_ndt')}
    edges=[('/dog_prior_map_ekf','/geode/scan_end_prediction','/dog_prior_map_ndt'),
           ('/dog_prior_map_ekf','/geode/scan_end_prediction','/geode_r7_bounded_input'),
           ('/dog_prior_map_ekf','/geode/deskewed_points','/dog_prior_map_ndt'),
           ('/dog_prior_map_ekf','/dog_livo/odom_corrected','/geode_r7_bounded_input'),
           ('/dog_prior_map_ekf','/dog_livo/odom_high_rate','/geode_r7_bounded_input'),
           ('/dog_prior_map_ndt','/dog_livo/ndt_odom','/dog_prior_map_ekf'),
           ('/dog_prior_map_ndt','/dog_livo/ndt_odom','/geode_r7_bounded_input'),
           ('/dog_prior_map_ndt','/dog_livo/diagnostics','/geode_r7_bounded_input')]
    def chain_ready():
        info={name:api.getBusInfo('/geode_r7_bounded_input')[2] for name,api in apis.items()}
        ready=connected_edges(info,edges)
        if ready: (run/'connection_readiness.json').write_text(json.dumps(info,indent=2))
        return ready
    wait(chain_ready,'all actual TCPROS prediction/Deskew/NDT/fusion connections',60)
    stop = threading.Event()
    memory_file = open(run / 'memory.csv','x',buffering=1)
    mw = csv.writer(memory_file); mw.writerow(['wall_monotonic','process','pid','rss_kib','hwm_kib','utime_ticks','stime_ticks'])

    def monitor():
        while not stop.is_set():
            for name,pid in pids.items():
                try:
                    status = Path('/proc/%d/status'%pid).read_text().splitlines()
                    stat = Path('/proc/%d/stat'%pid).read_text().rsplit(')',1)[1].split()
                    values = {x.split(':')[0]:x.split(':')[1].split()[0] for x in status if x.startswith(('VmRSS:','VmHWM:'))}
                    mw.writerow([time.monotonic(),name,pid,values['VmRSS'],values['VmHWM'],stat[11],stat[12]])
                except FileNotFoundError:
                    stop.set()
            stop.wait(.01)
    thread = threading.Thread(target=monitor); thread.start()
    receipts = open(run / 'publisher_receipts.csv','x',buffering=1)
    writer = csv.writer(receipts)
    writer.writerow(['scan_index','start_ns','end_ns','imu_watermark_ns','imu_sent_total','publish_to_ack_ms','status'])
    cursor = 0; completed = 0; started = time.monotonic()
    try:
        for cloud,row in zip(clouds,manifest['scans']):
            end = int(row['end_ns'])
            while cursor < len(imus):
                imu_pub.publish(imus[cursor]); cursor += 1
                if imus[cursor-1].header.stamp.to_nsec() >= end: break
            watermark = imus[cursor-1].header.stamp.to_nsec()
            wait(lambda: latest_high[0]>=watermark,'EKF processed real IMUs')
            sent = time.monotonic(); cloud_pub.publish(cloud)
            def ack():
                fusion = rows(run/'fusion.csv')
                with lock:
                    # The existing corrected publisher reconstructs ros::Time
                    # from native double seconds. Do not require impossible
                    # bit equality to the raw integer IMU epoch timestamp.
                    fresh_corrected = new_corrected_at(corrected,watermark,sent)
                return any(int(r['measurement_ns'])==end for r in fusion) and fresh_corrected and \
                    end in predicted and end in used and end in diagnosed
            wait(ack,'same-measurement OOSM receipt and corrected stream')
            receipt = next(r for r in rows(run/'fusion.csv') if int(r['measurement_ns'])==end)
            status = 'APPLIED' if receipt['update_applied']=='1' else 'REJECTED'
            writer.writerow([cloud.header.seq,cloud.header.stamp.to_nsec(),end,watermark,cursor,(time.monotonic()-sent)*1000,status])
            completed += 1
            print('scan',cloud.header.seq,status,'end_ns',end,flush=True)
            if status!='APPLIED': raise RuntimeError('preserved rejected fusion measurement')
        assert completed==20
        (run/'replay_summary.json').write_text(json.dumps({'scans_sent':completed,'imu_sent':cursor,
            'sensor_span_sec':(int(manifest['scans'][-1]['end_ns'])-int(manifest['scans'][0]['stamp_ns']))/1e9,
            'wall_sec':time.monotonic()-started,'gt_read':False,'candidate_feedback':0},indent=2))
    finally:
        stop.set(); thread.join(); memory_file.close(); receipts.close()
        for sub in subs: sub.unregister()
        with lock:
            logging_open[0] = False
            poses_file.close(); perf_file.close()
    # Keep real ROS trajectories available for the live viewer; no replays.
    print('COMPLETE; algorithm remains live for RViz, no further input',flush=True)
    rospy.spin()


if __name__=='__main__':
    main()
