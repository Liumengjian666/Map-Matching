#!/usr/bin/env python3
"""Create a physically gated, byte-preserving LiDAR/Xsens-only mapping bag."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import rosbag
import rospy

START = 1706671289400000810
END = 1706671533569869338
BAG_SHA = '162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def admitted(row):
    return START <= int(row['stamp_ns']) + int(row['offset_min']) and int(row['stamp_ns']) + int(row['offset_max']) <= END


def prepare(source, ledger, output):
    source, ledger, output = map(Path, (source, ledger, output))
    if source.resolve() != Path('/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag'):
        raise ValueError('only frozen internal source allowed')
    if output.exists() or not str(output.resolve()).startswith('/home/jian/livox_ws/p61_r5_geode_runs/'):
        raise ValueError('new internal output required')
    if sha(source) != BAG_SHA:
        raise ValueError('source bag identity mismatch')
    with ledger.open() as stream:
        rows = list(csv.DictReader(stream))
    selected = {int(r['stamp_ns']): r for r in rows if admitted(r)}
    if len(selected) != 2441:
        raise ValueError('frozen window/ledger count mismatch')
    lidar, imu = [], []
    with rosbag.Bag(str(source)) as src, rosbag.Bag(str(output), 'w') as dst:
        for topic, raw, stamp in src.read_messages(topics=['/livox/lidar', '/imu/data'], raw=True,
                start_time=rospy.Time(START // 10**9, START % 10**9),
                end_time=rospy.Time(END // 10**9, END % 10**9)):
            cls = raw[-1]
            msg = cls().deserialize(raw[1])
            ns = msg.header.stamp.to_nsec()
            if not START <= ns <= END:
                raise ValueError('record/header gate conflict')
            if topic == '/livox/lidar':
                if ns not in selected:
                    continue
                r = selected[ns]
                if hashlib.sha256(raw[1]).hexdigest() != r['serialized_sha256']:
                    raise ValueError('LiDAR source SHA mismatch')
                if ns + max(p.offset_time for p in msg.points) > END:
                    raise ValueError('point tail crosses window')
                lidar.append(ns)
            else:
                imu.append(ns)
            dst.write(topic, raw, stamp, raw=True)
    if lidar != sorted(selected) or any(b <= a for a, b in zip(imu, imu[1:])):
        raise ValueError('input count/order mismatch')
    # Re-traverse the physical artifact, not just the intended filter predicates.
    verified = {'/livox/lidar': 0, '/imu/data': 0}
    with rosbag.Bag(str(output)) as bag:
        for topic, raw, _ in bag.read_messages(raw=True):
            msg = raw[-1]().deserialize(raw[1]); ns = msg.header.stamp.to_nsec()
            assert topic in verified and START <= ns <= END
            verified[topic] += 1
            if topic == '/livox/lidar':
                assert hashlib.sha256(raw[1]).hexdigest() == selected[ns]['serialized_sha256']
    result = dict(source=str(source), source_sha256=BAG_SHA, ledger_sha256=sha(ledger),
                  bag=str(output), bag_sha256=sha(output), bytes=output.stat().st_size,
                  window_start_ns=START, window_end_ns=END, verified_counts=verified,
                  lidar_first_ns=lidar[0], lidar_last_ns=lidar[-1], imu_first_ns=imu[0],
                  imu_last_ns=imu[-1], whole_scan_guard=True, gt_read=False)
    output.with_suffix('.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    prepare(*sys.argv[1:])
