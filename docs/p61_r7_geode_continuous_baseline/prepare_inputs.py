#!/usr/bin/python3
"""Freeze the prescribed twenty real scans and Xsens measurements, no GT."""
import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2, PointField

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'docs/p61_r2_geode_real_data_run'))
from livox_contract import decode

ROOT = Path('/home/jian/livox_ws/p61_r7_geode_runs')
BAG = Path('/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag')
MAP = Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd')
INIT = REPO / 'docs/p61_r6_geode_causal_init_bridge/initialization.json'


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def main():
    import yaml
    expected = {MAP: '4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4',
                INIT: '9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d',
                BAG: '162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091'}
    for path, value in expected.items():
        assert sha(path) == value, str(path)
    ledger_path = REPO / 'docs/p61_r2_geode_real_data_run/scan_ledger.csv'
    assert sha(ledger_path) == '21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb'
    ledger = list(csv.DictReader(open(ledger_path)))
    chosen = ledger[2985:3005]
    state = json.loads(INIT.read_text())['initial_state']
    t0 = state['target_stamp_ns']
    end = int(chosen[-1]['stamp_ns']) + int(chosen[-1]['offset_max'])
    assert [int(r['scan_index']) for r in chosen] == list(range(2986, 3006))
    assert all(int(r['stamp_ns']) + int(r['offset_min']) >= t0 for r in chosen)
    out = ROOT / 'input'
    out.mkdir(parents=True, exist_ok=False)
    wanted = {int(r['stamp_ns']): r for r in chosen}
    scans, imus, seed = [], [], None
    dtype = np.dtype([(k, '<f4') for k in ('x', 'y', 'z', 'intensity', 'time')] + [('offset_time', '<u4')])
    with rosbag.Bag(str(BAG)) as bag, rosbag.Bag(str(out / 'twenty_scans.bag'), 'w') as target:
        for topic, raw, stamp in bag.read_messages(topics=['/livox/lidar', '/imu/data'], raw=True):
            if topic == '/imu/data':
                msg = raw[-1](); msg.deserialize(raw[1])
                ns = msg.header.stamp.to_nsec()
                if ns <= t0:
                    seed = msg
                elif ns <= end + 20000000:
                    imus.append(msg)
                    target.write('/imu/data', msg, msg.header.stamp)
                elif ns > end + 20000000:
                    break
                continue
            h, pts = decode(raw[1])
            if h['stamp_ns'] not in wanted:
                continue
            assert h['stamp_ns'] == stamp.to_nsec(), 'record stamp is not sensor header stamp'
            row = wanted[h['stamp_ns']]
            assert hashlib.sha256(raw[1]).hexdigest() == row['serialized_sha256']
            a = np.empty(len(pts), dtype=dtype)
            for name in ('x', 'y', 'z', 'offset_time'):
                a[name] = pts[name]
            a['intensity'] = pts['reflectivity']
            a['time'] = pts['offset_time'].astype(np.float64) * 1e-9
            msg = PointCloud2()
            msg.header.seq = int(row['scan_index']); msg.header.stamp = stamp
            msg.header.frame_id = h['frame_id']
            msg.height = 1; msg.width = len(pts); msg.point_step = dtype.itemsize
            msg.row_step = msg.width * msg.point_step; msg.is_dense = True
            msg.fields = [PointField(n, dtype.fields[n][1], PointField.UINT32 if n == 'offset_time' else PointField.FLOAT32, 1) for n in dtype.names]
            msg.data = a.tobytes()
            target.write('/geode/raw_points', msg, stamp)
            scans.append(dict(row, end_ns=h['stamp_ns'] + int(pts['offset_time'].max()), adapter_sha256=hashlib.sha256(msg.data).hexdigest()))
    assert len(scans) == 20 and seed.header.stamp.to_nsec() == state['last_raw_imu_ns']
    assert imus[0].header.stamp.to_nsec() > t0 and imus[-1].header.stamp.to_nsec() >= end
    ti = np.array(state['T_M_I']).reshape(4, 4)
    il = np.array(state['T_I_L']).reshape(4, 4)
    assert np.max(np.abs(ti @ il - np.array(state['T_M_L']).reshape(4, 4))) < 1e-12
    initialization = dict(explicit_state_enable=True, state_stamp_sec=t0 // 10**9, state_stamp_nsec=t0 % 10**9,
        map_T_imu_translation_m=ti[:3, 3].tolist(), map_T_imu_rotation_row_major=ti[:3, :3].ravel().tolist(),
        velocity_map_mps=state['velocity_M'], gyro_bias_imu_radps=state['gyro_bias_I'],
        accel_bias_imu_mps2=state['accel_bias_I'], gravity_map_mps2=state['gravity_M'],
        boundary_imu_stamp_sec=seed.header.stamp.secs, boundary_imu_stamp_nsec=seed.header.stamp.nsecs,
        boundary_imu_accel_mps2=[getattr(seed.linear_acceleration, k) for k in ('x','y','z')],
        boundary_imu_gyro_radps=[getattr(seed.angular_velocity, k) for k in ('x','y','z')], ndt_seed_enable=False)
    (out / 'initialization.yaml').write_text(yaml.safe_dump({'initialization': initialization, 'deskew': {
        'T_imu_lidar_translation_m': il[:3, 3].tolist(), 'T_imu_lidar_rotation_row_major': il[:3, :3].ravel().tolist()}}, sort_keys=False))
    manifest = {'inputs': {str(k): v for k, v in expected.items()}, 'scan_count': 20, 'scans': scans,
        'imu_count': len(imus), 'imu_first_ns': imus[0].header.stamp.to_nsec(), 'imu_last_ns': imus[-1].header.stamp.to_nsec(),
        'bag_sha256': sha(out / 'twenty_scans.bag'), 'config_sha256': sha(out / 'initialization.yaml'),
        'covariance_policy': 'Existing 15D fixed initialization; NOT 23D covariance restoration', 'gt_read': False}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({k: v for k, v in manifest.items() if k != 'scans'}, indent=2))


if __name__ == '__main__':
    main()
