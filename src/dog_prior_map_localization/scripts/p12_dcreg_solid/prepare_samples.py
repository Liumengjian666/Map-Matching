#!/usr/bin/env python3
"""Freeze 24 prior-metadata samples; extract only their original Livox scans."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

P11 = Path('/home/jian/rosbag/p11_solid_r1_20261011')
BAG = Path('/media/jian/HIKVISION/Jian_FASTLIVO_Data/dogrosbag/loop2_map_match_experiment_2026_08_08_17_36_19/dateset/loop3/bag/loop3_2026-08-11-16-09-07.bag')
MAP = Path('/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd')


def write_csv(path, rows):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def freeze(output):
    output.mkdir(parents=True, exist_ok=True)
    ndt = list(csv.DictReader((P11/'control/ndt.csv').open()))
    solid = list(csv.DictReader((P11/'solid_shadow/solid.csv').open()))
    assert len(ndt) == len(solid) == 5817
    positions = np.array([[float(r[f'final_used_t{a}']) for a in 'xyz'] for r in ndt])
    normal, straight, distant = [], [], []
    for i, row in enumerate(solid):
        assert int(row['source_hash'], 16) == int(ndt[i]['cloud_hash'], 16)
        if row['descriptor_status'] != 'OK':
            continue
        refs = np.array([[float(row[f'top{k}_map_{a}']) for a in 'xyz'] for k in (1, 2, 3)])
        spread = max(np.linalg.norm(refs[a]-refs[b]) for a in range(3) for b in range(a))
        if spread < 5:
            normal.append(i)
        if spread > 50:
            distant.append(i)
        segment = positions[max(0, i-25):min(len(ndt), i+26)]
        singular = np.linalg.svd(segment-segment.mean(axis=0), compute_uv=False)
        if np.linalg.norm(segment[-1]-segment[0]) > 2 and singular[0]**2/np.sum(singular**2) > .98:
            straight.append(i)
    selected = {}
    for name, pool in [('ordinary_low_top3_spread', normal),
                       ('straight_motion_corridor_proxy_NOT_SCENE_LABEL', straight),
                       ('widely_separated_top3', distant)]:
        pool = [i for i in pool if i not in selected]
        assert len(pool) >= 8
        for j in np.linspace(0, len(pool)-1, 8).round().astype(int):
            selected[pool[j]] = name
    rows = []
    for i in sorted(selected):
        row = dict(frame_index=i+1, stratum=selected[i], stamp_ns=solid[i]['stamp_ns'],
                   source_pcd=str(output/f'raw_scan_{i+1}.pcd'),
                   expected_points=ndt[i]['cloud_size_after_filter'], expected_hash=ndt[i]['cloud_hash'])
        for prefix in ('initial_guess', 'raw_ndt', 'final_used'):
            for field in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'):
                row[f'{prefix}_{field}'] = ndt[i][f'{prefix}_{field}']
        rows.append(row)
    write_csv(output/'samples.csv', rows)
    config = dict(protocol='P12_DCREG_SOLID_MINIMAL_R1', samples=24, top_k=3,
                  source_frame='livox_frame', target_frame='camera_init', deskew=False,
                  reference_pose_status='COARSE_POSITION_HYPOTHESIS',
                  candidate_rotation_source='recorded current NDT INITIAL guess, NOT raw/final pose',
                  candidate_translation_source='mapping IMU-origin coarse reference location; no claimed exact lever-arm/time transport',
                  dcreg_sha='8ce8451b15491a4bbe17cf85ab02a8bed6696861',
                  dcreg_parameterization='SO3', dcreg_parallel='Serial',
                  dcreg_parameters='unmodified author SolverParameters defaults: radius1m, max30, convergence translation1e-3m/rotation1e-5rad',
                  target=str(MAP), database=str(P11/'solid_database.txt'),
                  source_filter='finite XYZ; range0.5..80m; voxel0.25m; max1400 even-index cap',
                  target_filter='finite XYZ; voxel0.15m then voxel0.15m; no cap',
                  input_bag=str(BAG), input_bag_size=BAG.stat().st_size,
                  input_bag_historical_sha256='e7663609a90fd9e59552fefd7f2c4f9c339e8464610f86bfa1e8f2cff2e79df2',
                  bag_hash_recomputed_this_round=False, gt_loaded=False,
                  selection_uses_only_prior_p11_metadata=True,
                  scene_coverage_limit='straight-motion proxy is NOT independently verified corridor annotation',
                  samples_sha256=hashlib.sha256((output/'samples.csv').read_bytes()).hexdigest())
    for name, path in [('map', MAP), ('database', P11/'solid_database.txt')]:
        config[name+'_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    with (output/'freeze.json').open('x') as stream:
        json.dump(config, stream, indent=2)
    print('FROZEN', [r['frame_index'] for r in rows])


def extract(output):
    import rosbag
    import rospy
    rows = list(csv.DictReader((output/'samples.csv').open()))
    receipt = []
    with rosbag.Bag(str(BAG)) as bag:
        for row in rows:
            stamp_ns = int(row['stamp_ns'])
            found = []
            for _, msg, record_stamp in bag.read_messages(topics=['/livox/lidar'],
                    start_time=rospy.Time.from_sec(stamp_ns/1e9-.3),
                    end_time=rospy.Time.from_sec(stamp_ns/1e9+.3)):
                if msg.header.stamp.to_nsec() == stamp_ns:
                    found.append((msg, record_stamp))
            assert len(found) == 1, (row['frame_index'], len(found))
            msg, record_stamp = found[0]
            points = np.asarray([(p.x, p.y, p.z) for p in msg.points], dtype='<f4')
            assert msg.point_num == len(points)
            header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                      f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n')
            with Path(row['source_pcd']).open('xb') as stream:
                stream.write(header.encode('ascii'))
                stream.write(points.tobytes())
            receipt.append(dict(frame_index=row['frame_index'], stamp_ns=stamp_ns,
                                bag_record_ns=record_stamp.to_nsec(), frame_id=msg.header.frame_id,
                                raw_points=len(points), sha256=hashlib.sha256(Path(row['source_pcd']).read_bytes()).hexdigest()))
            print('EXTRACTED', row['frame_index'], len(points), flush=True)
    write_csv(output/'extraction.csv', receipt)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['freeze', 'extract'])
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    (freeze if args.action == 'freeze' else extract)(args.output)
