#!/usr/bin/env python3
"""Bounded causal data adapter; accumulation itself is the author's function."""
import argparse
import bisect
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import time

import numpy as np

spec = importlib.util.spec_from_file_location('p14_prepare', Path(__file__).resolve().parents[1] / 'p14_inlier_baseline/prepare.py')
p14 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p14)
sha, PIN = p14.sha, p14.PIN

QUERIES = [16, 45, 708, 1891, 1994, 4691]
AUTHOR = Path('/home/jian/rosbag/p14_inlier_r1_20261011/upstream_inlier')
P11 = Path('/home/jian/rosbag/p11_solid_r1_20261011')
MAPPING = Path('/home/jian/rosbag/loop2/loop2mapping/pcd/lidar_poses.txt')


def author_submap():
    spec = importlib.util.spec_from_file_location('author_submaps', AUTHOR / 'inlier/eval/submaps.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build_submap


def matrix(x, q):
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-4:
        raise ValueError('Invalid logged quaternion')
    q /= np.linalg.norm(q)
    a, b, c, d = q
    t = np.eye(4)
    t[:3, :3] = [[1-2*(b*b+c*c), 2*(a*b-c*d), 2*(a*c+b*d)],
                 [2*(a*b+c*d), 1-2*(a*a+c*c), 2*(b*c-a*d)],
                 [2*(a*c-b*d), 2*(b*c+a*d), 1-2*(a*a+b*b)]]
    t[:3, 3] = x
    if not np.isfinite(t).all():
        raise ValueError('Nonfinite pose')
    return t


def write_csv(path, rows, fields=None):
    with path.open('x', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields or list(rows[0]), lineterminator='\n')
        w.writeheader()
        w.writerows(rows)


def write_pcd(path, points):
    header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n')
    with path.open('xb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype('<f4').tobytes())


def clean(points):
    # Sensor no-return zeros must not turn into artificial points after translation.
    return points[np.isfinite(points).all(axis=1) & (points != 0).any(axis=1)]


def accumulate(scans, poses, n, build):
    if len(scans) < n:
        raise ValueError('INSUFFICIENT_PAST_SCANS')
    chosen = scans[-n:][::-1]  # current is author's window[0]; no future member
    if any(len(s['points']) == 0 for s in chosen):
        raise ValueError('EMPTY_MEMBER_SCAN')
    stamps = [s['stamp_ns'] for s in chosen]
    if any(not 0 < a-b <= 150_000_000 for a, b in zip(stamps, stamps[1:])):
        raise ValueError('NONCONTIGUOUS_SCANS')
    if any(s['end_ns'] > chosen[0]['end_ns'] for s in chosen):
        raise ValueError('FUTURE_SENSOR_POINT')
    tick = time.perf_counter()
    result = build(lambda i: chosen[i]['points'], list(range(n)), poses[-n:][::-1], list(range(n)))
    return result, (time.perf_counter()-tick)*1000


def main():
    import rosbag
    import rospy
    p = argparse.ArgumentParser()
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    if a.out.exists():
        raise RuntimeError('Do not overwrite an existing P15 experiment')
    assert subprocess.check_output(['git', '-C', str(AUTHOR), 'rev-parse', 'HEAD'], text=True).strip() == PIN
    assert not subprocess.check_output(['git', '-C', str(AUTHOR), 'status', '--porcelain'], text=True)
    docs = a.repo / 'docs'
    frozen = json.loads((docs / 'p11_mature_solid_prior_map_integration_r1/input_freeze.json').read_text())
    assert sha(MAPPING) == frozen['mapping_state_poses']['sha256']
    # Verify current N1 inputs before consuming newly built N5/N10 data.
    for name in ('references', 'queries'):
        for r in csv.DictReader((docs / f'p14_inlier_real_data_baseline_r1/{name}.csv').open()):
            if name == 'references' or int(r['id']) in QUERIES:
                assert sha(r['path']) == r['sha256']
    states = np.loadtxt(MAPPING)
    assert np.all(np.diff(states[:, 0]) > 0)
    state_ns = np.rint(states[:, 0]*1e9).astype(np.int64).tolist()
    ndt = list(csv.DictReader((P11 / 'control/ndt.csv').open()))
    assert len(ndt) == 5817
    ndt_by_ns = {int(round(float(r['lidar_header_stamp'])*1e9)): r for r in ndt}
    refrows = list(csv.DictReader((docs / 'p11_mature_solid_prior_map_integration_r1/database_build.csv').open()))
    queryrows = {int(r['id']): r for r in csv.DictReader((docs / 'p14_inlier_real_data_baseline_r1/queries.csv').open())}
    a.out.mkdir(parents=True)
    (a.out / 'freeze.json').write_text(json.dumps({
        'protocol': 'P15_CAUSAL_CURRENT_ANCHORED_APPROXIMATE_V1', 'queries': QUERIES, 'N': [5, 10],
        'upstream_sha': PIN, 'parameters': 'unchanged P14 official default YAML / min32',
        'ordering': 'current, previous1,...; original author build_submap; same for DB/query',
        'reference_pose': 'APPROXIMATE_BODY_AS_LIDAR: logged map_T_imu, current documented R_il=I assumption; historical extrinsic not certified; lever arm omitted',
        'reference_time': 'latest mapping state <= each raw scan end, hold-last max0.25s',
        'query_pose': 'APPROXIMATE_RAW_SCAN_POSE: corresponding P11 NDT final_used map_T_lidar; midpoint pose used on undeskewed scan',
        'future_data': 'FORBIDDEN: member scan end and pose timestamps <= anchor scan end',
        'zero_guard': 'drop raw zero/nonfinite before transforming; no-return zeros are not geometric points',
        'boundary_policy': 'reject incomplete, empty, missing-state or noncontiguous windows; preserve original reference IDs',
        'maximum_scan_gap_s': .15, 'maximum_mapping_pose_age_s': .25,
        'gicp': 'NOT_RUN', 'gt_loaded': False,
        'historical_bags': {k: frozen[k] for k in ('raw_mapping_bag', 'raw_query_bag')},
        'whole_bag_hash_recomputed': False,
        'pose_input_hashes': {str(MAPPING): sha(MAPPING), str(P11/'control/ndt.csv'): sha(P11/'control/ndt.csv')},
        'author_submaps_sha256': sha(AUTHOR/'inlier/eval/submaps.py')}, indent=2)+'\n')
    build = author_submap()
    ledgers, members, inputs = [], [], {(side, n): [] for side in ('references', 'queries') for n in (5, 10)}
    for side in ('references', 'queries'):
        path = frozen['raw_mapping_bag' if side == 'references' else 'raw_query_bag']['path']
        rows = refrows if side == 'references' else [queryrows[q] for q in QUERIES]
        with rosbag.Bag(path) as bag:
            for row in rows:
                ident = int(row['id'])
                target = int(round(float(row['stamp_sec'])*1e9)) if side == 'references' else int(row['stamp'])
                candidates = []
                # Indexed read only the bounded past window and record-time slack.
                for _, msg, record in bag.read_messages(topics=['/livox/lidar'],
                        start_time=rospy.Time.from_sec(target/1e9-1.5),
                        end_time=rospy.Time.from_sec(target/1e9+.3)):
                    ns = msg.header.stamp.to_nsec()
                    if ns > target+512:  # floating CSV rounding only, not a future scan
                        continue
                    points = np.asarray([(p.x, p.y, p.z) for p in msg.points], dtype=np.float32)
                    assert len(points) == msg.point_num
                    end = ns + max(p.offset_time for p in msg.points)
                    candidates.append(dict(stamp_ns=ns, end_ns=end, points=clean(points), raw=len(points), record_ns=record.to_nsec()))
                candidates.sort(key=lambda s: s['stamp_ns'])
                if not candidates or abs(candidates[-1]['stamp_ns']-target) > 512:
                    raise RuntimeError(f'Missing frozen anchor {side}/{ident}')
                scans = candidates[-10:]
                poses, evidence = [], []
                for scan in scans:
                    if side == 'references':
                        si = bisect.bisect_right(state_ns, scan['end_ns'])-1
                        if si < 0 or scan['end_ns']-state_ns[si] > 250_000_000:
                            poses.append(None); evidence.append(None); continue
                        s = states[si]
                        poses.append(matrix(s[1:4], s[4:8])); evidence.append(state_ns[si])
                    else:
                        # Match by sensor header, never by callback wall time or a future EKF state.
                        match = min((k for k in ndt_by_ns if abs(k-scan['stamp_ns']) <= 512), default=None)
                        if match is None:
                            poses.append(None); evidence.append(None); continue
                        r = ndt_by_ns[match]
                        pose_ns = int(round(float(r['scan_mid_stamp'])*1e9))
                        assert pose_ns <= scan['end_ns']+512
                        poses.append(matrix([float(r['final_used_t'+k]) for k in 'xyz'],
                                            [float(r['final_used_q'+k]) for k in 'xyzw']))
                        evidence.append(pose_ns)
                for n in (5, 10):
                    folder = a.out/f'N{n}'
                    folder.mkdir(exist_ok=True)
                    record = dict(side=side, id=ident, n=n, stamp_ns=scans[-1]['stamp_ns'],
                                  sensor_cutoff_ns=scans[-1]['end_ns'], status='OK', accumulated_ms=None,
                                  member_count=min(n, len(scans)), raw_points=sum(s['raw'] for s in scans[-n:]))
                    try:
                        if any(pose is None for pose in poses[-n:]):
                            raise ValueError('MISSING_CAUSAL_MAPPING_OR_NDT_POSE')
                        cloud, elapsed = accumulate(scans, poses, n, build)
                        output = folder/f'{side}_{ident}.pcd'
                        write_pcd(output, cloud)
                        record.update(accumulated_ms=elapsed, cleaned_points=len(cloud), sha256=sha(output))
                        inputs[side, n].append(dict(id=ident, stamp=str(scans[-1]['stamp_ns']), path=str(output), sha256=record['sha256']))
                        for s, pose, pose_ns in zip(scans[-n:], poses[-n:], evidence[-n:]):
                            m = dict(side=side, id=ident, n=n, scan_stamp_ns=s['stamp_ns'], scan_end_ns=s['end_ns'], pose_stamp_ns=pose_ns,
                                     raw_points=s['raw'], clean_points=len(s['points']))
                            m.update({f'T{k}': float(v) for k, v in enumerate(pose.ravel())})
                            members.append(m)
                    except ValueError as e:
                        record.update(status=str(e), cleaned_points=None, sha256=None)
                    ledgers.append(record)
                if ident % 50 == 0 or side == 'queries':
                    print('INPUT', side, ident, ledgers[-2]['status'], ledgers[-1]['status'], flush=True)
    for (side, n), rows in inputs.items():
        write_csv(a.out/f'N{n}/{side}.csv', rows, ['id', 'stamp', 'path', 'sha256'])
    write_csv(a.out/'input_ledger.csv', ledgers)
    write_csv(a.out/'member_poses.csv', members)
    print('CAUSAL_INPUT_BUILD_COMPLETE', {str(k): len(v) for k, v in inputs.items()}, flush=True)


if __name__ == '__main__':
    main()
