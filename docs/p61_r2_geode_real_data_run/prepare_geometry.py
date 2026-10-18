#!/usr/bin/env python3
"""Freeze consecutive smoke or time-uniform native pairs without GT or IMU use."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import rosbag
from livox_contract import decode

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'p60_r4_ground_challenge_ndt_degeneracy'))
from prepare_inputs import uniform_indices


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''): h.update(block)
    return h.hexdigest()


def prepare(bag_path, inspection, phase, output):
    if phase not in ('smoke', 'screen'): raise ValueError('unknown phase')
    with (inspection / 'scan_ledger.csv').open() as stream: rows = list(csv.DictReader(stream))
    summary = json.loads((inspection / 'inspection.json').read_text())
    if not summary['full_raw_message_traversal'] or not summary['all_topic_counts_match_index']:
        raise ValueError('complete input traversal is required')
    selected = list(range(2, 22)) if phase == 'smoke' else uniform_indices([int(r['stamp_ns']) for r in rows])
    output.mkdir(parents=True, exist_ok=False)
    freeze = {'phase': phase, 'source_indices': selected, 'selection': 'consecutive' if phase == 'smoke' else 'uniform_sensor_time',
              'scan_ledger_sha256': sha(inspection / 'scan_ledger.csv'), 'gt_read': False, 'imu_used': False,
              'bag': str(bag_path), 'native_fov_no_crop': True}
    with (output / 'selection_freeze.json').open('x') as stream: json.dump(freeze, stream, indent=2)
    needed = set(selected) | {k - 1 for k in selected}; saved = {}
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for index, (_, raw, _) in enumerate(bag.read_messages(topics=[summary['actual_lidar_topic']], raw=True), 1):
            if index not in needed: continue
            data = raw[1]; row = rows[index - 1]
            if hashlib.sha256(data).hexdigest() != row['serialized_sha256']: raise ValueError('scan identity changed')
            header, points = decode(data)
            if header['stamp_ns'] != int(row['stamp_ns']): raise ValueError('scan clock conflict')
            xyz = np.stack([points[name] for name in ('x', 'y', 'z')], axis=1).astype('<f4')
            pcd, fields = output / f'raw_scan_{index}.pcd', output / f'raw_scan_{index}.npy'
            text = f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n'
            with pcd.open('xb') as stream: stream.write(text.encode()); stream.write(xyz.tobytes())
            with fields.open('xb') as stream: np.save(stream, points, allow_pickle=False)
            saved[index] = {'pcd': str(pcd), 'pcd_sha256': sha(pcd), 'full_fields_npy': str(fields),
                            'npy_sha256': sha(fields), 'header': header, 'serialized_sha256': row['serialized_sha256']}
    if set(saved) != needed: raise ValueError('missing actual scans')
    with (output / 'sample_manifest.csv').open('x', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['scan_index', 'stamp_ns', 'previous_stamp_ns', 'source_path', 'target_path', 'source_pcd_sha256', 'target_pcd_sha256'])
        for k in selected:
            writer.writerow([k, rows[k - 1]['stamp_ns'], rows[k - 2]['stamp_ns'], saved[k]['pcd'], saved[k - 1]['pcd'], saved[k]['pcd_sha256'], saved[k - 1]['pcd_sha256']])
    with (output / 'export_manifest.json').open('x') as stream: json.dump(saved, stream, indent=2)
    print(json.dumps({'phase': phase, 'pairs': len(selected), 'raw_scans': len(saved), 'gt_read': False}))


if __name__ == '__main__': prepare(Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3], Path(sys.argv[4]))
