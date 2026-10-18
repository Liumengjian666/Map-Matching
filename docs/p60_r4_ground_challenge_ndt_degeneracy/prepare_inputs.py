#!/usr/bin/env python3
"""Freeze time-uniform neighboring scan pairs; LiDAR-only internal bag access."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

BAG_SHA = '6ef85c76a36bdcef4811d411ac0f09219803377f5b7b9bd0cf2a5258cc826cea'
CONTRACT_SHA = '2550abf04fd631b3c160c63458c52fc969db3ab69393712e7b7c5e9820a50b7d'
SEED = 6042026


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else stream_hash(stream)


def stream_hash(stream):
    h = hashlib.sha256()
    for block in iter(lambda: stream.read(1 << 20), b''):
        h.update(block)
    return h.hexdigest()


def uniform_indices(stamps, count=100):
    stamps = np.asarray(stamps, dtype=np.int64)
    if len(stamps) < count + 1 or np.any(np.diff(stamps) <= 0):
        raise ValueError('insufficient or nonmonotonic scan times')
    rel = (stamps - stamps[0]).astype(np.float64)
    # Source begins at original scan 2. Ties choose the earlier index.
    indices = [int(np.argmin(np.abs(rel[1:] - x))) + 2 for x in np.linspace(rel[1], rel[-1], count)]
    if len(set(indices)) != count:
        raise ValueError('time-uniform selection produced duplicate indices')
    return indices


def prepare(bag_path, contract_path, directory):
    import rosbag
    if digest(bag_path) != BAG_SHA or digest(contract_path) != CONTRACT_SHA:
        raise ValueError('frozen input identity mismatch')
    contract = json.loads(contract_path.read_text())
    records = contract['cloud_records']
    if len(records) != 974:
        raise ValueError('expected 974 original scans')
    selected = uniform_indices([r['stamp_ns'] for r in records])
    needed = set(selected) | {k - 1 for k in selected}
    directory.mkdir(parents=True, exist_ok=False)
    (directory / 'frozen_indices.json').write_text(json.dumps({'seed': SEED, 'source_indices': selected,
        'bag_sha256': BAG_SHA, 'contract_sha256': CONTRACT_SHA, 'gt_read': False}, indent=2))
    saved = {}
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for index, (_, msg, _) in enumerate(bag.read_messages(topics=['/velodyne_points']), 1):
            if index not in needed:
                continue
            record = records[index - 1]
            if msg.header.stamp.to_nsec() != record['stamp_ns'] or hashlib.sha256(msg.data).hexdigest() != record['data_sha256']:
                raise ValueError('original scan differs from frozen R2 contract')
            fields = {f.name: f for f in msg.fields}
            if msg.is_bigendian or any(fields[n].datatype != 7 for n in ('x', 'y', 'z')):
                raise ValueError('unsupported XYZ layout')
            dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                              'offsets': [fields[n].offset for n in ('x', 'y', 'z')], 'itemsize': msg.point_step})
            cloud = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                               strides=(msg.row_step, msg.point_step))
            xyz = np.stack([cloud[n].reshape(-1) for n in ('x', 'y', 'z')], axis=1).astype('<f4')
            path = directory / f'raw_scan_{index}.pcd'
            header = f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n'
            with path.open('xb') as stream:
                stream.write(header.encode('ascii')); stream.write(xyz.tobytes())
            saved[index] = (str(path), digest(path))
    if set(saved) != needed:
        raise ValueError('missing required scans')
    with (directory / 'sample_manifest.csv').open('x', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['scan_index', 'stamp_ns', 'previous_stamp_ns', 'source_path', 'target_path', 'source_pcd_sha256', 'target_pcd_sha256'])
        for k in selected:
            writer.writerow([k, records[k-1]['stamp_ns'], records[k-2]['stamp_ns'], saved[k][0], saved[k-1][0], saved[k][1], saved[k-1][1]])
    print(json.dumps({'pairs': len(selected), 'raw_clouds': len(saved), 'first_index': selected[0], 'last_index': selected[-1]}))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('bag', type=Path); p.add_argument('contract', type=Path); p.add_argument('new_directory', type=Path)
    a = p.parse_args()
    prepare(a.bag, a.contract, a.new_directory)
