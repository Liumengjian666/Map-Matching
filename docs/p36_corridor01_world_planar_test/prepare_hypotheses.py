#!/usr/bin/env python3
"""Freeze two permitted left transforms; reuse P33 input code, never optimize."""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
P33 = WS/'docs/p33_corridor01_complete_frame_relation'
spec = importlib.util.spec_from_file_location('p33_preparation', P33/'prepare.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
FIXED = [(665, '1517157286055073023'), (993, '1517157319135140896'),
         (1488, '1517157369057807922')]
PROBE = WS/'.p34_gt_frame.XDwg1L/build/p34_geometry'
PROBE_SHA = 'b26b25358a769788979120722e2a1502a61ecccf46a654442b83422d225f4571'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def certificate(T):
    T = np.asarray(T)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('NONFINITE_OR_SHAPE')
    R = T[:3, :3]
    result = dict(orthogonality_fro=float(np.linalg.norm(R.T@R-np.eye(3))),
                  determinant=float(np.linalg.det(R)))
    if (not np.array_equal(T[3], [0, 0, 0, 1]) or
            result['orthogonality_fro'] > 1e-5 or abs(result['determinant']-1) > 1e-5):
        raise ValueError('NOT_RIGID_WITHIN_FROZEN_TOLERANCE')
    return result


def hypotheses(Y, GT):
    poses = np.array([Y@GT, np.linalg.inv(Y)@GT])
    for T in poses:
        certificate(T)
    return poses


def main(output, p33_cache):
    if output.exists():
        raise ValueError('NO_OVERWRITE')
    frozen = json.loads((P33/'input_freeze.json').read_text())
    samples = frozen['samples']
    if [(s['raw_frame_index'], s['scan_ns']) for s in samples] != FIXED:
        raise ValueError('FROZEN_FRAME_IDENTITY')
    if sha(PROBE) != PROBE_SHA:
        raise ValueError('PROBE_SHA')
    paths = [(prior.ROOT/'gt/corridor01_gt.txt', frozen['GT_SHA256']),
             (prior.ROOT/'initial_pose/corridor01.yaml', frozen['YAML_SHA256']),
             (Path(frozen['target_map']), frozen['target_map_SHA256'])]
    hashes = {}
    for path, expected in paths:
        actual = sha(path)
        if actual != expected:
            raise ValueError('SMALL_INPUT_SHA_CHANGED')
        hashes[str(path)] = dict(bytes=path.stat().st_size, SHA256=actual, fresh=True)
    for name, evidence in frozen['P34_large_file_evidence'].items():
        path = Path(name)
        if path.stat().st_size != evidence['bytes']:
            raise ValueError('LARGE_INPUT_SIZE_CHANGED')
        hashes[name] = dict(evidence, fresh=False, inherited_from='P34 full validation')
    index = prior.DATA/'raw_timed_scan_index.csv'
    if sha(index) != hashes[str(index)]['SHA256']:
        raise ValueError('INDEX_SHA_CHANGED')
    hashes[str(index)]['fresh'] = True
    with index.open() as stream:
        scans = list(csv.DictReader(stream))
    prior.validate_scan_order(scans)
    yaml_file = paths[1][0]
    storage = cv2.FileStorage(str(yaml_file), cv2.FILE_STORAGE_READ)
    try:
        R = storage.getNode('extrinsicRotation_world_darpa').mat()
        t = storage.getNode('extrinsicTranslation_world_darpa').mat()
    finally:
        storage.release()
    if R is None or t is None or R.shape != (3, 3) or t.size != 3:
        raise ValueError('ACTIVE_YAML_MATRIX')
    Y = np.eye(4); Y[:3, :3] = R; Y[:3, 3] = t.ravel()
    if not np.array_equal(Y, frozen['T_yaml_unknown_semantics']):
        raise ValueError('YAML_MATRIX_CHANGED')
    cert = certificate(Y)
    gt_rows = prior.load_gt(paths[0][0])
    raw_cache = np.memmap(prior.DATA/'raw_timed_points.bin', mode='r', dtype=prior.RAW_POINT_DTYPE)
    output.mkdir(parents=True)
    prepared = []
    with rosbag.Bag(str(prior.DECODED)) as bag:
        for sample in samples:
            ns, end = int(sample['scan_ns']), int(sample['scan_end_ns'])
            row = scans[sample['raw_frame_index']-1]
            if int(row['scan_start_ns']) != ns or int(row['scan_end_ns']) != end:
                raise ValueError('INDEX_FRAME_MISMATCH')
            records = list(bag.read_messages(topics=['/p10/corridor01/raw_timed_points'], raw=True,
                           start_time=prior.stamp(end), end_time=prior.stamp(end)))
            if len(records) != 1:
                raise ValueError('EXACT_SCAN_RECORD')
            _, payload, record = records[0]
            msg = PointCloud2().deserialize(payload[1])
            if (msg.header.stamp.to_nsec() != ns or record.to_nsec() != end or
                    msg.header.frame_id != 'cmu_rc2_velodyne' or msg.point_step != 24 or
                    len(msg.data) != sample['points']*24 or msg.is_bigendian):
                raise ValueError('SOURCE_LAYOUT_OR_IDENTITY')
            rich = np.frombuffer(msg.data, dtype=prior.RICH_DTYPE)
            expected, _, _ = prior.make_cloud(row, raw_cache)
            compact = np.empty(len(rich), dtype=prior.RAW_DTYPE)
            for name in compact.dtype.names:
                compact[name] = rich[name]
            if compact.tobytes() != expected.data:
                raise ValueError('CANONICAL_ORDERED_XYZ_TIME_PARITY')
            xyz = np.column_stack([rich[n] for n in ['x', 'y', 'z']]).astype('<f4').tobytes()
            if hashlib.sha256(xyz).hexdigest() != sample['raw_XYZ_SHA256']:
                raise ValueError('P33_RAW_XYZ_SHA_PARITY')
            GT, bracket = prior.at(gt_rows, ns)
            if bracket != sample['GT_bracket'] or not np.array_equal(GT, sample['T_G_GT']):
                raise ValueError('P33_GT_INTERPOLATION_PARITY')
            folder = output/str(sample['target_s']); folder.mkdir()
            with (folder/'selected_raw.xyz').open('xb') as stream:
                stream.write(xyz)
            poses = hypotheses(Y, GT)
            with (folder/'hypotheses.f64').open('xb') as stream:
                stream.write(poses.astype('<f8').tobytes())
            p33_source = p33_cache/'input_fixed'/str(sample['target_s'])/'geometry/preprocessed_source.xyz'
            prepared.append(dict(sample, A_Y_GT=poses[0].tolist(), B_INVERSE_Y_GT=poses[1].tolist(),
                P33_preprocessed_source=str(p33_source.resolve()),
                P33_preprocessed_XYZ_SHA256=sha(p33_source),
                actual_decoded_scan_and_canonical_cache_parity=True))
    result = dict(task='P36-CORRIDOR01-WORLD-PLANAR-GT-TRANSFORM-TEST',
        Y=Y.tolist(), inverse_Y=np.linalg.inv(Y).tolist(), Y_certificate=cert,
        inverse_Y_certificate=certificate(np.linalg.inv(Y)),
        inverse_roundtrip_max_abs=float(np.max(np.abs(Y@np.linalg.inv(Y)-np.eye(4)))),
        Planar_equals_DARPA_certified=False, GT_equals_T_W_L_certified=False,
        YAML_direction_certified=False, YAML_original_text=yaml_file.read_text(),
        target_map=frozen['target_map'], target_map_SHA256=frozen['target_map_SHA256'],
        source='RAW_HEADER_REFERENCE_NO_DESKEW', samples=prepared, input_hashes=hashes,
        P33_freeze_SHA256=sha(P33/'input_freeze.json'),
        P33_controls_SHA256=sha(P33/'direct_projection.csv'),
        probe=str(PROBE), probe_SHA256=sha(PROBE),
        protocol_SHA256=sha(HERE/'PROTOCOL.md'), large_SHA_freshly_recomputed=False)
    prior.write(output/'input_freeze.json', result)
    print(json.dumps(dict(scans=len(prepared), Y_certificate=cert,
                         inverse_roundtrip_max_abs=result['inverse_roundtrip_max_abs']), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--p33-cache', type=Path, required=True)
    main(**vars(parser.parse_args()))
