#!/usr/bin/env python3
"""Freeze one real scan with Y and inverse(Y); GT is a later diagnostic only."""
import argparse
import csv
from decimal import Decimal
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
P36 = WS/'docs/p36_corridor01_world_planar_test'
spec = importlib.util.spec_from_file_location('p36_helpers', P36/'prepare_hypotheses.py')
helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
prior = helper.prior
sha, certificate = helper.sha, helper.certificate


def direct_poses(Y):
    """No GT parameter or extrinsic: exactly two direct placements."""
    result = np.array([Y, np.linalg.inv(Y)], dtype='<f8')
    for T in result:
        certificate(T)
    return result


def ndt_eligible(row):
    values = np.array([float(row[k]) for k in ['nn_median_m', 'nn_p90_m', 'coverage_0_5m']])
    return bool(np.isfinite(values).all() and 0 <= values[0] <= 1 and
                0 <= values[1] <= 3 and .5 <= values[2] <= 1)


def main(output):
    if output.exists():
        raise ValueError('NO_OVERWRITE')
    old = json.loads((P33/'input_freeze.json').read_text())
    paths = dict(map=Path(old['target_map']), GT=prior.ROOT/'gt/corridor01_gt.txt',
                 YAML=prior.ROOT/'initial_pose/corridor01.yaml', index=prior.DATA/'raw_timed_scan_index.csv')
    expected = dict(map=old['target_map_SHA256'], GT=old['GT_SHA256'], YAML=old['YAML_SHA256'],
                    index=old['P34_large_file_evidence'][str(paths['index'])]['SHA256'])
    hashes = {}
    for key, path in paths.items():
        actual = sha(path)
        if actual != expected[key]:
            raise ValueError('INPUT_SHA_CHANGED_'+key)
        hashes[str(path)] = dict(SHA256=actual, bytes=path.stat().st_size, fresh=True)
    for name, evidence in old['P34_large_file_evidence'].items():
        if Path(name).stat().st_size != evidence['bytes']:
            raise ValueError('LARGE_INPUT_SIZE_CHANGED')
        if name not in hashes:
            hashes[name] = dict(evidence, fresh=False, inherited_from='P34 full validation')
    if sha(helper.PROBE) != helper.PROBE_SHA:
        raise ValueError('PROBE_SHA')
    with paths['index'].open() as stream:
        scans = list(csv.DictReader(stream))
    prior.validate_scan_order(scans)
    if len(scans) != 2777:
        raise ValueError('SCAN_COUNT')
    first = int(scans[0]['scan_start_ns']); target = first+67_000_000_000
    index, row = prior.nearest(scans, target)
    ns, end = int(row['scan_start_ns']), int(row['scan_end_ns'])
    if (index, str(ns)) != helper.FIXED[0]:
        raise ValueError('FROZEN_RAW665_IDENTITY')
    storage = cv2.FileStorage(str(paths['YAML']), cv2.FILE_STORAGE_READ)
    try:
        R = storage.getNode('extrinsicRotation_world_darpa').mat()
        t = storage.getNode('extrinsicTranslation_world_darpa').mat()
    finally:
        storage.release()
    if R is None or t is None or R.shape != (3, 3) or t.size != 3:
        raise ValueError('ACTIVE_YAML_MATRIX')
    Y = np.eye(4); Y[:3, :3] = R; Y[:3, 3] = t.ravel()
    if not np.array_equal(Y, old['T_yaml_unknown_semantics']):
        raise ValueError('YAML_VALUES_CHANGED')
    poses = direct_poses(Y)
    with rosbag.Bag(str(prior.DECODED)) as bag:
        records = list(bag.read_messages(topics=['/p10/corridor01/raw_timed_points'], raw=True,
                           start_time=prior.stamp(end), end_time=prior.stamp(end)))
        if len(records) != 1:
            raise ValueError('EXACT_SCAN_RECORD')
        _, payload, record = records[0]
        msg = PointCloud2().deserialize(payload[1])
        count = int(row['cloud_point_count'])
        if (msg.header.stamp.to_nsec() != ns or record.to_nsec() != end or
                msg.header.frame_id != 'cmu_rc2_velodyne' or msg.point_step != 24 or
                msg.width*msg.height != count or len(msg.data) != count*24 or
                msg.row_step != msg.width*24 or msg.is_bigendian):
            raise ValueError('RAW_SCAN_LAYOUT_OR_IDENTITY')
        rich = np.frombuffer(msg.data, dtype=prior.RICH_DTYPE)
        raw_cache = np.memmap(prior.DATA/'raw_timed_points.bin', mode='r', dtype=prior.RAW_POINT_DTYPE)
        canonical, _, _ = prior.make_cloud(row, raw_cache)
        compact = np.empty(count, dtype=prior.RAW_DTYPE)
        for name in compact.dtype.names:
            compact[name] = rich[name]
        if compact.tobytes() != canonical.data:
            raise ValueError('CANONICAL_XYZ_TIME_PARITY')
        xyz = np.column_stack([rich[n] for n in ['x', 'y', 'z']]).astype('<f4').tobytes()
    if __import__('hashlib').sha256(xyz).hexdigest() != old['samples'][0]['raw_XYZ_SHA256']:
        raise ValueError('P33_ORDERED_RAW_XYZ_PARITY')
    p33_source = WS/'.p33_frame_relation.a9l5g3/input_fixed/67/geometry/preprocessed_source.xyz'
    p36_source = WS/'.p36_world_planar.gDJzEB/input/67/geometry/preprocessed_source.xyz'
    if p33_source.read_bytes() != p36_source.read_bytes():
        raise ValueError('P33_P36_SOURCE_MISMATCH')
    output.mkdir(parents=True)
    with (output/'selected_raw.xyz').open('xb') as stream:
        stream.write(xyz)
    with (output/'hypotheses.f64').open('xb') as stream:
        stream.write(poses.tobytes())
    frozen = dict(task='P38-CORRIDOR01-OFFICIAL-INITIAL-POSE-VALIDATION',
        input_hashes=hashes, target_map=str(paths['map']), Y=Y.tolist(), inverse_Y=poses[1].tolist(),
        Y_certificate=certificate(Y), inverse_Y_certificate=certificate(poses[1]),
        YAML_original_text=paths['YAML'].read_text(), YAML_sensor_reference_certified=False,
        YAML_direction_certified=False, comment_s67_reference_certified=False,
        translation_unit='Assumed metres consistently with point XYZ; YAML itself has no explicit unit',
        source_mode='RAW_HEADER_REFERENCE_NO_DESKEW', raw_frame_index=index, scan_ns=str(ns),
        scan_end_ns=str(end), raw_point_count=count, first_LiDAR_ns=str(first),
        target_sensor_ns=str(target), relative_sensor_s=str(Decimal(ns-first)/1_000_000_000),
        target_error_ns=ns-target, raw_XYZ_SHA256=sha(output/'selected_raw.xyz'),
        P33_preprocessed_source=str(p33_source), P36_preprocessed_source=str(p36_source),
        preprocessed_XYZ_SHA256=sha(p33_source), candidates_derived_from_GT=False,
        GT_text_file_poses_parsed_before_candidate_freeze=False,
        P33_archive_loaded_before_freeze_including_unused_GT_matrices=True,
        probe=str(helper.PROBE), probe_SHA256=sha(helper.PROBE),
        protocol_SHA256=sha(HERE/'PROTOCOL.md'), large_SHA_freshly_recomputed=False)
    # Persist candidates before parsing or interpolating the GT text-file poses.
    prior.write(output/'input_freeze.json', frozen)
    time_rows = [dict(event='sensor_header_target', raw_index='', header_ns=str(target), record_ns='',
                     relative_sensor_s='67', Y_time_binding_certified=False),
                 dict(event='selected_nearest_sensor_header', raw_index=index, header_ns=str(ns), record_ns=str(end),
                     relative_sensor_s=frozen['relative_sensor_s'], Y_time_binding_certified=False)]
    for key in ['original_bag_time', 'decoded_bag_time']:
        evidence = old[key]['first_LiDAR_after_bag_record_start_plus67']
        stamp = int(evidence['header_ns'])
        matching = next(i for i, r in enumerate(scans, 1) if int(r['scan_start_ns']) == stamp)
        time_rows.append(dict(event=key+'_record_offset_67_P33_evidence', raw_index=matching,
            header_ns=str(stamp), record_ns=evidence['record_ns'],
            relative_sensor_s=old[key]['sensor_header_offset_at_record67_s'], Y_time_binding_certified=False))
    with (output/'time_binding.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(time_rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(time_rows)
    GT, bracket = prior.at(prior.load_gt(paths['GT']), ns)
    prior.write(output/'gt_diagnostic.json', dict(scan_ns=str(ns), GT_covers_scan_header=True,
        GT_bracket=bracket, GT_at_scan_header=GT.tolist(), used_for_initialization=False,
        translation_vector_numeric_difference_m_NOT_localization_error=float(np.linalg.norm(Y[:3, 3]-GT[:3, 3])),
        rotation_block_numeric_difference_fro_NOT_rotation_error=float(np.linalg.norm(Y[:3, :3]-GT[:3, :3]))))
    print(json.dumps(dict(frame=index, scan_ns=str(ns), relative_sensor_s=frozen['relative_sensor_s'],
                         certificate=frozen['Y_certificate'], candidates_use_GT=False), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    main(**vars(parser.parse_args()))
