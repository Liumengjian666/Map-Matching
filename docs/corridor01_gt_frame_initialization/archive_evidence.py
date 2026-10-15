#!/usr/bin/env python3
"""Assemble only small P34 evidence. Does not run NN, NDT, ICP or localization."""
import argparse
import csv
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
START = '17b50a36f743ac1700ce5887c439accf9f9b2401'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save(name, value):
    with (HERE / name).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def table(name, rows):
    with (HERE / name).open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def main(cache, remote_head):
    if len(remote_head) != 40 or any(c not in '0123456789abcdef' for c in remote_head):
        raise ValueError('REMOTE_SHA')
    copies = {
        'input/input_validation.json': 'input_validation.json',
        'input/gt_lineage.json': 'gt_lineage.json',
        'input/selection.json': 'frame_selection.json',
        'input/hypothesis_freeze.json': 'transforms.json',
        'geometry/geometry.csv': 'initial_geometry.csv',
        'geometry/resources.txt': 'geometry_resources.txt',
        'tests.log': 'test_results.txt',
        'build.log': 'build_result.txt',
    }
    for source, destination in copies.items():
        if (HERE / destination).exists():
            raise FileExistsError(destination)
        shutil.copyfile(cache / source, HERE / destination)
    def load(name):
        return json.loads((HERE / name).read_text())
    inputs, selected, transforms = map(load, ['input_validation.json', 'frame_selection.json', 'transforms.json'])
    with (HERE / 'initial_geometry.csv').open() as stream:
        geometry = list(csv.DictReader(stream))
    if len(geometry) != 2 or (cache / 'geometry/ndt.csv').exists():
        raise ValueError('EXECUTION_BUDGET_CHANGED')
    if inputs['status'] != 'FULL_INPUT_REVALIDATION_PASS':
        raise ValueError('INPUT_CONTRACT')
    t0 = inputs['first_sensor_ns']['/p10/corridor01/raw_timed_points']
    events = [
        ('decoded_first_LiDAR', t0, 'actual decoded header'),
        ('decoded_first_IMU', inputs['first_sensor_ns']['/imu/data'], 'actual decoded header'),
        ('historical_init_and_explicit_EKF_seed', 1517157219188978910, 'raw2; integer frozen index/config, not CSV float inversion'),
        ('first_full_IMU_scan_end', selected['first_full_IMU_scan']['end_ns'], 'point-time max'),
        ('GT_first_and_selected_raw8', selected['GT_first_ns'], 'exact GT time, no extrapolation'),
        ('selected_raw8_end', selected['selected_first_full_IMU_and_GT_scan']['end_ns'], 'point-time max'),
        ('historical_initializer_window_end', 1517157224188978910, 'historical first+5s boundary'),
        ('conditional_bag_plus_67s_NOT_official_binding', t0+67000000000, 'only if comment zero means current first LiDAR'),
        ('GT_last', selected['GT_last_ns'], 'actual local GT timestamp'),
    ]
    table('time_correspondence.csv', [dict(event=label, stamp_ns=str(ns),
        relative_to_first_LiDAR_s=str(Decimal(ns-t0)/Decimal(1000000000)), evidence=note)
        for label, ns, note in events])
    save('ndt_test.json', dict(status='NOT_EXECUTED_REFERENCE_AND_WORLD_TRANSFORM_UNCERTIFIED',
        full_ndt_align_calls=0, ICP_calls=0, full_localization_runs=0,
        pre_NN_decision='--NN-only',
        reason='GT export reference and T_M_W are not certified; NN cannot select a physically certified T_map_lidar',
        initial_pose=None, final_pose=None, converged=None, fitness=None))
    resource = (cache / 'geometry/resources.txt').read_text()
    def metric(prefix):
        line = next(line.strip() for line in resource.splitlines() if line.strip().startswith(prefix))
        return line.rsplit(': ', 1)[1]
    wall = metric('Elapsed (wall clock)')
    minutes, seconds = wall.split(':')
    table('resource_summary.csv', [
        dict(stage='complete_input_revalidation', wall_s=inputs['elapsed_s'],
             cpu_user_s='', cpu_system_s='', peak_RSS_KiB=inputs['peak_RSS_KiB'], note='offline validation, not online performance'),
        dict(stage='two_fixed_NN_queries_including_map_load', wall_s=60*int(minutes)+float(seconds),
             cpu_user_s=metric('User time'), cpu_system_s=metric('System time'),
             peak_RSS_KiB=metric('Maximum resident set size'), note='no optimizer calls'),
    ])
    files = [
        WS/'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp',
        WS/'src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp',
        WS/'src/dog_prior_map_localization/src/fusion/ndt_observation.cpp',
        WS/'src/dog_prior_map_localization/src/ros_output.cpp',
        WS/'src/dog_prior_map_localization/include/dog_prior_map_localization/core/frame_conversions.hpp',
        WS/'src/dog_prior_map_localization/scripts/dataset_gt_path_publisher.py',
        WS/'src/dog_prior_map_localization/scripts/corridor01_decode/verify_full.py',
        WS/'src/dog_prior_map_localization/scripts/p32_gt_frame/prepare.py',
        WS/'src/dog_prior_map_localization/scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp',
        WS/'src/dog_prior_map_localization/launch/corridor01_localization.launch',
        Path('/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_first_segment_initializer.cpp'),
        Path('/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter/src/superloc_map_normalizer.cpp'),
    ]
    head = subprocess.check_output(['git', '-C', str(WS), 'rev-parse', 'HEAD'], text=True).strip()
    if head != START or subprocess.check_output(['git', '-C', str(WS), 'diff', START, '--', 'src'], text=True):
        raise ValueError('RUNTIME_SOURCE_CHANGED')
    save('results.json', dict(task='P34-CORRIDOR01-GT-FRAME-INITIALIZATION-R1',
        FINAL_RESULT='INITIALIZATION_FRAME_UNRESOLVED',
        git=dict(START_SHA=START, RUNTIME_CODE_SHA=START, END_SHA=None,
            END_SHA_note='Full final SHA is reported in chat; resolve archive commit via git log -1 --format=%H -- docs/corridor01_gt_frame_initialization',
            REMOTE_HEAD_observed=remote_head, PUSH_EXECUTED=False,
            branch='research/p9-r4-heldout-visual-evidence'),
        input_validation=inputs['status'], counts=inputs['counts'], points=inputs['points'],
        reference_evidence=dict(exact_official_GT_numeric_lineage=True,
            official_IMU_submission_convention=True, GT_physical_reference_certified=False,
            GT_LiDAR_reference_certified=False, T_M_W_certified=False,
            comment_s67_semantics_certified=False),
        runtime_GT_used_for_initialization=False, initialization_transform_bug_confirmed=False,
        runtime_code_changed=False, original_data_changed=False,
        map_XYZ_transformed=True, map_roundtrip_note='paired forward normalization checked, no new map written',
        map_parity=transforms['map_parity'],
        selection=selected, geometry=geometry,
        H1_H2_translation_difference_m=transforms['H1_H2_translation_difference_m'],
        H1_H2_rotation_difference_deg=transforms['H1_H2_rotation_difference_deg'],
        full_ndt_align_calls=0, ICP_calls=0, localization_replays=0, NN_hypotheses=2,
        tests_passed=4, numerical_input_contract=True,
        binary_SHA256=sha(cache/'build/p34_geometry'),
        preprocessed_source_SHA256=sha(cache/'geometry/preprocessed_source.xyz'),
        preprocessed_source_stable_hash='15ade92bba91ca60',
        local_cache=str(cache),
        limitations=['raw single scan without deskew, not archived online Source',
            'conditional W_GT=M, not a certified map/GT transform',
            'no physical GT exporter reference certification',
            'raw2 has no legal GT pose; raw8 substituted and disclosed',
            'standalone CONTROL launch is P18, not P31 C'],
        NEXT='Obtain traceable GT export physical-reference and PCD-to-GT-world binding evidence; no additional search authorized'))
    evidence_hashes = {str(p):dict(SHA256=sha(p), bytes=p.stat().st_size) for p in files}
    for path, old_sha in transforms['source_SHA256'].items():
        evidence_hashes[path] = dict(SHA256=old_sha, note='hash at hypothesis freeze, before geometric queries')
    for path in sorted(HERE.iterdir()):
        if path.is_file() and path.name != 'artifact_hashes.json':
            evidence_hashes[str(path.relative_to(WS))] = dict(SHA256=sha(path), bytes=path.stat().st_size)
    save('artifact_hashes.json', dict(files=evidence_hashes,
        binary=dict(path=str(cache/'build/p34_geometry'), SHA256=sha(cache/'build/p34_geometry')),
        large_input_hashes=inputs['hashes'], self_excluded=True))
    print('Archive assembled; no NN/NDT/ICP/replay executed by assembler.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--remote-head', required=True)
    main(**vars(parser.parse_args()))
