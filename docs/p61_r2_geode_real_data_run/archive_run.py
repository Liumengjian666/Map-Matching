#!/usr/bin/env python3
"""Archive small frozen results only; leave bag/PCD/NPY/binaries on internal disk."""
import hashlib
import json
from pathlib import Path
import shutil
import csv

DOC = Path(__file__).resolve().parent
DATA = Path('/home/jian/geode/Shield_tunnel1_gamma')
RUN = Path('/home/jian/livox_ws/p61_r2_geode_runs')


def table(path):
    with path.open() as stream: return list(csv.DictReader(stream))


def archive(source, name):
    destination = DOC / name
    if destination.exists(): raise FileExistsError(destination)
    shutil.copyfile(source, destination)
    payload = destination.read_bytes()
    return {'source': str(source), 'archive': name, 'bytes': len(payload), 'sha256': hashlib.sha256(payload).hexdigest()}


def main():
    records = []
    for source, name in ((DATA / 'input_freeze.json', 'bag_identity.json'), (DATA / 'rosbag_info.yaml', 'rosbag_info.yaml'),
                         (DATA / 'inspection/inspection.json', 'bag_inspection.json'),
                         (DATA / 'inspection/message_definitions.json', 'message_definitions.json'),
                         (DATA / 'inspection/scan_ledger.csv', 'scan_ledger.csv'),
                         (RUN / 'time_contract_audit.json', 'time_contract_audit.json')):
        records.append(archive(source, name))
    for phase in ('smoke', 'screen'):
        for path in sorted((RUN / phase).glob('*.csv')): records.append(archive(path, phase + '_' + path.name))
        for name in ('sample_manifest.csv', 'selection_freeze.json'):
            records.append(archive(RUN / f'{phase}_inputs' / name, phase + '_' + name))
        for suffix in ('resource.txt', 'input_freeze.json', 'output_freeze.json'):
            records.append(archive(RUN / f'{phase}_{suffix}', phase + '_' + suffix))
    for name in ('geometry_comparison.csv', 'weak_direction_records.csv', 'smoke_screen_overlap_parity.csv',
                 'curvature_support_time.png', 'weak_axes_and_spectrum.png'):
        records.append(archive(RUN / 'analysis' / name, name))
    result = json.loads((RUN / 'analysis/results.json').read_text())
    result.update(start_sha='ed31ade24a6bc7a36720c65321997fba1ef93ab4',
                  code_sha='25b24e607d50b9859b6609861f58a1ca4c7fa64a',
                  binary_sha256=hashlib.sha256((RUN / 'build/native_geometry').read_bytes()).hexdigest(),
                  bag_identity=json.loads((DATA / 'input_freeze.json').read_text()),
                  bag_integrity=json.loads((DATA / 'inspection/inspection.json').read_text()),
                  time_contract=json.loads((RUN / 'time_contract_audit.json').read_text()),
                  fusion_extrinsic_approved=False, independent_tunnel_map_available=False,
                  legal_map_initialization_available=False, push_executed=False,
                  original_global_timestamps_modified=False)
    for phase in ('smoke', 'screen'):
        rows = table(RUN / phase / 'hessian.csv')
        low = [r for r in rows if r['comparable'] == '1' and r['spd'] == '1' and float(r['ratio6']) <= .001]
        result['phases'][phase]['low_6d_component_only_count'] = len(low)
        result['phases'][phase]['translation_component_only_count'] = sum(float(r['ratio_translation']) <= .01 for r in rows if r['comparable'] == '1' and r['spd'] == '1')
        groups, current = [], []
        for row in rows + [None]:
            if row is not None and row in low: current.append(row)
            elif current:
                groups.append({'start_index': int(current[0]['scan_index']), 'end_index': int(current[-1]['scan_index']),
                               'sample_count': len(current), 'span_sec': (int(current[-1]['stamp_ns']) - int(current[0]['stamp_ns'])) * 1e-9,
                               'descriptive_component_only_not_severe': True, 'all_intermediate_raw_indices_measured': phase == 'smoke'})
                current = []
        result['phases'][phase]['low_6d_component_sample_runs'] = groups
    storage = {}
    for phase in ('copy', 'source_hash'):
        root = DATA / 'storage_evidence' / phase
        records.append(archive(root / 'receipt.json', 'storage_' + phase + '_receipt.json'))
        records.append(archive(root / 'monitor.jsonl', 'storage_' + phase + '_monitor.jsonl'))
        records.append(archive(root / 'kernel.log', 'storage_' + phase + '_kernel.log'))
        rows = [json.loads(line) for line in (root / 'monitor.jsonl').read_text().splitlines()]
        longest = count = 0
        for row in rows:
            count = count + 1 if row['worker_state'] == 'D' else 0; longest = max(longest, count)
        storage[phase] = {'receipt': json.loads((root / 'receipt.json').read_text()),
                          'd_state_samples': sum(r['worker_state'] == 'D' for r in rows),
                          'max_consecutive_d_state_samples': longest,
                          'explicit_storage_faults': sum(r['fault'] is not None for r in rows)}
    result['storage'] = storage
    for name, value in (('results.json', result), ('artifact_manifest.json', records)):
        with (DOC / name).open('x') as stream: json.dump(value, stream, indent=2, allow_nan=False)
    print('ARCHIVED', len(records), 'small artifacts; bag/PCD/NPY/binaries remain internal')


if __name__ == '__main__': main()
