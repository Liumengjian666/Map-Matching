#!/usr/bin/env python3
"""Descriptive P11 retrieval/runtime/parity audit. Never opens GT."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return {'count': len(values), 'mean': float(values.mean()),
            'p95': float(np.quantile(values, .95)), 'max': float(values.max()),
            'min': float(values.min())} if len(values) else {'count': 0}


def same(a, b):
    if a == b:
        return True
    try:
        x, y = float(a), float(b)
        return (math.isnan(x) and math.isnan(y)) or x == y
    except ValueError:
        return False


def compare(a, b, fields):
    differing_rows = 0
    first = None
    for index, (left, right) in enumerate(zip(a, b)):
        diff = [key for key in fields if not same(left[key], right[key])]
        if diff:
            differing_rows += 1
            if first is None:
                first = {'index': index, 'fields': diff,
                         'left': {k: left[k] for k in diff},
                         'right': {k: right[k] for k in diff}}
    return {'count_A': len(a), 'count_B': len(b),
            'compared': min(len(a), len(b)), 'differing_rows': differing_rows,
            'first_difference': first, 'fields': fields,
            'PASS': len(a) == len(b) and differing_rows == 0}


def runtime(root):
    ndt = rows(root / 'ndt.csv')
    fixed = rows(root / 'fixed_lag.csv')
    oosm = rows(root / 'oosm.csv')
    return {
        'ndt_calls': len(ndt), 'optimizer_converged': sum(r['ndt_has_converged'] == '1' for r in ndt),
        'step_limited': sum(r['translation_limited'] == '1' or r['rotation_limited'] == '1' for r in ndt),
        'extra_jointJet': sum(int(r['coupled_jet_calls']) for r in ndt),
        'extra_value_only': sum(int(r['coupled_value_calls']) for r in ndt),
        'feedback': sum(int(r['coupled_feedback_applied']) for r in ndt),
        'ndt_callback_ms': stats([1000 * (float(r['ndt_odom_publish_wall_sec']) - float(r['wall_time']))
                                  for r in ndt]),
        'fixed_lag_status': dict(Counter(r['status'] for r in fixed)),
        'late_count': sum('LATE' in r['status'] for r in fixed),
        'oosm_status': dict(Counter(r['oosm_result'] for r in oosm)),
        'execution': json.loads((root / 'execution.json').read_text()),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    a, b = args.root / 'control', args.root / 'solid_shadow'
    ndt_a, ndt_b = rows(a / 'ndt.csv'), rows(b / 'ndt.csv')
    ndt_fields = ['frame_index', 'lidar_header_stamp', 'cloud_size_raw',
                  'cloud_size_after_filter', 'cloud_hash', 'initial_guess_source',
                  'ndt_fitness', 'ndt_has_converged', 'ndt_iterations',
                  'translation_limited', 'rotation_limited']
    ndt_fields += [k for k in ndt_a[0] if k.startswith(('initial_guess_', 'raw_ndt_', 'final_used_'))
                   and k not in ndt_fields]
    lineage = [[r for r in rows(d / 'lineage.csv') if r['event_type'] in
                ('IMU_HIGH_RATE_PUBLISH', 'CORRECTED_PUBLISH')] for d in (a, b)]
    state_fields = ['event_type', 'state_stamp', 'p_x', 'p_y', 'p_z', 'q_x', 'q_y',
                    'q_z', 'q_w', 'v_x', 'v_y', 'v_z', 'last_ndt_observation_time',
                    'ndt_update_count', 'state_revision', 'high_rate_publish_seq']
    applied = [[r for r in rows(d / 'fixed_lag.csv') if r['status'] == 'APPLIED'] for d in (a, b)]
    release_fields = ['observation_id', 'ndt_stamp', 'deadline_stamp', 'release_state_stamp',
                      'replay_imu_count', 'correction_applied', 'status']
    query = rows(b / 'solid.csv')
    ok = [r for r in query if r['descriptor_status'] == 'OK']
    query_ndt_hash = compare(ndt_b, query, [])
    hash_differences = []
    for i, (n, q) in enumerate(zip(ndt_b, query)):
        if n['cloud_hash'].lstrip('0') != q['source_hash'].lstrip('0') or \
           n['cloud_size_after_filter'] != q['input_source_points'] or \
           abs(float(n['lidar_header_stamp']) - int(q['stamp_ns']) / 1e9) > 3e-7:
            hash_differences.append(i)
    query_ndt_hash.update({'differing_rows': len(hash_differences),
                           'first_difference': hash_differences[:1],
                           'PASS': len(ndt_b) == len(query) and not hash_differences})
    db_rows = np.loadtxt(args.root / 'solid_database.txt', skiprows=1)
    descriptors = db_rows[:, 9:49]
    normalized = descriptors / np.linalg.norm(descriptors, axis=1, keepdims=True)
    cosine = normalized @ normalized.T
    upper = np.triu_indices(len(db_rows), k=1)
    distances = np.linalg.norm(db_rows[:, None, 2:5] - db_rows[None, :, 2:5], axis=2)
    result = {
        'GT_LOADED': False, 'ACCURACY_GAIN': 'NOT_EVALUATED',
        'reference_count': len(db_rows), 'database_text_bytes': (args.root / 'solid_database.txt').stat().st_size,
        'descriptor_payload_bytes': len(db_rows) * 100 * 8,
        'descriptor_pose_payload_bytes': len(db_rows) * (100 * 8 + 16 * 8 + 8 + 8),
        'runtime_A': runtime(a), 'runtime_B': runtime(b),
        'parity': {'ndt': compare(ndt_a, ndt_b, ndt_fields),
                   'oosm_applied': compare(applied[0], applied[1], release_fields),
                   'ekf_states': compare(lineage[0], lineage[1], state_fields),
                   'query_is_exact_ndt_source': query_ndt_hash},
        'queries': len(query), 'successful_descriptors': len(ok),
        'status_counts': dict(Counter(r['descriptor_status'] for r in query)),
        'top5_queries': sum(bool(r['top5_id']) for r in ok),
        'unique_top1_reference_ids': len({r['top1_id'] for r in ok}),
        'top1_similarity': stats([r['top1_similarity'] for r in ok]),
        'top1_minus_top2_similarity': stats([float(r['top1_similarity']) - float(r['top2_similarity']) for r in ok]),
        'topk_reference_spread_m': stats([r['topk_reference_pose_spread_m'] for r in ok]),
        'queries_with_topk_reference_separation_gt5m': sum(float(r['topk_reference_pose_spread_m']) > 5 for r in ok),
        'descriptor_ms': stats([r['descriptor_ms'] for r in query]),
        'search_ms': stats([r['search_ms'] for r in query]),
        'callback_compute_ms_excluding_csv_flush': stats([r['callback_compute_ms'] for r in query]),
        'reference_exact_normalized_duplicate_pairs': int((cosine[upper] >= 1 - 1e-12).sum()),
        'reference_cosine_gt099_pairs_gt5m': int(((cosine[upper] > .99) & (distances[upper] > 5)).sum()),
        'retrieval_correctness': 'NOT_EVALUATED_NO_INDEPENDENT_MAP_FRAME_PLACE_LABELS',
        'reference_pose_contract': 'FAST_LIVO2_MAPPING_STATE_POSE_NOT_EXACT_LIDAR_POSE',
        'semantic_note': 'Place candidates only; separated references do not prove distinct NDT basins or ambiguity.',
    }
    result['mainchain_parity_certified'] = (
        all(r['PASS'] for r in result['parity'].values()) and
        not result['runtime_A']['late_count'] and not result['runtime_B']['late_count'])
    result['FINAL_RESULT'] = ('MATURE_SOLID_SHADOW_INTEGRATION_VERIFIED'
                              if result['mainchain_parity_certified'] else
                              'SOLID_QUERY_FUNCTIONAL_MAINCHAIN_PARITY_NOT_CERTIFIED')
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'parity'}, indent=2))


if __name__ == '__main__':
    main()
