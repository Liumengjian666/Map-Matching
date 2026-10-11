#!/usr/bin/env python3
"""Freeze <=6 P12 frames and unchanged Top3 BEFORE P12B registrations."""
import csv
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def write(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def freeze(workspace, out):
    previous = workspace / 'docs/p12_dcreg_solid_minimal_integration_r1'
    samples = read(previous / 'samples.csv')
    candidates = [r for r in read(previous / 'registration.csv') if r['rank'] != '0']
    by_frame = {r['frame_index']: [c for c in candidates if c['frame_index'] == r['frame_index']]
                for r in samples}

    def distance(sample, candidate):
        return sum((float(sample['initial_guess_t' + axis]) -
                    float(candidate['reference_t' + axis])) ** 2 for axis in 'xyz') ** .5

    # Sequential chronological selection, using only allowed frozen P12 evidence.
    selected, reasons = [], {}
    for category in ('previously_converged', 'near_failed_no_convergence', 'far_failed_no_convergence'):
        eligible = []
        for s in samples:
            f = s['frame_index']
            if f in reasons:
                continue
            cs = by_frame[f]
            converged = any(c['converged'] == '1' for c in cs)
            match = (converged if category == 'previously_converged' else
                     not converged and any(c['converged'] == '0' and
                     (distance(s, c) <= 5 if category.startswith('near') else distance(s, c) > 5)
                     for c in cs))
            if match:
                eligible.append(s)
        if len(eligible) < 2:
            raise RuntimeError('insufficient frozen stratum: ' + category)
        for s in eligible[:2]:
            selected.append(s)
            reasons[s['frame_index']] = category
    selected.sort(key=lambda s: int(s['frame_index']))
    if [int(s['frame_index']) for s in selected] != [16, 45, 124, 708, 1285, 1891]:
        raise RuntimeError('unexpected historical input; do not reselect')
    old_freeze = json.loads((previous / 'freeze.json').read_text())
    for name in ('map', 'database'):
        path = old_freeze['target' if name == 'map' else 'database']
        if sha(path) != old_freeze[name + '_sha256']:
            raise RuntimeError(name + ' parity failure')
    pairing = {r['id']: r for r in read(workspace / 'docs/p11_mature_solid_prior_map_integration_r1/reference_pose_pairing.csv')}
    expected = []
    for s in selected:
        for c in by_frame[s['frame_index']]:
            row = dict(c)
            row['reference_mapping_state_stamp'] = pairing[c['reference_id']]['mapping_state_stamp']
            row['reference_header_minus_state_sec'] = pairing[c['reference_id']]['signed_delta_sec']
            row['selection_category'] = reasons[s['frame_index']]
            expected.append(row)
    out.mkdir(exist_ok=False)
    write(out / 'samples.csv', selected)
    write(out / 'expected_candidates.csv', expected)
    receipt = {'protocol': 'P12B_APPROXIMATE_REFERENCE_STATE_POSE_R1',
               'frozen_at_utc': datetime.now(timezone.utc).isoformat(), 'frames': [int(s['frame_index']) for s in selected],
               'selection': reasons, 'top_k': 3, 'new_registration_budget': 18,
               'A': 'unchanged frozen P12 coarsePosition results (not rerun)',
               'B': 'APPROXIMATE_REFERENCE_STATE_POSE: copy full mapping state; NO claimed extrinsic/time conversion',
               'exact_conversion': False, 'gt_loaded': False, 'parameter_overrides': {},
               'upstream_sha': old_freeze['dcreg_sha'], 'map_sha256': old_freeze['map_sha256'],
               'database_sha256': old_freeze['database_sha256'],
               'samples_sha256': sha(out / 'samples.csv'), 'expected_candidates_sha256': sha(out / 'expected_candidates.csv'),
               'source_sha256': {s['source_pcd']: sha(s['source_pcd']) for s in selected},
               'historical_registration_sha256': sha(previous / 'registration.csv')}
    (out / 'freeze.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    freeze(Path(sys.argv[1]), Path(sys.argv[2]))
