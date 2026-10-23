#!/usr/bin/env python3
"""Select the earliest persistent suspect, first three frames, two terminal seeds each."""
import csv
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
BUILD = Path('/home/jian/livox_ws/p62_r19_ndt_basin_build/probe_stability')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    evaluation = json.loads((HERE / 'EVALUATION_SUMMARY.json').read_text())
    native = json.loads((HERE / 'NATIVE_OUTPUT_FREEZE.json').read_text())
    assert native['total_native_full_aligns'] == 480 and native['extra_stability_aligns_at_freeze'] == 0
    runs = evaluation['persistent_suspect_runs']
    assert runs, 'no persistent suspect: no probe allowed'
    rank = {'WEAK': 0, 'STRONG': 1}
    selected = min(runs, key=lambda x: (rank[x['role']], x['arm'], x['start_frame']))
    role, arm = selected['role'], selected['arm']
    with (HERE / role.lower() / 'formal_run_01/NDT_RESULTS.csv').open(newline='') as f:
        originals = {(r['arm'], int(r['frame'])): r for r in csv.DictReader(f)}
    requests = []
    for frame in range(selected['start_frame'], selected['start_frame'] + 3):
        for seed_arm in ('C0', arm):
            r = originals[(seed_arm, frame)]
            assert r['quality_feedback'] == r['converged'] == '1'
            requests.append({'role': role, 'arm': seed_arm, 'frame': frame,
                             'raw_ordinal': r['raw_ordinal'], 'stamp_ns': r['stamp_ns']})
    assert len(requests) == 6
    plan_csv = HERE / 'STABILITY_PROBE_PLAN.csv'
    with plan_csv.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(requests[0]), lineterminator='\n')
        writer.writeheader();writer.writerows(requests)
    plan = {'selection_rule': 'first persistent run ordered WEAK before STRONG, C1/C2/C3, earliest frame; first 3 frames; C0 and disturbed raw terminal starts',
            'persistent_run': selected, 'requests': requests, 'extra_native_align_budget': 8,
            'planned_extra_native_aligns': 6,
            'native_output_freeze_sha256': sha(HERE / 'NATIVE_OUTPUT_FREEZE.json'),
            'plan_csv_sha256': sha(plan_csv),
            'probe_source_sha256': sha(HERE / 'tools/probe_stability.cpp'),
            'probe_binary_sha256': sha(BUILD), 'reference_used_only_to_activate_preregistered_probe': True}
    with (HERE / 'STABILITY_PROBE_PLAN.json').open('x') as f:
        json.dump(plan, f, indent=2, allow_nan=False);f.write('\n')
    print('STABILITY_PLAN_FROZEN', sha(HERE / 'STABILITY_PROBE_PLAN.json'), selected)


if __name__ == '__main__':
    main()
