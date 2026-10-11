#!/usr/bin/env python3
"""Check measured P11 artifacts and hash the small, committed archive."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess

from analyze_shadow_pair import compare, rows

START = '0c594b11e68ed0e7f3a2b0b00aa9e6ce7584298e'
CODE = '18c702afa3af1750ffecac27c668dca3eb176075'
BRANCH = 'research/p9-r4-heldout-visual-evidence'
NEXT = 'CERTIFY_DETERMINISTIC_SOLID_SHADOW_REPLAY'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--archive', type=Path, required=True)
    args = parser.parse_args()
    results = json.loads((args.archive / 'results.json').read_text())
    audit = json.loads((args.archive / 'run_audit.json').read_text())
    frozen = json.loads((args.archive / 'input_freeze.json').read_text())
    assert frozen['code_commit'] == CODE
    assert not results['GT_LOADED'] and not audit['GT_LOADED']
    for name in ('runtime_A', 'runtime_B'):
        assert results[name]['ndt_calls'] == 5817
        assert results[name]['feedback'] == 0
        assert results[name]['extra_jointJet'] == 0
    assert results['queries'] == 5817 and results['reference_count'] == 389
    query = rows(args.archive / 'query_results.csv')
    assert sum(r['descriptor_status'] == 'OK' for r in query) == results['successful_descriptors']
    for r in query:
        if r['descriptor_status'] == 'OK':
            assert int(r['scored_candidates']) == 389
            scores = [float(r[f'top{k}_similarity']) for k in range(1, 6)]
            assert all(0 <= s <= 1 + 1e-12 for s in scores)
            assert scores == sorted(scores, reverse=True)
            assert len({r[f'top{k}_id'] for k in range(1, 6)}) == 5
    assert results['parity']['ndt']['PASS']
    assert results['parity']['query_is_exact_ndt_source']['PASS']
    for mode in ('control', 'solid_shadow'):
        assert audit[mode]['diagnostic_rows'] == 5817
    # Verify original mathematical/production implementation content is unchanged.
    protected = [
        'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp',
        'src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp',
        'src/dog_prior_map_localization/src/imu_processor.cpp',
        'src/dog_prior_map_localization/src/fusion/ndt_observation.cpp',
        'src/dog_prior_map_localization/src/core/oosm_fixed_lag_scheduler.cpp',
        'src/dog_prior_map_fastlio2_frontend_exp',
    ]
    changed = subprocess.check_output(
        ['git', '-C', str(args.workspace), 'diff', '--name-only', START, '--', *protected],
        text=True).strip()
    assert not changed, changed
    identity_checks = []
    for item in frozen['source_and_binaries'] + frozen['reference_pcds'] + [
            frozen['map'], frozen['mapping_state_poses'], frozen['database']]:
        path = Path(item['path'])
        assert path.stat().st_size == item['bytes'] and sha(path) == item['sha256'], path
        identity_checks.append(str(path))
    for item in audit['external_run_artifacts']:
        path = Path(item['path'])
        assert path.stat().st_size == item['bytes'] and sha(path) == item['sha256'], path
    # Preserve a compact, per-frame nominal parity ledger; raw full CSVs stay external.
    a, b = [rows(args.root / mode / 'ndt.csv') for mode in ('control', 'solid_shadow')]
    fields = results['parity']['ndt']['fields']
    with (args.archive / 'nominal_parity.csv').open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['frame_index', 'lidar_header_stamp', 'source_count', 'source_hash',
                         'differing_fields', 'exact_numerical_parity'])
        for left, right in zip(a, b):
            receipt = compare([left], [right], fields)
            writer.writerow([left['frame_index'], left['lidar_header_stamp'],
                             left['cloud_size_after_filter'], left['cloud_hash'],
                             receipt['first_difference'], receipt['PASS']])
    results.update({
        'TASK': 'P11-MATURE-SOLID-PRIOR-MAP-INTEGRATION-R1',
        'Repository': 'Liumengjian666/Map-Matching', 'Branch': BRANCH,
        'START_SHA': START, 'CODE_SHA': CODE, 'REMOTE_HEAD_observed': START,
        'PUSH_EXECUTED': False, 'NEXT': NEXT,
        'upstream_sha': '9a4f2895e5211f69fbbd6d4732577e1a67e9b882',
        'license': 'BSD-3-Clause', 'sensor': 'Livox MID-360',
        'raw_reference_sequence': 'Loop2 independent mapping',
        'raw_query_sequence': 'Loop3 independent recording',
        'verified_ndt_nonlocal_terminal_count': 'NOT_EVALUATED',
        'baseline_deskew_enabled': False,
        'cost_audit': {k: v for k, v in audit.items() if k != 'external_run_artifacts'},
        'comparison_is_not_accuracy_validation': True,
    })
    (args.archive / 'results.json').write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')
    # Canonicalize archive copies only. Frozen external CSVs/logs retain their
    # original bytes and hashes; no numeric fields or experimental data change.
    for path in args.archive.rglob('*'):
        if path.is_file() and path.suffix in ('.csv', '.log'):
            lines = path.read_text().splitlines()
            while lines and not lines[-1].strip():
                lines.pop()
            path.write_text('\n'.join(line.rstrip(' \t') for line in lines) + '\n')
    csv_counts = {}
    json_files = []
    for path in sorted(args.archive.rglob('*')):
        if path.suffix == '.csv':
            with path.open(newline='') as stream:
                reader = csv.reader(stream)
                header = next(reader)
                count = 0
                for row in reader:
                    assert len(row) == len(header), (path, count, len(row), len(header))
                    count += 1
                csv_counts[str(path.relative_to(args.archive))] = count
        if path.suffix == '.json' and path.name not in ('artifact_hashes.json', 'audit_receipt.json'):
            json.loads(path.read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            json_files.append(str(path.relative_to(args.archive)))
    receipt = {'CSV_JSON_HASH_AUDIT': 'PASS', 'source_and_binary_identity_checked': len(identity_checks),
               'external_run_artifacts_checked': len(audit['external_run_artifacts']),
               'csv_data_rows': csv_counts, 'json_files': json_files,
               'protected_implementation_changed': False, 'GT_LOADED': False,
               'mainchain_parity': 'FAIL',
               'archive_text_normalization': 'CSV/log archive copies use LF and no trailing whitespace; external original bytes remain unchanged.',
               'note': 'Artifact audit PASS does not override real EKF/OOSM parity FAIL.'}
    (args.archive / 'audit_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    hashes = {str(path.relative_to(args.archive)): {'bytes': path.stat().st_size, 'sha256': sha(path)}
              for path in sorted(args.archive.rglob('*'))
              if path.is_file() and path.name != 'artifact_hashes.json'}
    (args.archive / 'artifact_hashes.json').write_text(json.dumps(hashes, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
