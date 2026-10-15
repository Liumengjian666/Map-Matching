#!/usr/bin/env python3
"""Verify small archived evidence without NN, optimization or replay."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main(write_hashes):
    frozen = json.loads((HERE/'input_freeze.json').read_text())
    summary = json.loads((HERE/'results.json').read_text())
    comparison = rows(HERE/'nn_comparison.csv')
    if len(comparison) != 12 or [s['raw_frame_index'] for s in frozen['samples']] != [665, 993, 1488]:
        raise ValueError('FRAME_BUDGET')
    Y = np.array(frozen['Y']); invY = np.array(frozen['inverse_Y'])
    np.testing.assert_array_equal(invY, np.linalg.inv(Y))
    np.testing.assert_allclose(Y@invY, np.eye(4), rtol=0, atol=1e-14)
    for i, sample in enumerate(frozen['samples']):
        GT = np.array(sample['T_G_GT'])
        expected = [GT, np.array(sample['H_B_conditional']), Y@GT, invY@GT]
        part = comparison[4*i:4*i+4]
        if [r['group'] for r in part] != ['C0_GT', 'C1_GT_E', 'A_Y_GT', 'B_INVERSE_Y_GT']:
            raise ValueError('GROUP_ORDER')
        for item, T in zip(part, expected):
            if item['scan_ns'] != sample['scan_ns']:
                raise ValueError('EXACT_INTEGER_STAMP')
            actual = np.array([float(item[f'T{r}{c}']) for r in range(4) for c in range(4)]).reshape(4, 4)
            np.testing.assert_allclose(actual, T, rtol=0, atol=1e-14)
        for tag, k in [('A', 2), ('B', 3)]:
            for metric, key in [('nn_median_m', 'nn_median_m'), ('nn_p90_m', 'nn_p90_m'), ('coverage_8m', 'coverage_8m')]:
                if float(part[k][metric]) != summary[f'{tag}_{key}'][i]:
                    raise ValueError('RESULTS_NUMBER_CHANGED')
            if any(float(part[k][key]) != 0 for key in ['coverage_0_5m', 'coverage_2m']):
                raise ValueError('REPORTED_ZERO_OVERLAP_CHANGED')
    runtime = rows(HERE/'runtime_summary.csv')
    if (sum(int(r['NN_queries']) for r in runtime) != 6 or
            sum(int(r['NDT_aligns'])+int(r['ICP_calls']) for r in runtime) != 0):
        raise ValueError('CALL_BUDGET')
    parity = rows(HERE/'source_parity.csv')
    if len(parity) != 3 or not all(r['preprocessed_bytes_equal_P33'] == 'True' for r in parity):
        raise ValueError('SOURCE_CONTRACT')
    if sha(HERE/'PROTOCOL.md') != frozen['protocol_SHA256']:
        raise ValueError('PROTOCOL_SHA')
    dependencies = [WS/'docs/p33_corridor01_complete_frame_relation'/name
                    for name in ['input_freeze.json', 'direct_projection.csv', 'prepare.py']]
    dependencies += [WS/'docs/corridor01_gt_frame_initialization/geometry_probe.cpp',
        WS/'src/dog_prior_map_localization/scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp',
        WS/'src/dog_prior_map_localization/scripts/p32_gt_frame/prepare.py',
        WS/'src/dog_prior_map_localization/scripts/p10_corridor01_prospective/pack_causal_slice.py']
    if sha(dependencies[0]) != frozen['P33_freeze_SHA256'] or sha(dependencies[1]) != frozen['P33_controls_SHA256']:
        raise ValueError('HISTORICAL_CONTROL_SHA')
    probe = Path(frozen['probe'])
    if sha(probe) != frozen['probe_SHA256']:
        raise ValueError('EXECUTED_BINARY_SHA')
    files = [p for p in HERE.iterdir() if p.is_file() and p.name != 'artifact_hashes.json']+dependencies+[probe]
    manifest = dict(files={str(p.relative_to(WS)): dict(SHA256=sha(p), bytes=p.stat().st_size) for p in files},
                    input_hashes_reference='input_freeze.json (fresh/inherited flags are authoritative)')
    path = HERE/'artifact_hashes.json'
    if write_hashes:
        with path.open('x') as stream:
            json.dump(manifest, stream, indent=2); stream.write('\n')
    else:
        stored = json.loads(path.read_text())
        if stored != manifest:
            raise ValueError('ARTIFACT_SHA')
    print('PASS: 3 fixed frames; 4 matrices each; same frozen protocol/controls; 6 new NN; 0 ICP/NDT/replay.')
    print('PASS: archived A/B numbers, ordered source parity, binary and source/artifact SHA.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write-hashes', action='store_true', help='Create manifest once; no overwrite')
    main(**vars(parser.parse_args()))
