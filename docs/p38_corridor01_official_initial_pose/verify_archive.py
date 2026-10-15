#!/usr/bin/env python3
"""Verify archived matrices/statistics/provenance without geometry or GT queries."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from prepare_initial import sha, ndt_eligible

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]


def main(write_hashes):
    frozen = json.loads((HERE/'input_freeze.json').read_text())
    summary = json.loads((HERE/'results.json').read_text())
    gate = json.loads((HERE/'geometry_gate.json').read_text())
    with (HERE/'geometry_comparison.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 2 or [r['candidate'] for r in rows] != ['A_DIRECT_Y', 'B_INVERSE_Y']:
        raise ValueError('TWO_DIRECT_CANDIDATES_ONLY')
    Y = np.array(frozen['Y']); invY = np.array(frozen['inverse_Y'])
    np.testing.assert_array_equal(invY, np.linalg.inv(Y))
    np.testing.assert_allclose(Y@invY, np.eye(4), atol=1e-14, rtol=0)
    metrics = [('nn_median_m', 'nn_median_m'), ('nn_p90_m', 'nn_p90_m'),
               ('coverage_0_5m', 'overlap_0_5m'), ('coverage_2m', 'overlap_2m'), ('coverage_8m', 'overlap_8m')]
    for row, T, name in zip(rows, [Y, invY], ['A', 'B']):
        if row['scan_ns'] != '1517157286055073023' or row['raw_frame_index'] != '665':
            raise ValueError('INTEGER_SCAN_IDENTITY')
        actual = np.array([float(row[f'T{r}{c}']) for r in range(4) for c in range(4)]).reshape(4, 4)
        np.testing.assert_array_equal(actual, T)
        for old, new in metrics:
            if float(row[old]) != summary[name][new]:
                raise ValueError('ARCHIVE_STATISTICS_DIFFER')
        if ndt_eligible(row) or summary[name]['preregistered_strict_geometry_gate_pass']:
            raise ValueError('REPORTED_NDT_STOP_GATE_CHANGED')
    if (gate['numeric_eligible_candidates'] or gate['preregistered_strict_geometry_gate_pass'] or
            summary['NDT_align_calls'] != 0 or summary['ICP_calls'] != 0 or summary['localization_replays'] != 0):
        raise ValueError('STOP_BUDGET')
    if (HERE/'ndt_single_frame_result.csv').exists():
        raise ValueError('UNEXECUTED_ARTIFACT')
    if (sha(HERE/'PROTOCOL.md') != frozen['protocol_SHA256'] or
            frozen['candidates_derived_from_GT'] or
            frozen['GT_text_file_poses_parsed_before_candidate_freeze'] or
            not frozen['P33_archive_loaded_before_freeze_including_unused_GT_matrices']):
        raise ValueError('FREEZE_OR_GT_PROVENANCE')
    source = json.loads((HERE/'source_parity.json').read_text())
    if source['preprocessed_XYZ_SHA256'] != frozen['preprocessed_XYZ_SHA256']:
        raise ValueError('SOURCE_SHA')
    stdout = (HERE/'probe_stdout.txt').read_text()
    if stdout.count('NN_queries=2 full_NDT_aligns=0 ') != 1:
        raise ValueError('EXECUTED_CALL_COUNT')
    lengths = np.diff(np.array(gate['plot_axis_limits_m']), axis=1).ravel()
    np.testing.assert_allclose(lengths, gate['common_axis_span_m'], rtol=0, atol=1e-12)
    dependencies = [WS/'docs/p36_corridor01_world_planar_test/prepare_hypotheses.py',
        WS/'docs/p33_corridor01_complete_frame_relation/prepare.py',
        WS/'docs/p33_corridor01_complete_frame_relation/input_freeze.json',
        WS/'docs/corridor01_gt_frame_initialization/geometry_probe.cpp',
        WS/'src/dog_prior_map_localization/scripts/p23_budgeted_coupled/frozen_ndt_rules.hpp',
        WS/'src/dog_prior_map_localization/scripts/p10_corridor01_prospective/pack_causal_slice.py',
        WS/'src/dog_prior_map_localization/scripts/p32_gt_frame/prepare.py']
    probe = Path(frozen['probe'])
    if sha(probe) != frozen['probe_SHA256']:
        raise ValueError('PROBE_SHA')
    files = [p for p in HERE.iterdir() if p.is_file() and p.name != 'artifact_hashes.json']+dependencies+[probe]
    manifest = dict(files={str(p.relative_to(WS)): dict(SHA256=sha(p), bytes=p.stat().st_size) for p in files},
                    inputs='input_freeze.json lists real input SHA and fresh/inherited flags')
    path = HERE/'artifact_hashes.json'
    if write_hashes:
        with path.open('x') as stream:
            json.dump(manifest, stream, indent=2); stream.write('\n')
    elif json.loads(path.read_text()) != manifest:
        raise ValueError('ARTIFACT_SHA')
    print('PASS: raw665; direct Y/full inverse only; exact archived metrics and matrices; 2 NN, 0 NDT/ICP/replay.')
    print('PASS: strict gate stop, honest GT provenance, common plot scale, source and artifact SHA.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write-hashes', action='store_true')
    main(**vars(parser.parse_args()))
