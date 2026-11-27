"""Offline contract checks over frozen Q3 artifacts; no registration or filter calls."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

Q3 = Path(__file__).resolve().parent.parent / 'p64_r3_q3_geometry_reward_attribution'

def rows(name):
    with (Q3 / name).open() as stream:
        return list(csv.DictReader(stream))

def pose(row):
    return np.array([float(row[f'T{i}{j}']) for i in range(4) for j in range(4)]).reshape(4, 4)

def main():
    ranks = rows('real_lidar_candidate_results.csv')
    manifest = rows('real_fixed_manifest.csv')
    events = rows('causal_event_manifest.csv')
    by_id = {r['candidate_id']: r for r in manifest}
    checks = []
    committed = {}
    for frame, expected_support in [('B25', 34), ('W2_32', 11)]:
        event, = [r for r in events if r['frame'] == frame]
        origin, = [r for r in ranks if r['frame'] == frame and r['method'] == 'B0' and r['rank'] == '1']
        proposals = [r for r in ranks if r['frame'] == frame and r['method'] == 'B2']
        ranked = sorted(proposals, key=lambda r: int(r['rank']))
        assert len(ranked) == int(event['n_valid_modes']) == 2
        assert {r['co_support'] for r in ranked} == {str(expected_support)}
        assert {r['kept'] for r in ranked} == {str(expected_support)}
        identity = (event['stamp_ns'], event['source_sha256'])
        for row in ranked:
            record = by_id[row['candidate_id']]
            assert (record['stamp_ns'], record['source_sha256']) == identity
            mat = pose(record)
            np.testing.assert_allclose(mat @ np.linalg.inv(mat), np.eye(4), atol=1e-12)
            np.testing.assert_allclose(mat[3], [0, 0, 0, 1])
            assert abs(np.linalg.det(mat[:3, :3]) - 1) < 2e-6
        # Only the realized native choice carries a submit token in shadow mode.
        assert set((origin['candidate_id'], ranked[0]['candidate_id'])) <= set(by_id)
        for purpose, candidate in [('active', origin), ('shadow', ranked[0]), ('retry', origin)]:
            if purpose != 'shadow' and identity not in committed:
                committed[identity] = candidate['candidate_id']
        assert committed[identity] == origin['candidate_id']
        proposal_conflicts = origin['candidate_id'] != ranked[0]['candidate_id']
        assert proposal_conflicts == (frame == 'B25')
        checks.append(dict(frame=frame, support=expected_support, proposed_reorder=proposal_conflicts,
                           chosen=committed[identity], submit_tokens=1))
    assert len(committed) == len(checks) == 2
    historical = rows('reward_selection_factorial.csv')
    h307 = {(r['point_set'], r['aggregation']):r for r in historical if r['frame'] == 'H307'}
    assert [h307[k]['correct_rank'] for k in (('P0','R1'),('P2','R1'))] == ['1','2']
    names = ['real_lidar_candidate_results.csv','real_fixed_manifest.csv',
             'causal_event_manifest.csv','reward_selection_factorial.csv']
    sha = {name: hashlib.sha256((Q3/name).read_bytes()).hexdigest() for name in names}
    print(json.dumps(dict(status='PASS',mode='archived fixtures and shadow submit tokens only',
                          score_names_kept_distinct=['DIRECT7_SUM','DIRECT7_MAX','DIRECT7_MEAN'],
                          checks=checks,input_sha256=sha),indent=2))

if __name__ == '__main__':
    main()
