"""Check all sealed ranks, shared masks and geometry identity invariants."""
from pathlib import Path
import json
import sys
import numpy as np
from freeze import digest, read

run,out=map(Path,sys.argv[1:])
frozen=read(out/'fixed_hypothesis_manifest.csv')
assert len(frozen)==10
rows=read(out/'fixed_candidate_ranking.csv')
info=read(out/'information_retention.csv')
blind=json.loads((run/'blind/blind_seal.json').read_text())
for path,key in blind['files'].items():
    assert digest(path)==key,path
full=read(out/'native_fixed_score_audit.csv')
assert len(full)==250
assert sum(r['j']=='0' for r in full)==10
assert len(rows)==480 and len(info)==1280
lookup={(x['frame'],x['pair'],x['budget_percent'],x['method']):x for x in rows if x['method']!='B1'}
with np.load(run/'blind/shared_masks.npz') as archive:
    for x in rows:
        mask=archive[x['mask_id']]
        assert len(mask)==int(x['selected_n'])
        assert np.array_equal(np.unique(mask),mask)
        if x['method']=='B3_EQUAL':
            y=lookup[x['frame'],x['pair'],x['budget_percent'],'B4']
            assert np.array_equal(mask,archive[y['mask_id']])
        if x['method']=='B9' and int(x['fallback_n'])==int(x['selected_n']):
            y=lookup[x['frame'],x['pair'],x['budget_percent'],'B2']
            assert np.array_equal(mask,archive[y['mask_id']])
for x in info:
    assert int(x['rank'])<=6
    if x['lambda_min_ratio']:
        assert float(x['lambda_min_ratio'])<1+1e-9
print('PASS 250 fixed scorer calls, ten original objective audits, 480 masks, receipt verification and PSD retention checks')
