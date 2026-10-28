"""Reference-free native score ordering; imported by the trial before sealing."""
import csv,json,math
from pathlib import Path
def rank_candidates(candidates):
    eligible=[c for c in candidates if c['converged'] and c['pose_finite'] and math.isfinite(c['score'])]
    return sorted(eligible,key=lambda c:(-c['score'],c['candidate_id']))
def select(input_path,native_path,output_path):
    frozen=json.loads(Path(input_path).read_text())
    with Path(native_path).open() as stream:rows=list(csv.DictReader(stream))
    assert len(rows)==48 and [int(r['call_number']) for r in rows]==list(range(1,49))
    selected=[]
    for f in frozen['frames']:
        raw=f['raw_ordinal'];c0=f['candidate0']
        candidates=[{'candidate_id':0,'score':c0['native_probability'],'converged':bool(c0['converged']),
                     'pose_finite':all(math.isfinite(x) for row in c0['T_final'] for x in row)}]
        for r in rows:
            if int(r['raw_ordinal'])!=raw:continue
            candidates.append({'candidate_id':int(r['candidate_id']),'score':float(r['native_probability']),
                               'converged':r['converged']=='1','pose_finite':all(math.isfinite(float(r['final_'+str(i)+str(j)])) for i in range(4) for j in range(4))})
        assert [c['candidate_id'] for c in candidates]==list(range(9))
        ranking=rank_candidates(candidates)
        selected.append({'raw_ordinal':raw,'stamp_ns':f['stamp_ns'],
                         'selected_id':ranking[0]['candidate_id'] if ranking else None,'ranking':ranking})
    Path(output_path).write_text(json.dumps(selected,indent=2,allow_nan=False)+'\n')
    return selected
