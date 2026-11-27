"""Use the previous causal short window and unchanged Q1 grid/peak/fit logic."""
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from common import HERE,PREV,R2,HIST,FIELDS,digest,read,output
sys.path.insert(0,str(HERE.parent/'p64_r2_q1_candidate_search'))
import peaks

IDX=(5,15,25)

def assets(run):
    archive=R2/'run/protocol_b_frozen'
    seal=json.loads((R2/'docs/p64_r2_q2_bounded_search_adaptation/protocol_b_online_seal.json').read_text())
    for x in seal.values():
        assert digest(Path(x['path']))==x['sha256']
    r=json.loads((archive/'BLIND_INIT_RECEIPT.json').read_text())
    traj=read(Path(seal['output']['path']))
    map_path=Path(next(x['map_pcd'] for x in read(PREV/'sample_manifest.csv') if x['frame']=='O164'))
    checks=[]
    with (run/'mid_plan.tsv').open('x') as stream:
        for i in IDX:
            row=traj[i]
            scan=archive/r['scans'][i]['file']
            assert int(row['stamp_ns'])==r['scans'][i]['stamp_ns']
            assert digest(scan)==r['scans'][i]['sha256']
            print(f'B{i:02}',map_path,scan,*[row[j+s.lower()[1:]] for j in ('pred','ndt') for s in FIELDS],sep='\t',file=stream)
            checks.append(dict(frame=f'B{i:02}',stamp_ns=row['stamp_ns'],path=str(scan),scan_sha=digest(scan),
                               map_path=str(map_path),map_sha=digest(map_path),source='Protocol B causal independent map',
                               trigger='frozen short-window indices'))
    output(run/'mid_assets.csv',checks)

def seeds(run):
    grouped=peaks.read_grid(run/'mid_grid/grid.csv')
    assert len(grouped)==3
    peaks.METRICS=('native',)
    rows=[]
    with (run/'mid_seeds.tsv').open('x') as stream:
        for frame,grid in grouped.items():
            modes=peaks.extract_metric(grid,'native')
            rows.extend(modes)
            for row,rank,metric in peaks.refinement_seeds(grid,{'native':modes}):
                print(frame,row['index'],rank,metric,*[row[s] for s in FIELDS],sep='\t',file=stream)
    output(run/'mid_modes.csv',rows)

def qualify(run):
    fits=read(run/'mid_fit/fit.csv')
    rows=[]
    for i in IDX:
        frame=f'B{i:02}'
        ordered=sorted([x for x in fits if x['sample_id']==frame],key=lambda x:-float(x['native']))
        modes=[]
        for x in ordered:
            t=np.array([float(x[k]) for k in FIELDS]).reshape(4,4)
            valid=int(x['iterations'])>0 and int(x['reported_converged'])==1 and int(x['associated'])>=.5*int(x['total'])
            separate=valid and all(np.linalg.norm(z[:3,3]-t[:3,3])>=.25 or
                Rotation.from_matrix(z[:3,:3].T@t[:3,:3]).magnitude()>=np.deg2rad(2) for z in modes)
            if separate: modes.append(t)
            row=dict(**x,valid=valid,separate=separate,ratio_to_best=float(x['native'])/float(ordered[0]['native']))
            rows.append(row)
    output(run/'mid_qualified_fits.csv',rows)
    paths=['mid_assets.csv','mid_plan.tsv','mid_grid/grid.csv','mid_seeds.tsv','mid_fit/fit.csv','mid_qualified_fits.csv']
    (run/'mid_seal.json').write_text(json.dumps({p:digest(run/p) for p in paths},indent=2)+'\n')

if __name__=='__main__':
    {'plan':assets,'seeds':seeds,'qualify':qualify}[sys.argv[1]](Path(sys.argv[2]))
