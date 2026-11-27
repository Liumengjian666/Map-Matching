"""Evaluate sealed real Mid-360 terminals and visualize the weak secondary mode."""
from pathlib import Path
import json
import runpy
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from common import HERE,PREV,R2,FIELDS,digest,packet,pcd,output,read

def main(run,out):
    seal=json.loads((run/'mid_seal.json').read_text())
    for p,k in seal.items():
        assert digest(run/p)==k
    rows=read(run/'mid_qualified_fits.csv')
    inventory=read(run/'mid_assets.csv')
    stamps={x['frame']:int(x['stamp_ns']) for x in inventory}
    q1=runpy.run_path(str(HERE.parent/'p64_r2_q1_candidate_search/prepare_inputs.py'))
    track=q1['read_omni_reference'](max(stamps.values()))
    result=[]
    for x in rows:
        pose=np.array([float(x[k]) for k in FIELDS]).reshape(4,4)
        ref=q1['interpolate'](*track,stamps[x['sample_id']],True)
        e_t=np.linalg.norm(ref[:3,3]-pose[:3,3]) if ref is not None else None
        e_r=np.rad2deg(Rotation.from_matrix(ref[:3,:3].T@pose[:3,:3]).magnitude()) if ref is not None else None
        result.append(dict(**x,stamp_ns=stamps[x['sample_id']],ref_translation=e_t,ref_rotation=e_r,
                           correct=e_t<=.15 and e_r<=2 if ref is not None else None,
                           geometric_rescoring='NOT_RUN_NO_STRONG_COMPETITIVE_PAIR'))
    output(out/'mid360_validation_results.csv',result)
    for x in inventory:
        fit=[r for r in result if r['sample_id']==x['frame'] and r['separate']=='True']
        x.update(separate_valid_terminals=len(fit),
                 strong=len(fit)>1 and float(fit[1]['ratio_to_best'])>=.90,
                 protocol='independent Diff map, Protocol B causal; Q2 window already exposed for initializer verification')
    inventory.extend([
        dict(frame='O160–O164',source='R3-Q1 actual repeated Mid-360 observations',protocol='legacy reference-aided absolute initialization; O164 control artificial'),
        dict(frame='O164/O910/O1550/O2450',source='Q2 archive search results',protocol='legacy Protocol A absolute initialization; no independent candidate claim'),
        dict(frame='Loop2',source='R1 engineering map and test data',protocol='same-origin map; excluded from independent primary experiment')])
    output(out/'mid360_candidate_asset_inventory.csv',inventory)
    two=[x for x in result if x['sample_id']=='B25' and x['separate']=='True']
    if len(two)!=2:
        return
    points=read(run/'mid_fit/point_scores.csv')
    xyz=[]
    for event in two:
        pts=[r for r in points if r['sample_id']=='B25' and r['pose_id']=='fit_'+event['index']]
        xyz.append(np.array([[float(r[k]) for k in 'xyz'] for r in pts]))
    m=packet('O164')['map']
    a=xyz[0].mean(axis=0)
    crop=m[np.linalg.norm(m-a,axis=1)<8]
    cols=np.concatenate([np.full(len(crop),0),np.full(len(xyz[0]),1),np.full(len(xyz[1]),2)])
    pcd(run/'B25_nominal_and_weak_terminal.pcd',np.vstack((crop,*xyz)),cols)
    fig,ax=plt.subplots(figsize=(8,7),layout='constrained')
    ax.scatter(crop[:,0],crop[:,1],s=1,c='silver',label='Independent map')
    for d,c,legend in zip(xyz,['#16885c','#cb473c'],['Native best','Separated weak terminal']):
        ax.scatter(d[:,0],d[:,1],s=2,c=c,label=legend)
    ax.set_aspect('equal'); ax.set_xlabel('World x (m)'); ax.set_ylabel('World y (m)'); ax.legend()
    fig.savefig(out/'figures/B25_real_mid360_two_terminals.png',dpi=170)
if __name__=='__main__':
    main(Path(sys.argv[1]),Path(sys.argv[2]))
