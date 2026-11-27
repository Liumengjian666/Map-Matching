"""Apply archived evaluation labels to completed blind rankings."""
from collections import defaultdict
from pathlib import Path
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from attribution import HIST,FRAMES,pcd,output,read,digest

def labels():
    states={}
    for r in read(HIST/'historical_labels_DO_NOT_USE_FOR_SELECTION.csv'):
        name=r['candidate_id']; t=r['translation_m']; a=r['rotation_deg']
        states[name]='correct' if t and float(t)<=.15 and float(a)<=2 else 'wrong' if t else (
            'legacy_nominal' if name=='O164_native' else 'synthetic')
    return states

def review(rows,states):
    sets=defaultdict(list)
    results=[]; detailed=[]
    for r in rows:
        sets[tuple(r[k] for k in ('frame','point_set','aggregation'))].append(r)
    for key,cols in sets.items():
        ordered=sorted(cols,key=lambda c:int(c['rank']))
        accepted=[r for r in ordered if states[r['candidate_id']] in ('correct','legacy_nominal')]
        other=[r for r in ordered if states[r['candidate_id']] in ('wrong','synthetic')]
        top=ordered[0]
        t=accepted[0]
        a_main=next(r for r in other if not r['candidate_id'].endswith(('N1739','C3_terminal')))
        rated=[dict(**r,label=states[r['candidate_id']]) for r in ordered]
        detailed.extend(rated)
        results.append(dict(frame=key[0],point_set=key[1],aggregation=key[2],n=top['n'],N=top['N'],
            winner=top['candidate_id'],correct_rank=t['rank'],top1=t['rank']=='1',top2=int(t['rank'])<=2,
            margin_strongest_other=float(t['score'])-float(other[0]['score']),
            margin_main_pair=float(t['score'])-float(a_main['score']),
            co_support_points=t['co_support_points']))
    return detailed,results

def figures(run,out,results):
    folder=out/'figures'; folder.mkdir()
    fig,axs=plt.subplots(3,6,figsize=(17,8),layout='constrained')
    cloud=out/'point_clouds'; cloud.mkdir()
    for aa,frame in zip(axs,FRAMES[:3]):
        with np.load(run/f'{frame}_terms.npz') as d:
            source=d['source']
            u=d['support_equal']; v=d['diff_rank']
            a=np.isin(np.arange(len(source)),u); b=np.isin(np.arange(len(source)),v)
            indices=a.astype(int)+2*b
            for ax,key,title in zip(aa[:4],('delta','max_diff','quantity','quality'),
                                   ('DIRECT7 delta','Max voxel delta','Quantity term','Quality term')):
                x=d[key]; limit=max(.05,np.quantile(np.abs(x),.95))
                ax.scatter(source[:,0],source[:,2],c=x,cmap='coolwarm',s=2,vmin=-limit,vmax=limit)
                ax.set_title(f'{frame} {title}'); ax.set_aspect('equal')
                pcd(cloud/f'{frame}_{key}.pcd',source,np.where(x>.05,1,np.where(x<-.05,2,0)))
            for ax,condition,title in zip(aa[4:],(a,b),('Support-voxel','Support-difference')):
                ax.scatter(source[:,0],source[:,2],c=np.where(condition,'#2a9f69','#9a9a9a'),s=2)
                ax.set_title(title); ax.set_aspect('equal')
            pcd(cloud/f'{frame}_support_comparison.pcd',source,indices)
    fig.savefig(folder/'geometry_reward_attribution.png',dpi=165)
    fig,axs=plt.subplots(1,3,figsize=(12,4),layout='constrained')
    for ax,frame in zip(axs,FRAMES):
        z=np.empty((3,4))
        for r in results:
            if r['frame']==frame:z[int(r['aggregation'][1]),int(r['point_set'][1])]=int(r['correct_rank'])
        ax.imshow(z,vmin=1,vmax=3,cmap='RdYlGn_r')
        for i in range(3):
            for j in range(4):ax.text(j,i,str(int(z[i,j])),ha='center',va='center')
        ax.set_title(frame); ax.set_xticks(range(4),['Full','Voxel','M3','M6']); ax.set_yticks(range(3),['Sum','Max','Mean'])
    fig.savefig(folder/'complete_ranking.png',dpi=175)
    plt.close('all')

def evaluate(run,out):
    for p,v in json.loads((out/'blind_seal.json').read_text()).items():assert digest(p)==v
    states=labels()
    scored,res=review(read(out/'blind_ranking.csv'),states)
    output(out/'complete_candidate_reward_ranking.csv',scored)
    output(out/'reward_selection_factorial.csv',res)
    _,support=review(read(out/'blind_support.csv'),states)
    output(out/'support_gate_factorial.csv',support)
    interactions=[]
    idx={(r['frame'],r['point_set'],r['aggregation']):r for r in res}
    for frame in FRAMES:
        for base in ('P0','P1'):
            for picked in ('P2','P3'):
                rows=[idx[frame,x,y] for x,y in [(base,'R0'),(picked,'R0'),(base,'R1'),(picked,'R1')]]
                row=dict(frame=frame,base=base,geometry=picked)
                for column in ('margin_main_pair','margin_strongest_other'):
                    a,b,c,d=[r[column] for r in rows]
                    row.update({f'{column}_A':a,f'{column}_B':b,f'{column}_C':c,f'{column}_D':d,
                                f'{column}_interaction':(b-a)-(d-c)})
                interactions.append(row)
    output(out/'reward_interactions.csv',interactions)
    figures(run,out,res)

if __name__=='__main__':evaluate(Path(sys.argv[1]),Path(sys.argv[2]))
