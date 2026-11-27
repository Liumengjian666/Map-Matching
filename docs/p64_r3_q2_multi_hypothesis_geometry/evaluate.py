"""Open reference labels only after validating the blind scoring receipt."""
from collections import defaultdict
from pathlib import Path
import json
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from common import HERE,HIST,FRAMES,digest,output,read,pcd,packet

METHODS=('M0','M2','M3','M4','M5','M6','U1','U2','U3','U4','U3_weighted','U4_weighted')

def rank(out):
    seal=json.loads((out/'blind_seal.json').read_text())
    for name,key in seal.items():
        assert digest(name)==key,name
    hist=read(HIST/'historical_labels_DO_NOT_USE_FOR_SELECTION.csv')
    states={}
    for row in hist:
        t,r=row['translation_m'],row['rotation_deg']
        states[row['candidate_id']]=('legacy_nominal' if 'native' in row['candidate_id'] else 'synthetic') if not t else (
            'correct' if float(t)<=.15 and float(r)<=2 else 'wrong')
    return label(out,states)

def label(out,states):
    groups=defaultdict(list)
    for row in read(out/'blind_ranking.csv'):
        keys=['frame','subset','method','budget','seed']
        groups[tuple(row[k] for k in keys)].append(row)
    full=[]; summary=[]; wrong=[]
    for key,rows in groups.items():
        rows.sort(key=lambda x:int(x['rank']))
        labels=[states[r['candidate_id']] for r in rows]
        winners=[i for i,x in enumerate(labels) if x in ('correct','legacy_nominal')]
        invalid=[i for i,x in enumerate(labels) if x not in ('correct','legacy_nominal','unknown')]
        correct=min(winners)+1 if winners else None
        gap=float(rows[correct-1]['reward'])-float(rows[min(invalid)]['reward']) if correct is not None and invalid else None
        for r,x in zip(rows,labels):
            full.append(dict(**r,reference_state=x))
        s={k:rows[0][k] for k in keys+['candidate_count','selected','support','winner_margin',
            'native_association','selected_association','wins_axis','positive_regions_for_top','dominant_region_fraction','winner_runner_flip_leave_region']}
        s.update(winner=rows[0]['candidate_id'],winner_state=labels[0],target_rank=correct,
                 top1=correct==1 if correct is not None else None,top2=correct<=2 if correct is not None else None,
                 correct_minus_strongest_other=gap)
        summary.append(s)
        if all(x=='wrong' for x in labels):
            wrong.append(dict(**s,imu_absolute_consistency='NA_legacy_initialization',
                              unsupported_selection_fraction=1-int(s['support'])/int(s['selected'])))
    output(out/'full_candidate_ranking.csv',full)
    output(out/'multi_hypothesis_selection_results.csv',summary)
    output(out/'continuous_weight_comparison.csv',[r for r in summary if r['method'].startswith('U') or r['method']=='M3'])
    if wrong: output(out/'all_wrong_candidate_analysis.csv',wrong)
    return summary

def figures(run,out,rows):
    figs=out/'figures'
    figs.mkdir()
    im,axs=plt.subplots(3,4,figsize=(14,10),layout='constrained')
    with np.load(run/'shared_masks.npz') as masks:
        for axes,frame in zip(axs,FRAMES[:3]):
            saved=np.load(run/f'{frame}_deleted_geometry.npz')
            pts=saved['source']
            order=[np.zeros(len(pts)),saved['selected'],saved['delta'],np.isin(np.arange(len(pts)),masks[f'{frame}_full_M6_50_-1'])]
            for ax,d,title in zip(axes,order,['Full cloud','M3 selected green','Native reward A − B','Joint M6 selected green']):
                if title=='Native reward A − B':
                    ax.scatter(pts[:,0],pts[:,2],c=d,cmap='coolwarm',s=2,vmin=-1.5,vmax=1.5)
                else:
                    ax.scatter(pts[:,0],pts[:,2],c=np.where(d,'#28a467','#9a9a9a'),s=2)
                ax.set_title(frame+' '+title)
                ax.set_aspect('equal')
    im.savefig(figs/'selected_removed_and_scores.png',dpi=160)
    im,axs=plt.subplots(3,3,figsize=(13,9),layout='constrained')
    for axes,frame in zip(axs,FRAMES[:3]):
        data=packet(frame); pts=data['source']
        classes=np.load(run/f'{frame}_deleted_geometry.npz')['class_proxy']
        names,ic=np.unique(classes,return_inverse=True)
        axes[0].scatter(pts[:,0],pts[:,2],c=ic,cmap='tab10',s=3)
        axes[0].set_title(frame+' local geometry: '+', '.join(n.split('_')[0] for n in names))
        pcd(run/f'{frame}_local_plane_proxy.pcd',pts,np.minimum(ic,3))
        geom=np.load(HIST/'geometry'/f'geometry_{frame}.npz')
        hard=geom['pair_classes'][0,:,3]
        soft=np.load(run/f'{frame}_soft.npz')['weights'][0,1]
        axes[1].scatter(pts[:,0],pts[:,2],c=hard,cmap='viridis',s=3)
        axes[1].set_title(frame+' hard-band class')
        axes[2].scatter(pts[:,0],pts[:,2],c=soft,cmap='viridis',s=3)
        axes[2].set_title(frame+' continuous U4')
        for ax in axes: ax.set_aspect('equal')
    im.savefig(figs/'local_structure_hard_and_continuous.png',dpi=160)
    im,axs=plt.subplots(1,3,figsize=(14,4),layout='constrained')
    for ax,frame in zip(axs,FRAMES[:3]):
        for method in ('M2','M3','M4','M5','M6','U4_weighted'):
            group=[r for r in rows if r['frame']==frame and r['subset']=='full' and r['method']==method]
            ax.plot([int(r['budget']) for r in group],[r['correct_minus_strongest_other'] for r in group],'.-',label=method)
        ax.axhline(0,color='k',lw=.7); ax.set_title(frame); ax.set_xlabel('Source budget (%)')
    axs[0].set_ylabel('Correct minus strongest other reward')
    axs[-1].legend()
    im.savefig(figs/'full_candidate_margins.png',dpi=170)
    rows_info=read(out/'information_retention.csv')
    im,ax=plt.subplots(figsize=(10,4),layout='constrained')
    for frame,id in [('H307','H307_P2648'),('H515','H515_N2200'),('H309','H309_park_2648')]:
        g0=[r for r in rows_info if r['frame']==frame and r['candidate_id']==id and r['subset']=='full' and
            r['budget']=='50' and r['method'] in ('M2','M3','M4','M5','M6','U4_weighted')]
        ax.plot([r['method'] for r in g0],[float(r['weak_ratio']) for r in g0],'.-',label=frame)
    ax.set_ylabel('Weakest information eigenvalue retention')
    ax.legend()
    im.savefig(figs/'weak_information.png',dpi=170)
    plt.close('all')

if __name__=='__main__':
    run,out=map(Path,sys.argv[1:3])
    rows=rank(out)
    figures(run,out,rows)
