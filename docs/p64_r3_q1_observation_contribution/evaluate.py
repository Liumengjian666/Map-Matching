"""Verify the blind receipt, then expose historical labels for offline analysis."""
from collections import defaultdict
from pathlib import Path
import argparse
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from freeze import PAIRS, digest, read, write
from geometry import write_pcd

METHODS = ['B0','B1','B2','B3_NATURAL','B4','B5','B6','B7_E1','B7_E2','B7_E3','GEO0','GEO1','B8','B9']

def load_after_seal(run):
    blind=run/'blind'
    receipt=json.loads((blind/'blind_seal.json').read_text())
    for path, expected in receipt['files'].items():
        if digest(path)!=expected:
            raise RuntimeError(f'blind receipt changed: {path}')
    return receipt, blind

def outcome(error):
    if not error['translation_m']:
        return 'SYNTHETIC' if 'synthetic' in error['candidate_id'] else 'LEGACY_NORMAL'
    return 'CORRECT' if float(error['translation_m'])<=.15 and float(error['rotation_deg'])<=2 else 'WRONG'

def tri_state(frame, label):
    return label == ('LEGACY_NORMAL' if frame=='O164' else 'CORRECT')

def main(run,out):
    receipt,blind=load_after_seal(run)
    label={r['candidate_id']:r for r in read(run/'historical_labels_DO_NOT_USE_FOR_SELECTION.csv')}
    rows=read(blind/'blind_pair_ranking.csv')
    score=read(blind/'blind_candidate_scores.csv')
    infos=read(blind/'blind_information.csv')
    index=defaultdict(list)
    baselines={}
    for r in score:
        index[r['mask_id']].append(r)
    for r in rows:
        if r['method']=='B0':
            baselines[r['frame'],r['pair']]=r
    ranked=[]
    for r in rows:
        frame=r['frame']
        la,lb=[outcome(label[r[key]]) for key in ('candidate_a','candidate_b')]
        has_target=tri_state(frame,la) != tri_state(frame,lb)
        preferred=int(r['pair_preferred']) if r['pair_preferred'] else None
        full=int(baselines[frame,r['pair']]['pair_preferred'])
        desired=1 if tri_state(frame,la) else -1
        same=[idx for idx in index[r['mask_id']]]
        ordered=sorted(same,key=lambda x:-float(x['nominal_score']))
        all_ranks={x['candidate_id']:i+1 for i,x in enumerate(ordered)}
        is_top=preferred==desired if has_target and preferred is not None else ''
        target_id=r['candidate_a' if desired==1 else 'candidate_b']
        ranked.append(dict(**r,label_a=la,label_b=lb,has_pair_target=has_target,
                           pair_target_rank=(1 if is_top else (2 if preferred else '')) if has_target else '',
                           pair_top1_correct=is_top,
                           pair_transition=(f'{int(full==desired)}->{int(is_top)}' if is_top!='' else ''),
                           target_margin=float(r['net'])*desired if has_target and preferred is not None else '',
                           target_all_frame_rank=all_ranks[target_id] if has_target and preferred is not None else '',
                           all_frame_top=ordered[0]['candidate_id'] if preferred is not None else '',
                           all_frame_top_state=outcome(label[ordered[0]['candidate_id']]) if preferred is not None else ''))
    write(out/'fixed_candidate_ranking.csv',ranked)
    write(out/'selection_method_comparison.csv',[
        {k:r[k] for k in ('frame','pair','method','budget_percent','seed','selected_n','fallback_n','occupied_voxels_030',
            'geo2_discriminative','geo3_discriminative','pair_top1_correct','pair_transition','target_margin',
            'target_all_frame_rank','scale2_paired_a_wins','scale2_cross_a_wins','scale1_sign_stable_fraction')}
        for r in ranked])
    write(out/'information_retention.csv',infos)
    groups=defaultdict(list)
    for r in ranked:
        if r['method'] in METHODS:
            groups[r['frame'],r['pair'],r['method'],r['budget_percent']].append(r)
    summary=[]
    for (frame,pair,method,budget),g in groups.items():
        margin=[float(x['net']) for x in g]
        outcomes=[x['pair_top1_correct'] for x in g if x['pair_top1_correct']!='']
        summary.append(dict(frame=frame,pair=pair,method=method,budget_percent=budget,runs=len(g),
            a_minus_b_median=np.median(margin),a_minus_b_min=min(margin),a_minus_b_max=max(margin),
            a_wins=sum(int(x['pair_preferred'])==1 for x in g if x['pair_preferred']),
            correct_wins=sum(outcomes) if outcomes else '',
            incorrect_wins=len(outcomes)-sum(outcomes) if outcomes else '',
            median_a_scale2_wins=np.median([int(x['scale2_paired_a_wins']) for x in g]),
            fallback_median=np.median([int(x['fallback_n']) for x in g])))
    write(out/'method_budget_summary.csv',summary)
    figures(run,out,ranked,infos,summary)
    (out/'offline_evaluation_receipt.json').write_text(json.dumps(dict(mask_count=receipt['mask_count'],
        blind_seal_sha256=digest(blind/'blind_seal.json'),
        archive_labels_sha256=digest(run/'historical_labels_DO_NOT_USE_FOR_SELECTION.csv'),
        reference_threshold={'translation_m':.15,'rotation_deg':2}, n_hypotheses=len(label)),indent=2)+'\n')

def figures(run,out,rows,infos,summary):
    fig,axes=plt.subplots(2,3,figsize=(15,8.5),layout='constrained')
    selected_methods=('B1','B2','B4','B5','B6','B7_E1','B7_E2','B7_E3','GEO0','B8','B9')
    for ax,(frame,pair) in zip(axes.ravel(),[(f,n) for f,p in PAIRS.items() for n,_,_ in p]):
        group=[r for r in summary if (r['frame'],r['pair'])==(frame,pair)]
        for method in selected_methods:
            stats=[r for r in group if r['method']==method]
            budgets=[r['budget_percent'] for r in stats]
            ax.plot(budgets,[r['a_minus_b_median'] for r in stats],'.-',label=method)
        baseline=next(r for r in group if r['method']=='B0')
        ax.scatter(100,baseline['a_minus_b_median'],c='k',s=20,label='B0')
        ax.axhline(0,color='black',lw=.6)
        ax.set_title(f'{frame} / {pair}')
        ax.set_xlabel('Source budget (%)')
    axes[0,0].set_ylabel('Raw Native reward A − B')
    axes[1,0].set_ylabel('Raw Native reward A − B')
    axes[1,-1].legend(fontsize=6,ncol=2)
    fig.savefig(out/'ranking_by_budget.png',dpi=160)
    plt.close(fig)
    h=[r for r in infos if r['frame']=='H307' and r['pair']=='primary'
       and r['candidate_id']=='H307_P2648' and r['budget_percent']=='50' and int(r['seed'])==-1
       and r['method'] in selected_methods]
    fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
    ax.bar([r['method'] for r in h],[float(r['lambda_min_ratio']) for r in h])
    ax.axhline(.5,c='k',ls='--',label='50% of full')
    ax.set_ylabel('Weakest eigenvalue / full-cloud value')
    ax.tick_params(axis='x',rotation=30)
    ax.legend()
    fig.savefig(out/'information_weak_direction_h307.png',dpi=170)
    plt.close(fig)
    packet=np.load(run/'H307.npz')
    source=packet['source']
    scores=packet['score']
    delta=scores[0,0]-scores[1,0]
    with np.load(run/'blind/shared_masks.npz') as masks:
        fig,axs=plt.subplots(1,3,figsize=(11,4.2),layout='constrained')
        axs[0].scatter(source[:,0],source[:,2],c=delta,cmap='coolwarm',vmin=-1.5,vmax=1.5,s=4)
        axs[0].set_title('Native point reward A − B')
        for ax,method in zip(axs[1:],('B4','B7_E2')):
            mark=next(x for x in rows if x['frame']=='H307' and x['pair']=='primary' and x['method']==method and x['budget_percent']=='50')
            idx=masks[mark['mask_id']]
            ax.scatter(source[:,0],source[:,2],c='lightgray',s=2)
            ax.scatter(source[idx,0],source[idx,2],c=delta[idx],cmap='coolwarm',vmin=-1.5,vmax=1.5,s=4)
            ax.set_title(method+' 50%')
        for ax in axs:
            ax.set_aspect('equal')
            ax.set_xlabel('Sensor x (m)')
        axs[0].set_ylabel('Sensor z (m)')
        fig.savefig(out/'h307_native_contribution_masks.png',dpi=170)
    write_pcd(run/'H307_native_support_sign.pcd',source,np.where(delta>.05,1,np.where(delta<-.05,2,0)))
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('out',type=Path)
    args=parser.parse_args()
    main(args.run,args.out)
