#!/usr/bin/env python3
"""Label-blind, frozen-cluster near-optimal admission; no NDT or GT reader."""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import sys
import tempfile
import time

import numpy as np
import p9_r2b_nonoracle_evidence as g
import p9_r4_contract as c

START = "5a95362861464d9b8756e444ea3dcf999e032d6a"
OUT = c.ROOT / "docs/p9_r6_nonlocal_admission"
R5 = c.ROOT / "docs/p9_r5_candidate_bottleneck"
LIB = Path("/tmp/p9_r4_release.Eirto1/libp9_r2b_chart.so")
LIB_SHA = "2a71c945f5b23b1eacdf4f73a63aba86fd509d2c556392652a0eda7c723cb711"
GAMMA = .05


def near(score, best, count):
    g.require(count > 0 and np.isfinite([score, best]).all(), "invalid score/source")
    return (best-score)/count <= GAMMA*max(1., abs(best/count))


def tensor(vectors):
    a = sum((np.outer(v,v) for v in vectors), np.zeros((6,6)))
    eigen = np.linalg.eigvalsh(a)
    g.require(eigen[0] >= -1e-10*max(1.,np.trace(a)), "non-PSD evidence")
    return g.text(a), float(np.sqrt(max(0., eigen[-1]))), float(np.trace(a))


def construct(frame, method, nominal, points, clusters, count, chart):
    best = max([nominal['score']] + [float(r['raw_score']) for r in clusters])
    nominal_near = near(nominal['score'], best, count)
    candidates=[];vectors=[];strict=0;separated=0;straddling=0;suppressed=0
    for r in clusters:
        ranks=[int(x) for x in r['members_probe_ranks'].split(';')]
        members=[points[k] for k in ranks]
        rep=max(members, key=lambda p:(p['score'],-p['rank']))
        g.require(rep['rank']==int(r['representative_rank']) and rep['matrix']==r['representative_pose_matrix16']
                  and rep['score']==float(r['raw_score']), 'representative parity failure')
        g.require(int(0 in ranks)==int(r['is_nominal']), 'nominal membership parity failure')
        dt,dr=g.separation(nominal['pose'],rep['pose'])
        outside=dt>.2 or dr>2
        strict_here=not int(r['is_nominal']) and outside and rep['score']>=nominal['score']+g.EPS_SCORE
        g.require(outside==bool(int(r['representative_outside_nominal_ball'])) and
            abs(dt-float(r['translation_from_nominal_m']))<1e-12 and
            abs(dr-float(r['rotation_from_nominal_deg']))<1e-10 and
            bool(int(r['competitive']))==(not int(r['is_nominal']) and rep['score']>=nominal['score']+g.EPS_SCORE),
            'frozen cluster geometry/strict parity failure')
        flags=[(lambda d:d[0]>.2 or d[1]>2)(g.separation(nominal['pose'],p['pose'])) for p in members]
        boundary=any(flags) and not all(flags)
        eligible=not int(r['is_nominal']) and outside and near(rep['score'],best,count)
        member_near_out=sum(flag and near(p['score'],best,count) for p,flag in zip(members,flags))
        suppressed_here=member_near_out>0 and not eligible
        xi=chart(g.vector(nominal['matrix'],16),g.vector(rep['matrix'],16))
        candidates.append(dict(frame=frame,method=method,cluster_id=r['cluster_id'],
            members_probe_ranks=r['members_probe_ranks'],member_count=len(members),
            representative_rank=rep['rank'],representative_pose_matrix16=rep['matrix'],
            S0=nominal['score'],S_star=best,S_k=rep['score'],N=count,
            delta=(best-rep['score'])/count,tau=GAMMA*max(1.,abs(best/count)),
            near_optimal=int(near(rep['score'],best,count)),nominal_near=int(nominal_near),
            translation_m=dt,rotation_deg=dr,strict_separated=int(outside),
            is_nominal=int(r['is_nominal']),strict_eligible=int(strict_here),
            near_eligible=int(eligible),boundary_straddling=int(boundary),
            outside_members=sum(flags),near_outside_members=member_near_out,
            representative_suppression=int(suppressed_here),xi=g.text(xi)))
        if eligible:vectors.append(xi)
        strict+=int(strict_here);separated+=int(outside and not int(r['is_nominal']))
        straddling+=int(boundary);suppressed+=int(suppressed_here)
    event=('TYPE_A' if nominal_near else 'TYPE_B') if vectors else 'NONE'
    a,ua,ta=tensor(vectors if event=='TYPE_A' else [])
    b,ub,tb=tensor(vectors if event=='TYPE_B' else [])
    frame_row=dict(frame=frame,method=method,N=count,S0=nominal['score'],S_star=best,
        nominal_delta=(best-nominal['score'])/count,tau=GAMMA*max(1.,abs(best/count)),
        nominal_near=int(nominal_near),event_type=event,strict_count=strict,
        cluster_count=len(clusters),separated_cluster_count=separated,eligible_cluster_count=len(vectors),
        eligible_member_count=sum(r['member_count'] for r in candidates if r['near_eligible']),
        eligible_outside_member_count=sum(r['outside_members'] for r in candidates if r['near_eligible']),
        boundary_cluster_count=straddling,suppressed_cluster_count=suppressed,
        A_near=a,U_near=ua,trace_near=ta,A_disfavored=b,U_disfavored=ub,trace_disfavored=tb)
    return candidates,frame_row


def self_test():
    g.require(near(95,100,100) and not near(np.nextafter(95.,-np.inf),100,100), 'inclusive score boundary')
    g.require(near(-105,-100,100) and not near(-106,-100,100), 'negative score absolute band')
    g.require(near(190,200,100) and not near(189,200,100), 'normalized score scaling')
    chart=g.chart_function(LIB);g.self_test(chart)
    mat=lambda x:g.text(np.array([[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]],float))
    def fixture(scores):
        rows=[dict(probe_rank=str(i+1),source_points='100',nominal_ndt_score_sum='100',
            converged='1',iterations='80',terminal_pose_matrix16=mat(x),
            terminal_pose_xyz_q_xyzw=f'{x};0;0;0;0;0;1',raw_ndt_score_sum=str(s))
            for i,(x,s) in enumerate(scores)]
        clusters,_,_=g.evidence_for_frame(1,len(rows),mat(0),rows,chart)
        nominal=g.entry(0,mat(0),100);points={0:nominal}
        points.update({i+1:g.entry(i+1,r['terminal_pose_matrix16'],float(r['raw_ndt_score_sum']),r['terminal_pose_xyz_q_xyzw']) for i,r in enumerate(rows)})
        return construct(1,'TEST',nominal,points,clusters,100,chart)
    _,a=fixture([(1,99)])
    g.require(a['event_type']=='TYPE_A' and a['U_near']>0 and a['U_disfavored']==0 and a['strict_count']==0,'TYPE A regression')
    _,b=fixture([(1,120)])
    g.require(b['event_type']=='TYPE_B' and b['U_near']==0 and b['U_disfavored']>0,'TYPE B regression')
    _,none=fixture([(0.01,120),(1,90)])
    g.require(none['event_type']=='NONE' and not none['nominal_near'],'nominal cluster representative substituted for S0')
    rows,boundary=fixture([(-.05,100),(.19,102),(.21,101)])
    g.require(any(r['boundary_straddling'] and r['representative_suppression'] for r in rows)
        and boundary['event_type']=='NONE','boundary member improperly admitted')
    print('P9_R6_SCORE_TYPE_TENSOR_SELF_TEST=PASS')


def install_read_guard(allowed):
    """Enforce experiment-data reads in this process; do not trust convention alone."""
    allowed={Path(p).resolve() for p in allowed};opened=set()
    def guard(event,args):
        if event!='open' or not isinstance(args[0],(str,bytes,Path)):return
        path=Path(args[0]).resolve()
        access=args[2] & os.O_ACCMODE
        if access==os.O_WRONLY:
            g.require(OUT in path.parents,'WRITE_OUTSIDE_R6_DENIED: '+str(path))
            return
        # Python/shared-library installation resources are non-experiment inputs.
        system=any(root in path.parents for root in (Path('/usr'),Path('/lib'),Path('/lib64')))
        g.require(path in allowed or system, 'LABEL_GT_READ_GUARD_DENIED: '+str(path))
        if not system:opened.add(str(path))
    sys.addaudithook(guard)
    return opened


def isolation_test():
    handle,fixture=tempfile.mkstemp(prefix='p9_r6_copied_label_',suffix='.csv')
    os.close(handle)
    install_read_guard([])
    for path in (c.OUT/'oracle/oracle_labels.csv',c.OUT/'oracle/oracle_clusters.csv',
                 R5/'results.json',Path('/media/forbidden_gt.csv')):
        try:path.read_bytes()
        except RuntimeError as error:
            g.require('LABEL_GT_READ_GUARD_DENIED' in str(error),'unexpected isolation exception')
        else:raise RuntimeError('forbidden read succeeded')
    for mode in ('r','a+','r+'):
        try:
            with open(fixture,mode) as stream:stream.read()
        except RuntimeError as error:
            g.require('LABEL_GT_READ_GUARD_DENIED' in str(error),'readable mode incorrectly classified')
        else:raise RuntimeError('copied label/readable update mode bypassed guard')
    os.unlink(fixture)
    print('P9_R6_LABEL_GT_ISOLATION_SELF_TEST=PASS')


def grouped(rows,field='frame'):
    groups=defaultdict(list)
    for r in rows:groups[int(r[field])].append(r)
    return groups


def build():
    tick=time.perf_counter()
    g.require(not (OUT/'evidence_freeze.json').exists(),'evidence already frozen')
    paths=dict(weak=c.OUT/'candidate/conditioned_weak_runs.csv',full=c.OUT/'oracle/oracle_candidates.csv',
        bclusters=c.OUT/'candidate/terminal_clusters.csv',fclusters=R5/'full263_terminal_clusters.csv',
        cohort=c.OUT/'heldout_final_cohort.csv',r4_hash=c.OUT/'artifact_hashes.json',r5_hash=R5/'artifact_hashes.json')
    outputs=('nearoptimal_candidates.csv','nominal_disfavored.csv','boundary_straddling.csv',
        'frame_level_evidence.csv','cluster_representative_parity.csv','runtime_breakdown.csv')
    helpers=('p9_r2b_nonoracle_evidence.py','p9_r2b_chart.cpp','p9_r4_contract.py')
    reads=install_read_guard(list(paths.values())+[OUT/'THEORY.md',Path(__file__),LIB]+
        [c.HERE/name for name in helpers]+[OUT/name for name in outputs])
    hashes={str(p.relative_to(c.ROOT)):c.pinned(p,START) for p in paths.values()}
    h4=json.loads(paths['r4_hash'].read_text())['artifact_sha256']
    h5=json.loads(paths['r5_hash'].read_text())['artifacts']
    for name in ('weak','full','bclusters','cohort'):
        g.require(c.digest(paths[name])==h4[str(paths[name].relative_to(c.OUT))],'R4 input hash chain failed')
    g.require(c.digest(paths['fclusters'])==h5['full263_terminal_clusters.csv'],'R5 cluster hash chain failed')
    g.require(c.digest(LIB)==LIB_SHA,'chart library changed')
    frozen={name:c.pinned(c.HERE/name,START) for name in helpers}
    data={key:c.read_csv(paths[key]) for key in ('weak','full','bclusters','fclusters','cohort')}
    g.require(all('label' not in rows[0] for rows in data.values()),'labeled input entered builder')
    ids=[int(r['transaction_id']) for r in data['cohort']]
    g.require(len(ids)==len(set(ids))==96,'cohort changed')
    groups={key:grouped(data[key]) for key in ('weak','full','bclusters','fclusters')}
    g.require(all(set(v)==set(ids) for v in groups.values()),'input frame mismatch')
    load_seconds=time.perf_counter()-tick
    candidates=[];frames=[];parity=[];runtime=[];chart=g.chart_function(LIB)
    for tx in ids:
        weak=sorted(groups['weak'][tx],key=lambda r:int(r['probe_rank']))
        nominal=g.entry(0,weak[0]['nominal_pose_matrix16'],float(weak[0]['nominal_ndt_score_sum']))
        n=int(weak[0]['source_points']);source=weak[0]['source_hash_actual']
        for method,key,ckey,budget in (('B12','weak','bclusters',12),('FULL263','full','fclusters',263)):
            start=time.perf_counter()
            raw=groups[key][tx];clusters=groups[ckey][tx];points={0:nominal}
            g.require(len(raw)==budget,'incomplete terminal pool')
            for row in raw:
                rank=int(row['probe_rank']) if method=='B12' else int(row['seed_index'])+1
                g.require(float(row['nominal_ndt_score_sum'])==nominal['score'] and int(row['source_points'])==n
                    and row['source_hash_actual']==row['source_hash_expected']==source,'source/S0 mismatch')
                g.require(row['converged'] in ('0','1'),'invalid convergence flag')
                if row['converged']=='1':
                    g.require(rank not in points,'duplicate terminal rank')
                    points[rank]=g.entry(rank,row['terminal_pose_matrix16'],float(row['raw_ndt_score_sum']),
                        row['terminal_pose_xyz_q_xyzw'],int(row['iterations']))
            assignments=[int(x) for r in clusters for x in r['members_probe_ranks'].split(';')]
            g.require(len(assignments)==len(set(assignments)) and set(assignments)==set(points),'cluster partition mismatch')
            g.require(all(int(r['budget'])==budget for r in clusters),'cluster budget mismatch')
            verify_time=time.perf_counter()-start
            evidence_start=time.perf_counter()
            cs,fr=construct(tx,method,nominal,points,clusters,n,chart)
            evidence_time=time.perf_counter()-evidence_start
            candidates.extend(cs);frames.append(fr)
            parity.append(dict(frame=tx,method=method,clusters=len(clusters),members=len(points),
                strict_count=fr['strict_count'],cluster_source_sha256=hashes[str(paths[ckey].relative_to(c.ROOT))],parity='PASS'))
            runtime.append(dict(frame=tx,method=method,cached_partition_verification_seconds=verify_time,
                representative_verification_and_evidence_seconds=evidence_time,
                cluster_recomputation_seconds=0.,NEW_NDT_CALLS=0))
    c.write_csv(OUT/'nearoptimal_candidates.csv',candidates)
    c.write_csv(OUT/'nominal_disfavored.csv',[r for r in frames if r['event_type']=='TYPE_B'],list(frames[0]))
    c.write_csv(OUT/'boundary_straddling.csv',[r for r in candidates if r['boundary_straddling']],list(candidates[0]))
    c.write_csv(OUT/'frame_level_evidence.csv',frames)
    c.write_csv(OUT/'cluster_representative_parity.csv',parity)
    c.write_csv(OUT/'runtime_breakdown.csv',runtime)
    c.save_json(OUT/'evidence_freeze.json',dict(state='EVIDENCE_FROZEN_BEFORE_LABELS',start_sha=START,
        input_sha256=hashes,frozen_source_sha256=frozen,chart_library_sha256=LIB_SHA,
        code_sha256=c.digest(__file__),theory_sha256=c.digest(OUT/'THEORY.md'),
        artifacts={name:c.digest(OUT/name) for name in outputs},gamma=GAMMA,
        allowed_data_reads=sorted(reads),ORACLE_LABELS_LOADED_BY_EVIDENCE=False,GT_LOADED=False,
        NEW_NDT_CALLS=0,BASELINE_REPLAY_CALLS=0,VISUAL_EXTRACTION=0,
        input_hash_and_load_seconds=load_seconds,offline_build_wall_seconds=time.perf_counter()-tick))
    print('P9_R6_BLIND_EVIDENCE_FREEZE=PASS; rows='+str(len(frames)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('self-test','isolation-test','build'))
    args=parser.parse_args()
    {'self-test':self_test,'isolation-test':isolation_test,'build':build}[args.action]()
