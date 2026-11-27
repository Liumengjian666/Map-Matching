"""Prepare independent short causal windows from sealed telemetry, without reference."""
from pathlib import Path
import csv
import json
import runpy
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from attribution import HERE,PREV,R32,FIELDS,read,digest,output

Q2=HERE.parent/'p64_r2_q2_bounded_search_adaptation'
SOURCE=runpy.run_path(str(PREV/'prepare_omni_observations.py'))
MAP=SOURCE['MAP']; FEED=SOURCE['FEED']; TRACE=SOURCE['TRACE']
WINDOW=72; ANCHOR=32

def position(row,prefix):
    return np.array([float(row[f'{prefix}{i}3']) for i in range(3)])

def matrix(row,prefix):
    return np.array([float(row[f'{prefix}{r}{c}']) for r in range(4) for c in range(4)]).reshape(4,4)

def issue_score(row):
    estimate=np.array([float(row['t'+k]) for k in 'xyz'])
    q=Rotation.from_quat([float(row['q'+k]) for k in 'xyzw']).as_matrix()
    pred=matrix(row,'guess_')
    translation=np.linalg.norm(estimate-pred[:3,3])
    rotation=np.rad2deg(Rotation.from_matrix(q.T@pred[:3,:3]).magnitude())
    return max(translation/.20,rotation/4.),translation,rotation

def prepare(run):
    source=runpy.run_path(str(Q2/'protocol_b_prepare.py'))
    assert digest(MAP)==SOURCE['MAP_SHA']
    feed=json.loads((FEED/'INPUT.json').read_text())
    actual=feed['scans']; by_stamp={r['stamp_ns']:i for i,r in enumerate(actual)}
    trace=read(TRACE/'NATIVE_NDT.csv')
    rows=[]
    for r in trace:
        stamp=int(r['stamp_ns'])
        i=by_stamp.get(stamp,-1)
        if i<100 or i+WINDOW-ANCHOR>len(actual):continue
        rows.append((issue_score(r),r,i))
    rows.sort(key=lambda x:(-x[0][0],x[2]))
    chosen=[]
    for stats,row,i in rows:
        xyz=np.array([float(row['t'+k]) for k in 'xyz'])
        if all(abs(int(row['stamp_ns'])-ts)>=30_000_000_000 and np.linalg.norm(xyz-p)>=3 for _,_,ts,p in chosen):
            chosen.append((stats,i,int(row['stamp_ns']),xyz))
        if len(chosen)==3:break
    world=source['map_xyz']()
    results=[]
    imus=read(FEED/'imu_si.csv')
    with (FEED/'scans_xyzi.bin').open('rb') as inp:
        for j,(stats,i,stamp,xyz) in enumerate(chosen,1):
            root=run/f'W{j}'
            root.mkdir()
            records=[]
            for k,r in enumerate(actual[i-ANCHOR:i-ANCHOR+WINDOW]):
                inp.seek(r['offset'])
                payload=inp.read(r['points']*16)
                import hashlib
                assert hashlib.sha256(payload).hexdigest()==r['xyzi_sha256']
                cloud=np.frombuffer(payload,dtype='<f4').reshape(-1,4)
                select=np.isfinite(cloud[:,:3]).all(axis=1)&(np.linalg.norm(cloud[:,:3],axis=1)>.20)
                cleaned=np.ascontiguousarray(cloud[select])
                name=f'scan_{k:02}.pcd'
                source['write_pcd'](root/name,cleaned)
                if k==0:first=cleaned[:,:3].astype(float)
                records.append(dict(scan_index=k,stamp_ns=r['stamp_ns'],raw=r['raw'],file=name,sha256=digest(root/name)))
            seed=source['initial_guess'](first,world)
            output(root/'scan_receipt.csv',records)
            (root/'scans.tsv').write_text(''.join(f"{r['stamp_ns']} {r['file']}\n" for r in records))
            fields=['stamp_ns','ax','ay','az','gx','gy','gz']
            with (root/'imu.tsv').open('x') as f:
                writer=csv.writer(f,delimiter='\t',lineterminator='\n')
                writer.writerows([r[k] for k in fields] for r in imus if records[0]['stamp_ns']<int(r['stamp_ns'])<=records[-1]['stamp_ns'])
            if seed['qualifies_initialization']:
                s=seed['seed']
                (root/'anchor.tsv').write_text(' '.join(map(str,[*s['translation'],s['yaw_deg']]))+'\n')
            (root/'init.json').write_text(json.dumps(seed,indent=2)+'\n')
            results.append(dict(window=root.name,anchor_time_ns=records[0]['stamp_ns'],event_time_ns=stamp,
                center_raw=actual[i]['raw'],baseline_score=stats[0],baseline_innovation_m=stats[1],
                baseline_innovation_deg=stats[2],n_planned=WINDOW,seed_score=seed['seed']['occupancy_score'],
                seed_margin=seed['absolute_margin'],qualified=seed['qualifies_initialization'],x=xyz[0],y=xyz[1],z=xyz[2]))
    output(run/'window_manifest.csv',results)

def trigger(run):
    events=[]
    with (run/'mid_plan.tsv').open('x') as f:
        for win in read(run/'window_manifest.csv'):
            root=run/win['window']
            if win['qualified']!='True':continue
            timeline=read(root/'trajectory.csv')
            if not timeline:continue
            assert len(timeline)<=WINDOW
            indices={i:row for i,row in enumerate(timeline)}
            assets=read(root/'scan_receipt.csv')
            chosen=[ANCHOR] if ANCHOR in indices else []
            for i,row in indices.items():
                if i<22 or len(chosen)==4 or any(abs(i-j)<5 for j in chosen):continue
                a,b=matrix(row,'pred'),matrix(row,'ndt')
                t=np.linalg.norm(a[:3,3]-b[:3,3]); r=np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())
                if t>=.20 or r>=4 or int(row['supported'])/int(row['source'])<.80 or row['converged']!='1' or int(row['iterations'])==0:
                    chosen.append(i)
            for i in sorted(chosen):
                row=indices[i]; asset=assets[i]
                assert asset['stamp_ns']==row['stamp_ns']
                assert int(row['latest_imu_ns'])<=int(row['stamp_ns'])
                scan=root/asset['file']
                ident=f"{root.name}_{i:02}"
                a,b=matrix(row,'pred'),matrix(row,'ndt')
                print(ident,MAP,scan,*a.ravel(),*b.ravel(),sep='\t',file=f)
                events.append(dict(frame=ident,window=root.name,frame_index=i,trigger='frozen_center' if i==ANCHOR else 'causal_threshold',
                    stamp_ns=row['stamp_ns'],source_pcd=str(scan.resolve()),source_sha256=asset['sha256'],
                    native_baseline_support=int(row['supported'])/int(row['source']),baseline_iterations=row['iterations'],
                    baseline_converged=row['converged'],baseline_innovation_m=float(np.linalg.norm(a[:3,3]-b[:3,3])),
                    baseline_innovation_deg=float(np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())),
                    **dict(zip(FIELDS,b.ravel()))))
    if events:output(run/'events.csv',events)
if __name__=='__main__':{'prepare':prepare,'trigger':trigger}[sys.argv[1]](Path(sys.argv[2]).resolve())
