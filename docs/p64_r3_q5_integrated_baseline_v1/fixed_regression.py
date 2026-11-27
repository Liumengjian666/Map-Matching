"""Fixed-pose tests on frozen actual candidates, separate from continuous UKF."""
from pathlib import Path
import csv
import sys
import numpy as np
from geometry import live,read_pcd,g

P=Path('/home/jian/p64_r3_q1_ws/run')
Q=Path('/home/jian/p64_r3_q3_ws/run')
MAP=Path('/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_048_diff_map_formal_01/nav_a_diff_prior_voxel0p15.pcd')

def pcd(path,p):
    p=np.asarray(p,dtype='<f4',order='C')
    h=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nPOINTS {len(p)}\nDATA binary\n'
    path.write_bytes(h.encode()+p.tobytes())

def main(run,out):
    with (Q/'mid_fixed_manifest.csv').open() as f:manifest=list(csv.DictReader(f))
    rows=[]
    for frame in ['H307','H515','H309','B25','W2_32']:
        work=run/frame;work.mkdir()
        if frame.startswith('H'):
            with np.load(P/f'{frame}.npz') as r:
                points=r['source'];world=r['map'];poses=r['nominal']
                scores=r['score'][:,0]
            map_path=work/'map.pcd';pcd(map_path,world)
        else:
            split=[r for r in manifest if r['frame']==frame]
            points=np.fromfile(Q/'mid_fixed'/f'{frame}_filtered_source_xyz.bin','<f4').reshape(-1,3)
            poses=np.array([[float(r[f'T{i}{j}']) for i in range(4) for j in range(4)] for r in split]).reshape(-1,4,4)
            scores=np.stack([np.genfromtxt(Q/'mid_fixed'/f"{r['candidate_id']}_points.csv",names=True,delimiter=',')['native_reward']
                             for r in split])
            map_path=MAP
        pcd(work/'source.pcd',points)
        np.savetxt(work/'poses.tsv',poses.reshape(-1,16))
        live(work,map_path)
        with np.load(work/'geometry_evidence.npz') as e:
            chosen=e['mask']
            world=read_pcd(map_path)
            data,_=g.compute(points.astype(float),world,poses.astype(float),np.repeat(poses[:,None],26,axis=1).astype(float))
            original=data['pair_observable'][:,:,0].any(axis=0)
            v=np.where(chosen.size,np.sum(scores[:,chosen],axis=1),0)
            total=scores.sum(axis=1); select=scores[:,original].sum(axis=1)
            if frame=='B25':
                assert original.sum()==34
                assert np.argmax(total)==0 and np.argmax(select)==1
            rows.append(dict(frame=frame,candidates=len(poses),full_native_best=int(np.argmax(total)),
                support_only_best=int(np.argmax(select)),original_support=original.sum(),
                m3c2_support=e['support'].sum(),common=e['common'].sum(),differences=len(chosen),
                score_if_differences=';'.join(map(str,v)) if len(chosen) else '',
                nearest_neighbor_distinct=e['nearest_neighbor'].sum(),
                support_margin_a_b=select[0]-select[1],full_margin_a_b=total[0]-total[1],
                result='SHARED_M3C2_RANKED' if len(chosen) else 'NO_DISTINCT_EVIDENCE'))
    with out.open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n')
        w.writeheader();w.writerows(rows)

if __name__=='__main__':main(Path(sys.argv[1]),Path(sys.argv[2]))
