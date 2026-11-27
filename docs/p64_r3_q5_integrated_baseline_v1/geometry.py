"""CloudCompare M3C2 and a sensor-ray adapter for actual fixed candidates."""
from itertools import combinations
from pathlib import Path
import os
import subprocess
import sys
import time
import numpy as np
from scipy.spatial import cKDTree

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'p64_r3_q1_observation_contribution'))
# This directory also has geometry.py, so explicitly load the historic file.
from importlib.util import module_from_spec,spec_from_file_location
spec=spec_from_file_location('r3_geometry',HERE.parent/'p64_r3_q1_observation_contribution'/'geometry.py')
g=module_from_spec(spec)
spec.loader.exec_module(g)

def read_pcd(path):
    header,body=path.read_bytes().split(b'DATA binary\n',1)
    fields=next(x.split()[1:] for x in header.splitlines() if x.startswith(b'FIELDS '))
    n=int(next(x.split()[1] for x in header.splitlines() if x.startswith(b'POINTS ')))
    return np.frombuffer(body,'<f4',count=n*len(fields)).reshape(n,len(fields))[:,:3].astype(float)

def m3c2(a,b,work):
    work.mkdir(exist_ok=False)
    np.savetxt(work/'a.xyz',a,fmt='%.8f')
    np.savetxt(work/'b.xyz',b,fmt='%.8f')
    env=dict(os.environ,QT_QPA_PLATFORM='offscreen')
    env.pop('LD_LIBRARY_PATH',None)
    command=['CloudCompare','-SILENT','-AUTO_SAVE','OFF','-C_EXPORT_FMT','ASC','-ADD_HEADER',
             '-O','a.xyz','-O','b.xyz','-M3C2',str(HERE/'m3c2.ini'),'-SAVE_CLOUDS']
    with (work/'m3c2.log').open('x') as log:
        subprocess.run(command,cwd=work,env=env,check=True,stdout=log,stderr=subprocess.STDOUT,timeout=3)
    files=list(work.glob('a_M3C2_*.asc'))
    if len(files)!=1:raise ValueError('no unique native M3C2 output')
    with files[0].open() as f:
        fields=f.readline().lstrip('/').split()
        values=np.loadtxt(f)
    d=values[:,fields.index('M3C2_distance')]
    u=values[:,fields.index('distance_uncertainty')]
    support=(values[:,fields.index('Npoints_cloud1')]>=5)&(values[:,fields.index('Npoints_cloud2')]>=5)
    if not len(d):raise ValueError('empty M3C2 core output')
    # CloudCompare projects the saved coordinates. Reverse the signed normal shift
    # and re-associate to the input core cloud, with explicit origin tolerance.
    corrected=values[:,:3]-d[:,None]*values[:,[fields.index(k) for k in ('Nx','Ny','Nz')]]
    valid=support&np.isfinite(d)&np.isfinite(u)&np.isfinite(corrected).all(axis=1)
    distances,indices=cKDTree(a).query(corrected[valid])
    allowed=distances<.045
    mapped=np.full(len(a),np.nan); uncertainty=mapped.copy()
    mapped[indices[allowed]]=d[valid][allowed]
    uncertainty[indices[allowed]]=u[valid][allowed]
    return mapped,uncertainty

def live(out,map_path):
    cloud=read_pcd(out/'source.pcd')
    world=read_pcd(map_path); poses=np.loadtxt(out/'poses.tsv').reshape(-1,4,4)
    dist=np.linalg.norm(cloud,axis=1)
    rays=cloud/np.maximum(dist[:,None],1e-9)
    views=[g.make_view(world,t,np.repeat(t[None],26,axis=0),rays,dist) for t in poses]
    different=np.zeros(len(cloud),bool); common=np.zeros(len(cloud),bool)
    support=np.zeros(len(cloud),bool)
    all_common=np.ones(len(cloud),bool)
    nn_alternative=np.zeros(len(cloud),bool)
    for i,j in combinations(range(len(views)),2):
        va,vb=views[i],views[j]
        ok=va['good'] & vb['good'] & (dist>=.4)&(dist<=8)
        if not ok.any():
            all_common[:]=False
            continue
        a=va['map']; b=vb['map']
        ac=a[::max(1,(len(a)+5999)//6000)]; bc=b[::max(1,(len(b)+5999)//6000)]
        x=a[va['ids'][ok]]; y=b[vb['ids'][ok]]
        dr=cKDTree(b).query(x)[0]
        dl=cKDTree(a).query(y)[0]
        positions=np.flatnonzero(ok)
        nn_alternative[positions]|=np.maximum(dl,dr)>=.05
        d,u=m3c2(ac,bc,out/f'm3c2_{i}_{j}')
        point_dist,nearest=cKDTree(ac).query(x)
        usable=(point_dist<=.12)&np.isfinite(d[nearest])&np.isfinite(u[nearest])
        supported=positions[usable]
        pair_close=np.zeros(len(cloud),bool)
        pair_close[supported]=np.abs(d[nearest[usable]])+u[nearest[usable]]<=.03
        different[supported]|=np.abs(d[nearest[usable]])-u[nearest[usable]]>=.05
        support[supported]=True
        all_common&=pair_close
    common=all_common&support&~different
    mask=np.flatnonzero(different)
    blocks=len(np.unique(np.floor(cloud[mask]/.5),axis=0))
    (out/'mask.tsv').write_text(f'OK {common.sum()} {len(cloud)-common.sum()-different.sum()} {blocks} {support.sum()}\n'+
                                '\n'.join(map(str,mask))+'\n')
    np.savez_compressed(out/'geometry_evidence.npz',source=cloud,mask=mask,common=common,support=support,nearest_neighbor=nn_alternative)

if __name__=='__main__':live(Path(sys.argv[1]),Path(sys.argv[2]))
