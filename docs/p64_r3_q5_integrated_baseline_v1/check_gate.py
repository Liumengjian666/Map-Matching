"""Validate shadow replay equivalence without consuming reference."""
import csv
import json
from pathlib import Path
import sys
import numpy as np

def read(p):
    with p.open() as s:return list(csv.DictReader(s))

def inspect(base,trial):
    a,b=[read(p/'trajectory.csv') for p in (base,trial)]
    da,db=[read(p/'diagnostics.csv') for p in (base,trial)]
    assert len(a)==len(b)==len(da)==len(db)
    identity=('stamp_ns','iterations','converged','supported','source','imu_count','latest_imu_ns')
    keys=[f'{p}{i}{j}' for p in ('pred','ndt','state') for i in range(4) for j in range(4)]
    delta=0.;p_diff=0.
    for x,y,xx,yy in zip(a,b,da,db):
        assert all(x[k]==y[k] for k in identity)
        assert int(y['latest_imu_ns'])<=int(y['stamp_ns'])
        assert y['updates']=='1' and y['last_update_ns']==y['stamp_ns']
        delta=max(delta,max(abs(float(x[k])-float(y[k])) for k in keys))
        dx=np.fromstring(xx['covariance_after'],sep=';')
        dy=np.fromstring(yy['covariance_after'],sep=';')
        assert len(dx)==len(dy)==256
        p_diff=max(p_diff,np.abs(dx-dy).max())
    assert delta<=1e-6 and p_diff<=1e-6,(delta,p_diff)
    return dict(frames=len(a),max_matrix_abs=delta,max_covariance_abs=float(p_diff),updates=len(a),duplicate=0)

if __name__=='__main__':
    a,b=map(Path,sys.argv[1:3])
    print(json.dumps(inspect(a,b),indent=2))
