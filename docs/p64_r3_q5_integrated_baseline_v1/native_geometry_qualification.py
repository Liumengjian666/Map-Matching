"""Run installed reference M3C2 on a planar model fitted to a real map patch."""
from pathlib import Path
import csv
import sys
import time
import numpy as np
from geometry import m3c2

def main(run,output):
    actual=np.loadtxt(run/'cc/same_real_surface/a.xyz')
    center=actual.mean(axis=0)
    _,v=np.linalg.eigh(np.cov((actual-center).T))
    norm=v[:,0]
    plane=np.stack(np.meshgrid(np.linspace(-.6,.6,41),np.linspace(-.6,.6,41)),axis=-1).reshape(-1,2)@v[:,1:].T+center
    rng=np.random.default_rng(6411)
    comparison=[('dense_same',plane,0.),('dense_half',plane[rng.random(len(plane))<.5],0.),
                ('dense_2cm',plane+.02*norm,.02),('dense_5cm',plane+.05*norm,.05),
                ('dense_10cm',plane+.10*norm,.10),('dense_roughness',plane+rng.normal(0,.02,(len(plane),1))*norm,0.)]
    out=[]
    for name,data,expected in comparison:
        start=time.perf_counter()
        score,limit=m3c2(plane,data,run/'cc'/name)
        keep=np.isfinite(score)&np.isfinite(limit)
        out.append(dict(case=name,mode='ORIGINAL_CLOUDCOMPARE',n1=len(plane),n2=len(data),valid=int(keep.sum()),
                 median_distance_m=float(np.nanmedian(np.abs(score))),median_lod_m=float(np.nanmedian(limit)),
                 expected_offset_m=expected,matched_significant=int((keep&(np.abs(score)>limit)).sum()),
                 elapsed_s=time.perf_counter()-start))
    with output.open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(out[0]),lineterminator='\n')
        w.writeheader();w.writerows(out)
if __name__=='__main__':main(Path(sys.argv[1]),Path(sys.argv[2]))
