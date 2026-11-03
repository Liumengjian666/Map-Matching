"""Only scheduling/diagnostic rules; the R3 validity and stopping semantics are unchanged."""
import csv,math
from pathlib import Path
import numpy as np
class CsvTail:
    def __init__(self,path):self.path=Path(path);self.file=None;self.header=None
    def poll(self):
        if self.file is None:
            if not self.path.exists():return None
            self.file=self.path.open()
        start=self.file.tell();line=self.file.readline()
        if not line or not line.endswith('\n'):
            self.file.seek(start);return None
        cells=next(csv.reader([line]))
        if self.header is None:self.header=cells;return self.poll()
        if len(cells)!=len(self.header):raise RuntimeError('DIAGNOSTIC_CSV_SHAPE')
        return dict(zip(self.header,cells))
    def close(self):
        if self.file:self.file.close()
def assess(v,raw,native_converged,status_converged,fitness,error,inlier,previous):
    v=np.asarray(v,float);raw=np.asarray(raw,float);previous=np.asarray(previous,float)
    finite=bool(np.isfinite(v).all() and .99<np.linalg.norm(v[3:])<1.01)
    finite=finite and bool(np.isfinite(raw).all())
    valid=finite and native_converged and status_converged and math.isfinite(fitness) and math.isfinite(error) and inlier>=.50
    dt=float(np.linalg.norm(v[:3]-previous[:3])) if finite else float('nan')
    dr=float(np.degrees(2*np.arccos(np.clip(abs(v[3:]@previous[3:])/(np.linalg.norm(v[3:])*np.linalg.norm(previous[3:])),0,1)))) if finite else float('nan')
    return finite,bool(valid),dt,dr
