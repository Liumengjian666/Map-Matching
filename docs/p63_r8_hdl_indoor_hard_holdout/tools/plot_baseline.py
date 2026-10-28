"""Plot only the sealed original UKF trajectory; no stitched candidate trajectory."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
A=Path(__file__).resolve().parents[1]
with (A/'BASELINE_TRAJECTORY.csv').open() as s:rows=list(csv.DictReader(s))
fig,axes=plt.subplots(2,1,figsize=(10,5.5),sharex=True,constrained_layout=True)
for ax,key,threshold,label in zip(axes,['translation_error_m','rotation_error_deg'],[.5,5],['Translation error (m)','Rotation error (deg)']):
    ax.plot([int(r['raw_ordinal']) for r in rows],[float(r[key]) for r in rows],label='Original HDL UKF (no IMU)',linewidth=1)
    ax.axhline(threshold,color='gray',linestyle='--',linewidth=.8)
    for raw in [30,55,80,105,130,155]:ax.axvline(raw,color='gray',alpha=.25,linewidth=.8)
    ax.axvspan(84,141,color='tab:red',alpha=.1,label='Sustained episode by frozen 3-frame rule')
    ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlim(5,164)
axes[-1].set_xlabel('Hard original scan ordinal (vertical lines: preregistered six scans)')
fig.suptitle('160 original causal outputs; map-anchored GICP+IMU reference\nNo offline candidate was fed back into this trajectory')
(A/'figures').mkdir(exist_ok=True);fig.savefig(A/'figures/baseline_errors.png',dpi=180);plt.close(fig)
print('Saved full 160-frame original baseline error figure, align=0.')
