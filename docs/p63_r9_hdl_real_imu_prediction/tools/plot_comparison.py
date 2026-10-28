"""Actual continuous UKF and actual NDT guesses only; no assembled candidates."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
A=Path(__file__).resolve().parents[1];r=list(csv.DictReader((A/'PREDICTION_COMPARISON.csv').open()));x=[int(a['raw_ordinal']) for a in r]
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
for ax,key,label in zip(axes,['translation_error_m','rotation_error_deg','pred_rotation_error_deg'],['UKF position error (m)','UKF orientation error (deg)','NDT initial orientation error (deg)']):
    for method,title in [('r8','R8 original HDL: IMU off'),('r9','R9 original HDL: real IMU')]:ax.plot(x,[float(a[method+'_'+key]) for a in r],label=title,lw=1.5)
    ax.set_ylabel(label);ax.grid(alpha=.3)
axes[0].axhline(.5,color='gray',ls='--');axes[1].axhline(5,color='gray',ls='--');axes[0].legend();axes[2].set_xlabel('Original scan ordinal')
fig.suptitle('Koide indoor_hard_01: complete raw5–164, map-anchored reference; no alignment')
fig.tight_layout();(A/'figures').mkdir(exist_ok=True);fig.savefig(A/'figures/continuous_comparison.png',dpi=160);plt.close(fig)
