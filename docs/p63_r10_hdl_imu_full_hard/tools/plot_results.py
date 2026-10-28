"""Display sealed estimates/errors only; never run localization or parse new GT."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
A=Path(__file__).resolve().parents[1];O=A/'figures';O.mkdir(exist_ok=True)
r=list(csv.DictReader((A/'FULL_TRAJECTORY.csv').open()))
x=np.array([int(a['raw_ordinal']) for a in r])
v=lambda k:np.array([float(a[k]) if a[k] else np.nan for a in r])
fig,ax=plt.subplots(3,1,figsize=(10,9),sharex=True)
ax[0].plot(x,v('translation_error_m'));ax[0].set_yscale('symlog',linthresh=1);ax[0].axhline(.5,color='r',ls='--');ax[0].set_ylabel('Position error (m, symlog)')
ax[1].plot(x,v('rotation_error_deg'));ax[1].axhline(5,color='r',ls='--');ax[1].set_ylabel('Orientation error (deg)')
ax[2].plot(x,v('inlier_fraction'),label='Native cropped inlier');ax[2].set_ylim(-.02,1.05);ax[2].set_ylabel('Native inlier fraction');ax[2].set_xlabel('Raw scan ordinal (entire evaluated sequence)')
for a in ax:
 a.axvspan(309,313,color='orange',alpha=.2);a.axvspan(515,2379,color='red',alpha=.08);a.grid(alpha=.25)
fig.suptitle('Frozen HDL + real IMU: full Hard sequence failure\nMap-anchored GICP+IMU reference; no alignment, no reset')
fig.tight_layout();fig.savefig(O/'full_sequence_errors.png',dpi=160);fig.savefig(O/'full_sequence_errors.pdf');plt.close(fig)
s=(x>=299)&(x<=328);fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
for p,n in [('pred_','IMU prediction'),('ndt_','Native NDT'),('','UKF output')]:
 ax[0].plot(x[s],v(p+'translation_error_m')[s],'.-',label=n)
 ax[1].plot(x[s],v(p+'rotation_error_deg')[s],'.-',label=n)
ax[0].axhline(.5,color='r',ls='--');ax[1].axhline(5,color='r',ls='--');ax[0].set_ylabel('Position error (m)');ax[1].set_ylabel('Orientation error (deg)');ax[1].set_xlabel('Raw scan ordinal')
for a in ax:a.axvspan(309,313,color='orange',alpha=.2);a.grid(alpha=.25);a.legend()
fig.suptitle('First sustained failure: frozen ten-before / twenty-from-onset diagnosis')
fig.tight_layout();fig.savefig(O/'first_failure.png',dpi=160);fig.savefig(O/'first_failure.pdf');plt.close(fig)
