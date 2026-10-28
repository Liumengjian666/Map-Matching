"""Plots only sealed online features and their separate post-seal reference labels."""
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
A=Path(__file__).resolve().parents[1];O=A/'figures';O.mkdir(exist_ok=True)
f=list(csv.DictReader((A/'CAUSAL_FEATURES.csv').open()));r=list(csv.DictReader((A/'REFERENCE_DIAGNOSIS.csv').open()))
x=np.array([int(a['raw_ordinal']) for a in r]);v=lambda rows,k:np.array([float(a[k]) if a[k] else np.nan for a in rows])
for w,raw in [('B',309),('C',515)]:
 s=np.array([a['window']==w for a in r]);fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
 for p,label in [('pred','Prediction'),('ndt','NDT'),('ukf','UKF (post-fusion)')]:
  ax[0].plot(x[s],v(r,p+'_t_m')[s],'.-',label=label);ax[1].plot(x[s],v(r,p+'_R_deg')[s],'.-',label=label)
 for a in ax:a.axvline(raw,c='r',ls='--');a.grid(alpha=.2);a.legend()
 ax[0].set_ylabel('Position error (m)');ax[1].set_ylabel('Orientation error (deg)');ax[1].set_xlabel('Raw ordinal')
 fig.suptitle('Post-seal reference diagnosis: window '+w+'\nMap-anchored GICP+IMU reference, not independent mocap')
 fig.tight_layout();fig.savefig(O/('reference_window_'+w+'.png'),dpi=140);plt.close(fig)
normal=np.array([a['normal_ndt_0_15m_2deg']=='True' for a in r]);beneficial=np.array([a['both_improve']=='True' for a in r])
fig,ax=plt.subplots(figsize=(9,6));ax.scatter(v(f,'d_t_m'),v(f,'d_R_deg'),c='lightgray',label='All 90 (online features)');ax.scatter(v(f,'d_t_m')[normal],v(f,'d_R_deg')[normal],label='Low reference error NDT (offline label)');ax.scatter(v(f,'d_t_m')[beneficial],v(f,'d_R_deg')[beneficial],marker='+',c='green',label='Both reference errors reduced (offline label)')
for k in [182,309,314,515]:
 i=list(x).index(k);ax.annotate('raw'+str(k),(v(f,'d_t_m')[i],v(f,'d_R_deg')[i]),xytext=(5,7),textcoords='offset points');ax.scatter([v(f,'d_t_m')[i]],[v(f,'d_R_deg')[i]],facecolors='none',edgecolors='red',s=100)
ax.set_xlabel('Online translation correction (m)');ax.set_ylabel('Online relative rotation correction (deg)');ax.set_title('Large correction is not equivalent to harmful correction\nNo rejection threshold fitted or applied');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(O/'correction_overlap.png',dpi=140);plt.close(fig)
fig,ax=plt.subplots(2,2,figsize=(11,7))
for row,key in enumerate(['gaussian','native_inlier']):
 for col,err in enumerate(['delta_error_t_m','delta_error_R_deg']):
  a=ax[row,col];a.scatter(v(f,key),v(r,err),c='gray',s=17);a.axhline(0,c='k',lw=.8)
  for k in [309,314,515]:
   i=list(x).index(k);a.scatter(v(f,key)[i],v(r,err)[i],c='r');a.annotate(str(k),(v(f,key)[i],v(r,err)[i]),xytext=(4,3),textcoords='offset points')
  a.set_xlabel(key+' (online)');a.set_ylabel(err+' (post-seal)');a.grid(alpha=.2)
fig.suptitle('Native quality vs change in reference error\nPositive: worsened dimension; no fitted score or gate')
fig.tight_layout();fig.savefig(O/'quality_vs_error_change.png',dpi=140);plt.close(fig)
