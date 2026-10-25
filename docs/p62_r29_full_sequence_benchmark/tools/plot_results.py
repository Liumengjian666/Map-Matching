#!/usr/bin/env python3
from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parents[1];anchor=json.loads((P/'input/DATASETS.json').read_text())['datasets']['EASY']['anchor_stamp_ns'];out=P/'figures';out.mkdir(exist_ok=True)
labels={'A':'A: original pre-gate','B':'B: G-only','C':'C: C+G (same poses as B)','D':'D: native, no pre-gate'}
colors={'A':'#bf5b17','B':'#7b3294','C':'#0082b2','D':'#238b45'}
fig,axes=plt.subplots(2,1,figsize=(10,6.5),sharex=True)
fig2,ax=plt.subplots(figsize=(10,3))
for i,m in enumerate('ABCD'):
 r=list(csv.DictReader((P/'FULL_SEQUENCE_TRAJECTORIES'/('EASY_C0_'+m+'.csv')).open()));x=np.array([(int(a['state_stamp_ns'])-anchor)/1e9 for a in r]);t=np.array([float(a['translation_error_m']) if a['translation_error_m'] else np.nan for a in r]);rot=np.array([float(a['rotation_error_deg']) if a['rotation_error_deg'] else np.nan for a in r])
 for axis,y in zip(axes,[t,rot]):axis.plot(x,y,label=labels[m],color=colors[m],linestyle='--' if m=='B' else '-',linewidth=1.5,alpha=.9)
 accepted=[j for j,a in enumerate(r) if a['ekf_update_applied']=='True'];ax.scatter(x[accepted],[i]*len(accepted),s=3,color=colors[m])
axes[0].set_yscale('log');axes[0].set_ylabel('Position error (m, log scale)');axes[1].set_ylabel('Rotation error (degrees)');axes[1].set_xlabel('Sensor time after initialization anchor (s)');axes[0].legend(fontsize=8,ncol=2)
axes[0].axhline(.5,color='grey',linewidth=.8,linestyle=':');axes[1].axhline(5,color='grey',linewidth=.8,linestyle=':')
for a in axes:a.grid(True,alpha=.2)
fig.suptitle('Koide indoor_easy_01: complete admitted span, C0\n2023 estimated frames; 2022 valid benchmark references; no trajectory alignment',fontsize=11);fig.tight_layout()
for suffix in ['png','pdf']:fig.savefig(out/('FULL_SEQUENCE_ERRORS.'+suffix),dpi=160,bbox_inches='tight')
ax.set_yticks(range(4));ax.set_yticklabels([labels[m] for m in 'ABCD']);ax.set_xlabel('Sensor time after initialization anchor (s)');ax.set_title('Actual EKF/OOSM map updates; no multistart candidate accepted');ax.grid(True,alpha=.2);fig2.tight_layout()
for suffix in ['png','pdf']:fig2.savefig(out/('MAP_UPDATE_TIMELINE.'+suffix),dpi=160,bbox_inches='tight')
