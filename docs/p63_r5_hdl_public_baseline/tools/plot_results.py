#!/usr/bin/env python3
"""Figures from sealed raw5–164 only; no new matching or trajectory alignment."""
import csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
A=Path(__file__).resolve().parents[1]
def rows(name):
    with (A/name).open() as stream:return list(csv.DictReader(stream))
hdl=rows('RESULTS.csv');old=rows('P62_MATCHED_WINDOW.csv');out=A/'figures';out.mkdir(exist_ok=True)
fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,constrained_layout=True)
for name,label,color,style in [('HDL_NATIVE_NO_IMU','HDL: no IMU','tab:green','-'),
    ('P62_NATIVE_NDT_EKF_D','P62 native NDT + IMU/EKF D','tab:blue','-'),
    ('P62_G_ONLY','P62 G-only (C+G coincides)','tab:red','-')]:
    data=hdl if name=='HDL_NATIVE_NO_IMU' else [r for r in old if r['method']==name]
    for axis,key in zip(axes,['translation_error_m','rotation_error_deg']):
        axis.plot([int(r['raw_ordinal']) for r in data],[float(r[key]) for r in data],style,color=color,label=label,linewidth=1)
for axis,threshold,ylabel in zip(axes,[.5,5],['Translation error (m)','Rotation error (deg)']):
    axis.axhline(threshold,linestyle='--',color='gray');axis.grid(alpha=.2);axis.set(ylabel=ylabel,xlim=(5,164));axis.legend(fontsize=8)
axes[-1].set_xlabel('Original raw scan ordinal')
fig.suptitle('Matched window; map-anchored GICP+IMU reference, no posthoc alignment\nSystem comparison: preprocessing, filter and IMU use differ')
fig.savefig(out/'matched_window_errors.png',dpi=180);plt.close(fig)
fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True,constrained_layout=True)
x=[int(r['raw_ordinal']) for r in hdl]
for prefix,label in [('pred_','Prediction'),('ndt_','Raw NDT'),('','UKF output')]:
    axes[0].plot(x,[float(r[prefix+'translation_error_m']) for r in hdl],label=label,linewidth=1)
    axes[1].plot(x,[float(r[prefix+'rotation_error_deg']) for r in hdl],label=label,linewidth=1)
axes[0].axhline(.5,color='gray',linestyle='--');axes[1].axhline(5,color='gray',linestyle='--')
axes[0].set_ylabel('Translation error (m)');axes[1].set_ylabel('Rotation error (deg)')
axes[2].plot(x,[float(r['inlier_fraction']) for r in hdl],color='tab:purple',label='Native inlier (map-origin 25m crop)')
axes[2].set(ylabel='Native inlier fraction',ylim=(0,1.03),xlabel='Original raw scan ordinal')
for axis in axes:
    axis.axvspan(54,58,color='tab:red',alpha=.1);axis.axvspan(78,126,color='tab:red',alpha=.1)
    axis.grid(alpha=.2);axis.legend(fontsize=8);axis.set_xlim(5,164)
fig.suptitle('HDL frozen causal outputs: reference error and original quality are distinct\nShading is post-seal reference-error analysis, not online gating')
fig.savefig(out/'hdl_components_and_quality.png',dpi=180);plt.close(fig)
print('Saved matched full-window and native prediction/NDT/UKF figures.')
