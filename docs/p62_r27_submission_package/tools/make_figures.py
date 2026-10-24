#!/usr/bin/env python3
"""Publication schematics and a replot of two frozen memory measurements."""
from pathlib import Path
import json,hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch,FancyArrowPatch
from PIL import Image
D=Path(__file__).resolve().parents[1];F=D/'figures'
plt.rcParams.update({'font.family':'DejaVu Sans','pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none'})
navy='#19354b';green='#17624f';orange='#9d4710'
def box(ax,x,y,w,h,text,color=navy,fontsize=8):
 ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.012,rounding_size=0.025',edgecolor=color,facecolor='white',linewidth=1))
 ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontsize=fontsize,color=color)
def arrow(ax,a,b,color=navy,style='-'):
 ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=10,linewidth=1,color=color,linestyle=style))
def export(fig,name,dpi=300):
 for ext in ['pdf','svg','png']:fig.savefig(F/(name+'.'+ext),dpi=dpi,facecolor='white',bbox_inches='tight',pad_inches=.06)
 plt.close(fig)
fig,ax=plt.subplots(figsize=(7.16,2.38));ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
box(ax,.01,.62,.15,.27,'Real IMU\npropagation\n+ deskew',fontsize=8)
box(ax,.22,.62,.17,.27,'Pre-geometry\ndiagnostic (G)\nNo attempt veto',fontsize=8)
box(ax,.45,.62,.16,.27,'Native Full NDT\nNominal seed\nfrom causal state',fontsize=8)
box(ax,.67,.62,.14,.27,'Unchanged\npost-quality\nadmission',green,8)
box(ax,.86,.62,.13,.27,'Historical\nEKF update\n+ IMU replay',green,8)
for a,b in [((.16,.755),(.22,.755)),((.39,.755),(.45,.755)),((.61,.755),(.67,.755)),((.81,.755),(.86,.755))]:arrow(ax,a,b)
box(ax,.44,.12,.26,.25,'C: map-yaw ±5 deg\n≤2 extra/event; ≤12/run',orange,8)
arrow(ax,(.74,.62),(.74,.25),orange);arrow(ax,(.74,.25),(.70,.25),orange)
arrow(ax,(.44,.245),(.42,.245),orange);arrow(ax,(.42,.245),(.42,.58),orange);arrow(ax,(.42,.58),(.52,.62),orange)
ax.text(.77,.39,'failure +\nbudget',fontsize=7.6,ha='left',color=orange)
arrow(ax,(.805,.62),(.825,.03),green);arrow(ax,(.825,.03),(.07,.03),green);arrow(ax,(.07,.03),(.07,.62),green)
ax.text(.20,.04,'No admitted candidate: IMU continues; no artificial map observation',fontsize=7.2,va='bottom',color=green)
ax.plot([.305,.305],[.89,.99],ls='--',color='#a12c2c',lw=1)
ax.text(.42,.97,'Original hard veto can make registration/recovery unreachable',fontsize=7.5,color='#a12c2c',ha='center',va='bottom')
export(fig,'measurement_path')
fig,ax=plt.subplots(figsize=(6.6,2.95),dpi=100);fig.subplots_adjust(0,0,1,1);ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
ax.text(.5,.91,'Gate-aware prior-map localization',ha='center',fontsize=16,color=navy,fontweight='bold')
for x,text in [(.03,'Geometry\ndiagnostic'),(.36,'Native NDT\n+ quality'),(.69,'IMU / EKF\nmap update')]:box(ax,x,.48,.28,.27,text,green if x>.6 else navy,12)
arrow(ax,(.31,.615),(.36,.615));arrow(ax,(.64,.615),(.69,.615),green)
box(ax,.26,.17,.48,.21,'Rejected? Fixed ±5° retries\nStrict extra-call budget',orange,12)
arrow(ax,(.5,.48),(.5,.38),orange)
ax.text(.5,.06,'Development case: 60 scans · 8 extra calls · 57 map updates',ha='center',fontsize=10.5,color=navy)
for ext in ['pdf','svg']:fig.savefig(F/('gagraphic.'+ext),facecolor='white')
fig.savefig(F/'gagraphic.png',dpi=100,facecolor='white');plt.close(fig)
im=Image.open(F/'gagraphic.png').convert('RGB');assert im.size==(660,295)
im.save(F/'gagraphic.png',dpi=(300,300),optimize=True)
values=[775308,138200]
fig,ax=plt.subplots(figsize=(3.45,2.35));fig.subplots_adjust(left=.20,bottom=.21,right=.97,top=.89)
bars=ax.bar(['Global target','Local target'],[v/1024 for v in values],color=[navy,green],width=.52)
ax.set_ylim(0,880);ax.set_ylabel('NDT process HWM (MiB)',fontsize=9);ax.tick_params(labelsize=8)
ax.spines[['right','top']].set_visible(False)
for b,v in zip(bars,values):ax.text(b.get_x()+b.get_width()/2,b.get_height()+13,f'{v/1024:.1f}',ha='center',fontsize=9)
ax.text(.5,.96,'Historical matched 300-scan resource study',transform=ax.transAxes,ha='center',fontsize=7.5)
export(fig,'local_map_memory')
for path in F.glob('*.svg'):
 path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
provenance={'schematics':'measurement_path/gagraphic: new vector explanatory graphics; no experimental recomputation','memory_plot':{'global_NDT_HWM_KiB':775308,'local_NDT_HWM_KiB':138200,'source':'R25 RESOURCE_COMPARISON.md citing frozen P45 record','change':'internal stage labels replaced by global/local target, measurements unchanged'},'error_plot':'R25 ablation_errors.pdf copied byte-identically','sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(F.glob('*')) if p.is_file()}}
(D/'FIGURE_PROVENANCE.json').write_text(json.dumps(provenance,indent=2)+'\n')
print('figure files',len(provenance['sha256']),'GA bytes',(F/'gagraphic.png').stat().st_size)
