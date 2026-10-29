"""Plot sealed global ranking metrics; no decisions or new thresholds."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parents[1];out=P/'figures';out.mkdir(exist_ok=True)
r=json.loads((P/'RESULTS.json').read_text());m=r['global_metrics'];names=[x['feature'] for x in m];y=np.arange(len(m))
fig,ax=plt.subplots(1,2,figsize=(13,6),sharey=True)
colors=['#117a65' if x['passes_exploratory_criteria'] else '#6b7c93' for x in m]
ax[0].barh(y,[x['harm_vs_benefit_auroc'] for x in m],color=colors)
ax[0].axvline(.75,color='#a93226',ls='--',lw=1,label='Frozen AUROC criterion')
ax[0].set_xlim(0,1);ax[0].set_yticks(y);ax[0].set_yticklabels(names);ax[0].invert_yaxis()
ax[0].set_xlabel('AUROC: 20 harmful vs 24 beneficial corrections');ax[0].legend(loc='lower right',fontsize=8)
for i,x in enumerate(m):ax[0].text(x['harm_vs_benefit_auroc']+.012,i,f"{x['harm_vs_benefit_auroc']:.3f}",va='center',fontsize=9)
ax[1].barh(y-.18,[100*x['top_benefit_fraction'] for x in m],height=.35,label='Beneficial / Top10% set',color='#5dade2')
ax[1].barh(y+.18,[100*x['top_benefit_n']/x['material_benefit_n'] for x in m],height=.35,label='Top10% beneficial / All beneficial',color='#e67e22')
ax[1].set_xlim(0,105);ax[1].set_xlabel('Percent (different denominators)');ax[1].legend(loc='lower right',fontsize=8)
fig.suptitle('R13: frozen global ranking support does not establish safe rejection',fontsize=13)
fig.text(.5,.015,'2162 valid-reference frames; pre-loss group has only 1 harmful / 2 beneficial. No online feedback.',ha='center',fontsize=10)
fig.tight_layout(rect=[0,.04,1,.94])
fig.savefig(out/'ranking_and_beneficial_confusion.png',dpi=160)
fig.savefig(out/'ranking_and_beneficial_confusion.pdf')
plt.close(fig)
print('Saved sealed-metric figure (PNG/PDF); no matching or re-ranking')
