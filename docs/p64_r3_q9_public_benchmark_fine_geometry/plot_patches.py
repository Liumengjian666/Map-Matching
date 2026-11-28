"""Render the real IILABS shared/changed target identities in map coordinates."""
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from paired import read_pcd


def main(out):
    p=out/"visuals"
    map_=read_pcd(p/"W2_local_prior.pcd")
    scan=read_pcd(p/"W2_source_selected_world.pcd")
    target=read_pcd(p/"W2_patch_target_world.pcd")
    first=read_pcd(p/"W2_changed_targets_T0.pcd")
    second=read_pcd(p/"W2_changed_targets_Td.pcd")
    fig,axs=plt.subplots(1,2,figsize=(11,4.5))
    for ax,data,label in zip(axs,(scan,second),("Selected query and target","Changed target identity after directional NDT")):
        ax.scatter(map_[:,0],map_[:,1],s=1,c=".83",label="Independent prior")
        if ax is axs[0]:
            ax.scatter(target[:,0],target[:,1],s=4,c="tab:blue",label="Target")
            pair=target
        else:
            ax.scatter(first[:,0],first[:,1],s=4,c="tab:blue",label="T0 target")
            pair=first
        ax.scatter(data[:,0],data[:,1],s=4,c="tab:orange",label="Source" if ax is axs[0] else "Td target")
        for a,b in zip(data[::17],pair[::17]):
            ax.plot([a[0],b[0]],[a[1],b[1]],color=".35",alpha=.30,lw=.55)
        ax.axis("equal");ax.set_xlabel("World X (m)");ax.set_ylabel("World Y (m)")
        ax.set_title(label);ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(p/"W2_patch_correspondences.png",dpi=105)
    plt.close(fig)


if __name__=="__main__":
    main(Path(sys.argv[1]))
