"""Audit the natural failure without changing or replaying any state updates."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from reference_checks import interpolate, poses
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q10_public_mechanism_validation"))
from evaluate_exp18 import field, matrix, pcd, read_pcd, read_reference, rows, transform, voxel, write


def run(input_, target, baseline, diagnostic, dense, out):
    out.mkdir()
    g_time,g_pose=read_reference(dense)
    reference=poses(g_pose)
    ili=np.array(json.loads((input_/"input_summary.json").read_text())["T_I_L"])
    log=rows(baseline/"trajectory.csv")
    base=rows(diagnostic/"frame_diagnostics.csv")
    tick=np.array([int(x["stamp_ns"]) for x in log])
    t=(tick-tick[0])*1e-9
    e=np.array([float(x["absolute_m"]) for x in base])
    nonempty=np.array([int(x["supported"])>0 for x in log])
    flags=np.array([x["native_weak"]=="1" for x in base])
    first_bad=np.flatnonzero(e>1.0)[0]
    final_loss=np.flatnonzero(nonempty)[-1]+1
    earlier_weak=np.flatnonzero(flags & nonempty)
    first_weak=earlier_weak[0]
    map_=read_pcd(target)
    nn=cKDTree(map_)
    centers,mats=field(map_)
    fi=cKDTree(centers)
    sequence=list(dict.fromkeys((first_bad-1,first_bad,first_weak,final_loss-1,final_loss)))
    facts=[]
    for frame in sequence:
        datum=log[frame]
        ref=interpolate(g_time,reference,tick[frame])
        p=read_pcd(input_/f"scan_{frame:04d}.pcd")
        p=voxel(p,.10)
        cutoff=np.linalg.norm(p,axis=1)<=10
        local=p[cutoff]
        s=matrix(datum,"state")
        est=transform(local,s)
        frame_error=dict(frame=frame,time_since_anchor_s=t[frame],native_weak=int(flags[frame]),
            ndt_support=int(datum["supported"]),source=int(datum["source"]),ndt_iterations=int(datum["iterations"]),
            ndt_converged=int(datum["converged"]),absolute_deviation_m=e[frame])
        dr=nn.query(est)[0]
        frame_error.update(local_query_n=len(local),estimate_nn_median_m=np.median(dr),
                           estimate_nn_within_0p30=np.mean(dr<=.30))
        sw=np.array([float(base[frame][f"axis_{axis}"]) for axis in "xyz"])
        if ref is not None:
            expected=transform(local,ref@ili)
            dg=nn.query(expected)[0]
            here=fi.query_ball_point(ref[:3,3],8.)
            val,U=np.linalg.eigh(mats[here].sum(0))
            physical_axis=U[:,0]
            physical_axis*=np.sign(physical_axis[np.argmax(abs(physical_axis))])
            frame_error.update(dense_ref_nn_median_m=np.median(dg),dense_ref_nn_within_0p30=np.mean(dg<=.30),
                dense_location_weak_fraction=val[0]/val.sum(),
                dense_location_weak_mass=val[0]/len(here),dense_location_weak_axis=";".join(map(str,physical_axis)),
                abs_weak_axis_dot=abs(physical_axis@sw) if np.isfinite(sw).all() else "")
            if frame in (first_bad,first_weak):
                view=map_[(abs(map_-ref[:3,3])<[10,10,5]).all(1)]
                view=voxel(view,.20)
                pcd(out/f"event_{frame}_map.pcd",view)
                pcd(out/f"event_{frame}_estimate_scan.pcd",est)
                pcd(out/f"event_{frame}_dense_ref_scan.pcd",expected)
                fig,axes=plt.subplots(1,2,figsize=(11,5))
                for ax,i,label in zip(axes,(1,2),("Y","Z")):
                    ax.scatter(view[::3,0],view[::3,i],s=1,c=".75",label="Independent E57")
                    ax.scatter(est[:,0],est[:,i],s=3,c="tab:red",label="Native UKF")
                    ax.scatter(expected[:,0],expected[:,i],s=3,c="tab:blue",label="Published dense reference")
                    ax.quiver(*ref[:3,3][[0,i]],*physical_axis[[0,i]],angles="xy",
                              scale_units="xy",scale=.5,color="black",label="Map weak axis")
                    ax.set_aspect("equal")
                    ax.set(xlabel="Map X (m)",ylabel=f"Map {label} (m)")
                axes[0].legend(fontsize=7)
                fig.tight_layout()
                fig.savefig(out/f"event_{frame}_overlay.png",dpi=120)
                plt.close(fig)
        facts.append(frame_error)
    write(out/"EVENT_GEOMETRIC_AUDIT.csv",facts)
    figure,plots=plt.subplots(3,1,figsize=(11,8),sharex=True)
    plots[0].semilogy(t,e,c="tab:red",label="Fixed-map dense position error")
    plots[0].set_ylabel("Error (m, log scale)")
    plots[1].plot(t,[float(x["supported"])/int(x["source"]) for x in log],label="Native DIRECT7 associations")
    plots[1].set_ylabel("Associated fraction")
    plots[2].plot(t,[float(x["physical_fraction"]) for x in base],label="Physical low-direction fraction")
    plots[2].set_ylabel("Physical ratio")
    plots[2].set_xlabel("Elapsed time (s)")
    for graph in plots:
        for marker,label in ((first_bad,"First >1 m"),(first_weak,"First supported weak flag"),(final_loss,"No associations thereafter")):
            graph.axvline(t[marker],linestyle="--",label=label,linewidth=.8)
        graph.set_xlim(0,t[-1])
        graph.legend(fontsize=7,ncol=2)
    figure.tight_layout()
    figure.savefig(out/"full_sequence_failure_timeline.png",dpi=115)
    aux=dict(first_reference_error_above_1m=int(first_bad),supported_weak_onset=int(first_weak),
             persistent_loss_first_frame=int(final_loss),zero_association_frames=int((~nonempty).sum()),
             weak_with_associations=int((flags & nonempty).sum()),weak_without_associations=int((flags & ~nonempty).sum()),
             observed_persistent_loss_lower_bound_s=float(t[-1]-t[final_loss]))
    (out/"failure_summary.json").write_text(json.dumps(aux,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for name in ("input","target","baseline","diagnostic","dense","output"):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    run(a.input,a.target,a.baseline,a.diagnostic,a.dense,a.output)
