"""Non-GT minimal reproduction of independent anchor-carrier audit differences."""
import numpy as np
from scipy.spatial.transform import Rotation
from run_budgeted import read
from run_anchored import ARCHIVE,MODES
from evaluate_budgeted import matrix
from evaluate_anchored import chart

for mode in MODES:
    d=ARCHIVE/"attempt_0"/mode
    for f,a,r in zip(read(d/"frames.csv"),read(d/"admission.csv"),read(d/"anchor.csv")):
        if r["evaluated"]!="1":continue
        W=np.array([float(v) for v in r["weak_basis"].split(";")]).reshape(6,2)[:,:int(r["weak_dimension"])]
        nom=matrix(f["nominal_pose"]);alt=matrix(a["candidate_pose"]);anchor=matrix(r["anchor_prediction"])
        cn=float(np.linalg.norm(W.T@chart(nom,anchor))**2);ca=float(np.linalg.norm(W.T@chart(alt,anchor))**2)
        diff=chart(alt,nom);wf=min(1.,float(np.linalg.norm(W.T@diff)**2/np.dot(diff,diff)))
        errors=[abs(cn-float(r["nominal_weak_cost"])),abs(ca-float(r["alternative_weak_cost"])),abs(wf-float(r["weak_fraction"]))]
        if max(errors)>1e-8:print(mode,f["transaction_id"],"nom/alt/fraction errors",errors)
