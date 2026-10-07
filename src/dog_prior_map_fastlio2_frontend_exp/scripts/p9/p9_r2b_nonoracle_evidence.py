#!/usr/bin/env python3
"""Blind terminal evidence construction. No oracle, labels, GT or optimizer imports."""
import argparse
import csv
import ctypes
import hashlib
import itertools
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
OUT=ROOT/"docs/p9_r2b_nonoracle_evidence"
R2A=ROOT/"docs/p9_r2a_predictor_conditioned_search"
START_SHA="2712573a8d3be1768ba59f6aa92f184d0949d19f"
BRANCH="research/p9-r2b-nonoracle-evidence"
BUDGETS=(4,8,12,16)
EPS_SCORE=2.747604276e-4
LIBRARY=Path("/tmp/p9_r2_build/libp9_r2b_chart.so")


def require(condition,message):
    if not condition:raise RuntimeError(message)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:return list(csv.DictReader(stream))


def write_csv(path,rows,fields=None):
    fields=list(fields or rows[0].keys())
    with Path(path).open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fields,lineterminator="\n")
        writer.writeheader();writer.writerows(rows)


def vector(value,size):
    result=np.fromstring(value,sep=";")
    require(result.size==size and np.isfinite(result).all(),"invalid numeric vector")
    return result


def text(value):
    return ";".join(format(float(x),".17g") for x in np.asarray(value).ravel())


def frozen_hash(path):
    require(ROOT in path.parents,"input outside committed paper archive")
    value=subprocess.check_output(["git","show",START_SHA+":"+path.relative_to(ROOT).as_posix()],cwd=ROOT)
    h=hashlib.sha256(value).hexdigest()
    require(digest(path)==h,"committed input changed: "+str(path))
    return h


def chart_function(library):
    function=ctypes.CDLL(str(library)).p9_r2b_displacement
    pointer=np.ctypeslib.ndpointer(dtype=np.float64,flags="C_CONTIGUOUS")
    function.argtypes=[pointer,pointer,pointer];function.restype=None
    def evaluate(from16,to16):
        result=np.empty(6)
        function(np.ascontiguousarray(from16,dtype=float),np.ascontiguousarray(to16,dtype=float),result)
        require(np.isfinite(result).all(),"nonfinite P9 chart result")
        return result
    return evaluate


def entry(rank,matrix,score,xyz_quat=None,iterations=0):
    m=vector(matrix,16).reshape(4,4)
    require(np.allclose(m[3],[0,0,0,1],atol=1e-12,rtol=0) and np.isfinite(score),"invalid terminal")
    if xyz_quat is None:pose=(m[:3,3],Rotation.from_matrix(m[:3,:3]))
    else:
        v=vector(xyz_quat,7);require(np.linalg.norm(v[3:])>1e-12,"zero quaternion")
        pose=(v[:3],Rotation.from_quat(v[3:]))
        require(np.linalg.norm(v[:3]-m[:3,3])<=1e-8,"terminal matrix/pose translation mismatch")
    return dict(rank=rank,matrix=matrix,score=float(score),pose=pose,iterations=iterations)


def separation(a,b):
    return float(np.linalg.norm(a[0]-b[0])),float(np.degrees((a[1].inv()*b[1]).magnitude()))


def complete_link(entries):
    n=len(entries);distances=np.zeros((n,n));groups=[(i,) for i in range(n)]
    for i,j in itertools.combinations(range(n),2):
        dt,dr=separation(entries[i]["pose"],entries[j]["pose"])
        distances[i,j]=distances[j,i]=max(dt/.2,dr/2.)
    while len(groups)>1:
        options=[(float(distances[np.ix_(a,b)].max()),a,b,i,j)
                 for i,a in enumerate(groups) for j,b in enumerate(groups) if i<j]
        distance,a,b,i,j=min(options)
        if distance>1.:break
        groups=[g for k,g in enumerate(groups) if k not in (i,j)]+[tuple(sorted(a+b))]
        groups.sort()
    return groups


def evidence_for_frame(frame,budget,nominal,rows,chart):
    prefix=rows[:budget]
    source_count=int(prefix[0]["source_points"])
    points=[entry(0,nominal,float(prefix[0]["nominal_ndt_score_sum"]))]
    for row in prefix:
        if row["converged"]=="1":points.append(entry(int(row["probe_rank"]),row["terminal_pose_matrix16"],
            float(row["raw_ndt_score_sum"]),row["terminal_pose_xyz_q_xyzw"],int(row["iterations"])))
    clusters=[];competitors=[]
    for index,members in enumerate(complete_link(points)):
        is_nominal=0 in members
        representative=max((points[i] for i in members),key=lambda p:(p["score"],-p["rank"]))
        competitive=(not is_nominal) and representative["score"]>=points[0]["score"]+EPS_SCORE
        dt,dr=separation(points[0]["pose"],representative["pose"])
        row=dict(frame=frame,budget=budget,cluster_id="NOMINAL" if is_nominal else f"C{index:03d}",
            is_nominal=int(is_nominal),members_probe_ranks=";".join(str(points[i]["rank"]) for i in members),
            member_count=len(members),representative_rank=representative["rank"],
            representative_pose_matrix16=representative["matrix"],raw_score=representative["score"],
            energy=-representative["score"],nominal_raw_score=points[0]["score"],
            score_advantage=representative["score"]-points[0]["score"],competitive=int(competitive),
            translation_from_nominal_m=dt,rotation_from_nominal_deg=dr,
            representative_outside_nominal_ball=int(dt>.2 or dr>2.),
            iteration_limit_members=sum(points[i]["iterations"]>=80 for i in members),
            representative_iteration_limit=int(representative["iterations"]>=80))
        clusters.append(row)
        if competitive:
            xi=chart(vector(nominal,16),vector(representative["matrix"],16))
            competitors.append(dict(row,xi=text(xi),xi_norm=float(np.linalg.norm(xi)),
                                    score_advantage_per_source=row["score_advantage"]/source_count))
    a=np.zeros((6,6));norms=[]
    for row in competitors:
        xi=vector(row["xi"],6);a+=np.outer(xi,xi);norms.append(row["xi_norm"])
    eigenvalues,eigenvectors=np.linalg.eigh(a)
    require(eigenvalues.min()>=-1e-10*max(1.,float(np.trace(a))),"evidence tensor not PSD")
    principal=eigenvectors[:,-1] if competitors else np.zeros(6)
    if principal[np.argmax(np.abs(principal))]<0:principal=-principal
    evidence=dict(frame=frame,budget=budget,source_points=source_count,cluster_count=len(clusters),
        competitive_cluster_count=len(competitors),converged_probes=len(points)-1,
        nonconverged_probes=budget-(len(points)-1),iteration_limit_probes=sum(int(r["iterations"])>=80 for r in prefix),
        U_comp=float(np.sqrt(max(0.,eigenvalues[-1]))),trace=float(np.trace(a)),
        max_displacement=max(norms,default=0.),median_displacement=float(np.median(norms)) if norms else 0.,
        best_score_advantage_per_source=max((r["score_advantage_per_source"] for r in competitors),default=0.),
        competitive_representatives_inside_nominal_ball=sum(not r["representative_outside_nominal_ball"] for r in competitors),
        A_comp_rowmajor=text(a),eigenvalues=text(eigenvalues),principal_direction=text(principal),
        principal_direction_resolved=int(bool(competitors) and eigenvalues[-1]-eigenvalues[-2]>1e-10*max(1.,eigenvalues[-1])))
    return clusters,competitors,evidence


def self_test(chart):
    origin=np.eye(4);origin[:3,:3]=Rotation.from_euler("xyz",[.3,.4,-.6]).as_matrix()
    target=origin.copy();target[0,3]=.8
    target[:3,:3]=Rotation.from_rotvec([0,0,.2]).as_matrix()@origin[:3,:3]
    require(np.linalg.norm(chart(origin.ravel(),target.ravel())-np.array([1,0,0,0,0,.2]))<3e-7,
            "library must use spatial map product chart, not body log")
    matrix=lambda x:text(np.array([[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]],dtype=float))
    points=[entry(0,matrix(0),100),entry(1,matrix(.1),100),entry(2,matrix(.25),101)]
    require(len(complete_link(points))==2,"complete-link improperly chains neighbors")
    rows=[dict(probe_rank=str(i+1),source_points="1400",nominal_ndt_score_sum="100",converged="1",
          terminal_pose_matrix16=matrix(x),terminal_pose_xyz_q_xyzw=f"{x};0;0;0;0;0;1",
          raw_ndt_score_sum=str(score),iterations="80") for i,(x,score) in enumerate(((0.,100.),(1.,101.),(-1.,102.),(2.,99.)))]
    clusters,competitors,evidence=evidence_for_frame(1,4,matrix(0),rows,chart)
    require(len(competitors)==2 and evidence["iteration_limit_probes"]==4,"competitive sign/iteration-limit regression")
    require(abs(evidence["trace"]-2/.8**2)<1e-6 and abs(evidence["U_comp"]-np.sqrt(2)/.8)<1e-6,
            "tensor incorrectly normalized by budget or competitor count")
    for row in rows:row["raw_ndt_score_sum"]="100.0001"
    require(evidence_for_frame(1,4,matrix(0),rows,chart)[2]["U_comp"]==0.,"numerical score tie manufactured evidence")
    rows[1]["raw_ndt_score_sum"]="101";rows[1]["converged"]="0"
    require(not evidence_for_frame(1,4,matrix(0),rows,chart)[1],"nonconverged competitor admitted")
    for i,x in enumerate((0.,-.15,.15,0.)):
        rows[i].update(converged="1",terminal_pose_matrix16=matrix(x),
                       terminal_pose_xyz_q_xyzw=f"{x};0;0;0;0;0;1",raw_ndt_score_sum="101")
    boundary=evidence_for_frame(1,4,matrix(0),rows,chart)[2]
    require(boundary["competitive_representatives_inside_nominal_ball"]==1,
            "complete-link center-ball boundary case silently filtered")
    print("P9_R2B_NONORACLE_EVIDENCE_SELF_TEST=PASS")


def input_groups():
    # The sole data-reader allowlist for the blind stage; no evaluation module is imported.
    inputs=[R2A/name for name in ("ndt_runs.csv","predictor_parity.csv","score_carrier_sensitivity.csv")]
    hashes={str(path):frozen_hash(path) for path in inputs}
    all_runs=read_csv(inputs[0]);nominal_rows=read_csv(inputs[1])
    require(len(all_runs)==2560 and len(nominal_rows)==32,"frozen R2A input counts invalid")
    nominals={int(row["frame"]):row["nominal_pose_matrix16"] for row in nominal_rows}
    require(len(nominals)==32 and all(row["pass"]=="1" for row in nominal_rows),"nominal/predictor records invalid")
    groups={}
    for row in all_runs:
        if row["method"]=="COND_WEAK2":
            require(row["random_rep"]=="-1","unexpected WEAK replicate")
            groups.setdefault(int(row["frame"]),[]).append(row)
    require(set(groups)==set(nominals),"frame coverage mismatch")
    for rows in groups.values():
        rows.sort(key=lambda row:int(row["probe_rank"]))
        require(len(rows)==16 and [int(row["probe_rank"]) for row in rows]==list(range(1,17)),"incomplete WEAK prefix")
        require(all(row["converged"] in ("0","1") and 0<=int(row["iterations"])<=80 and
                    int(row["source_points"])>0 for row in rows),"invalid terminal status")
        require(len({row["nominal_ndt_score_sum"] for row in rows})==1 and
                len({row["source_points"] for row in rows})==1 and len({row["source_hash"] for row in rows})==1,
                "per-frame nominal score/source changed")
        for row in rows:
            entry(int(row["probe_rank"]),row["terminal_pose_matrix16"],float(row["raw_ndt_score_sum"]),
                  row["terminal_pose_xyz_q_xyzw"],int(row["iterations"]))
    carrier=read_csv(inputs[2])
    carrier_max=max(abs(float(row["score_gap"])) for row in carrier)
    require(carrier_max<=EPS_SCORE,"score guard smaller than recorded carrier gap")
    return nominals,groups,hashes,carrier_max


def construct(library):
    branch=subprocess.check_output(["git","branch","--show-current"],cwd=ROOT,text=True).strip()
    require(branch==BRANCH,"incorrect R2B branch")
    subprocess.run(["git","merge-base","--is-ancestor",START_SHA,"HEAD"],cwd=ROOT,check=True)
    OUT.mkdir(parents=True,exist_ok=True)
    require(not (OUT/"evidence_freeze.json").exists(),"evidence already frozen; refusing recomputation")
    chart=chart_function(library);self_test(chart)
    nominals,groups,hashes,carrier_max=input_groups()
    clusters=[];competitors=[];evidence=[]
    for frame in sorted(groups):
        for budget in BUDGETS:
            c,k,e=evidence_for_frame(frame,budget,nominals[frame],groups[frame],chart)
            clusters.extend(c);competitors.extend(k);evidence.append(e)
    write_csv(OUT/"terminal_clusters.csv",clusters)
    write_csv(OUT/"competitive_terminals.csv",competitors,
              list(clusters[0])+["xi","xi_norm","score_advantage_per_source"])
    write_csv(OUT/"nonoracle_evidence.csv",evidence)
    code=[Path(__file__),HERE/"p9_r2b_chart.cpp",HERE/"CMakeLists.txt",OUT/"THEORY.md"]
    freeze=dict(task="PAPER-P9-R2B-NONORACLE-COMPETING-TERMINAL-EVIDENCE-GATE",state="EVIDENCE_FROZEN_BEFORE_LABELS",
        git=dict(branch=branch,start_sha=START_SHA,construction_head=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()),
        EPS_SCORE=EPS_SCORE,source_carrier_max_gap=carrier_max,budgets=list(BUDGETS),frame_count=32,accepted_method="COND_WEAK2",
        new_ndt_calls=0,labels_loaded=False,canonical_inputs_loaded=False,gt_used_for_evidence=False,
        input_read_allowlist=list(hashes),input_sha256=hashes,construction_source_sha256={str(p):digest(p) for p in code},
        chart_library=str(library),chart_library_sha256=digest(library),
        frozen_artifact_sha256={name:digest(OUT/name) for name in ("terminal_clusters.csv","competitive_terminals.csv","nonoracle_evidence.csv")},
        rows=dict(clusters=len(clusters),competitive_terminals=len(competitors),evidence=len(evidence)))
    (OUT/"evidence_freeze.json").write_text(json.dumps(freeze,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print(json.dumps(dict(EVIDENCE_FROZEN=True,nonoracle_evidence_sha256=freeze["frozen_artifact_sha256"]["nonoracle_evidence.csv"],
        rows=freeze["rows"],NEW_NDT_CALLS=0,LABELS_LOADED=False,GT_USED_FOR_EVIDENCE=False),indent=2))


def audit_frozen():
    freeze=json.loads((OUT/"evidence_freeze.json").read_text())
    require(freeze["state"]=="EVIDENCE_FROZEN_BEFORE_LABELS" and not freeze["labels_loaded"] and
            not freeze["gt_used_for_evidence"] and not freeze["canonical_inputs_loaded"],"blind freeze contract invalid")
    for paths in (freeze["input_sha256"],freeze["construction_source_sha256"]):
        for name,h in paths.items():require(digest(name)==h,"frozen construction input changed: "+name)
    require(digest(freeze["chart_library"])==freeze["chart_library_sha256"],"chart library changed")
    for name,h in freeze["frozen_artifact_sha256"].items():require(digest(OUT/name)==h,"frozen evidence changed: "+name)
    nominals,groups,_,_=input_groups();chart=chart_function(Path(freeze["chart_library"]))
    lookup={(int(r["frame"]),int(r["budget"])):r for r in read_csv(OUT/"nonoracle_evidence.csv")}
    actual_clusters=read_csv(OUT/"terminal_clusters.csv");actual_competitors=read_csv(OUT/"competitive_terminals.csv")
    require(len(lookup)==128 and len(actual_clusters)==freeze["rows"]["clusters"] and
            len(actual_competitors)==freeze["rows"]["competitive_terminals"],"freeze denominator mismatch")
    for frame in sorted(groups):
        for budget in BUDGETS:
            c,k,e=evidence_for_frame(frame,budget,nominals[frame],groups[frame],chart)
            expected=lookup[frame,budget]
            require(all(str(value)==expected[key] for key,value in e.items()),"evidence/CSV recomputation mismatch")
            select=lambda rows:[r for r in rows if int(r["frame"])==frame and int(r["budget"])==budget]
            for rows,expected_rows in ((c,select(actual_clusters)),(k,select(actual_competitors))):
                require([{key:str(value) for key,value in row.items()} for row in rows]==expected_rows,
                        "cluster/competitor recomputation mismatch")
    return freeze


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("self-test","construct","audit"))
    parser.add_argument("--library",type=Path,default=LIBRARY);args=parser.parse_args()
    if args.stage=="self-test":self_test(chart_function(args.library))
    elif args.stage=="construct":construct(args.library.resolve())
    else:
        audit_frozen();print("P9_R2B_BLIND_EVIDENCE_HASH_AUDIT=PASS NEW_NDT_CALLS=0")
