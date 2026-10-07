#!/usr/bin/env python3
"""Post-run archive-ID evaluation. No NDT optimizer or proposal mutation."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation
from run_r2a_predictor_search import (ARCHIVE, HERE, H2, ROOT, OUT, START_SHA, FRAME_ORDER, METHODS,
                                     BUDGETS, digest, read_csv, write_csv, require, vector, text,
                                     verify_manifest, git_state)

CANONICAL = ROOT / "docs/p9_r1a_true_profile_closure/oracle_terminal_requests.csv"
H1 = ROOT / "docs/p9_foundation_weak_discovery/h1"
MAJOR = (368,616,2226,2350,2722,2846,3341,3796,3962)
MISSES = ((2226,"P05"),(2350,"P01"),(2350,"P05"),(2722,"P05"),(3341,"P02"),(3341,"P03"))
PREFIX = np.arange(1,17)


def pose(value):
    v=vector(value,len(value.split(";")))
    if v.size==7:
        return v[:3],Rotation.from_quat(v[3:])
    require(v.size==16,"invalid pose vector size")
    m=v.reshape(4,4)
    return m[:3,3],Rotation.from_matrix(m[:3,:3])


def separation(a,b):
    pa,ra=a;pb,rb=b
    return float(np.linalg.norm(pa-pb)),float(np.degrees((ra.inv()*rb).magnitude()))


def admitted(row,targets):
    eligible=[]
    if row["converged"]=="1":
        terminal=pose(row["terminal_pose_xyz_q_xyzw"])
        for cluster,canonical in targets[int(row["frame"])]:
            dt,dr=separation(terminal,canonical)
            if dt<=.2 and dr<=2.:
                eligible.append(((dt/.2)**2+(dr/2.)**2,cluster,dt,dr))
    eligible.sort()
    return eligible[0][1] if eligible else "NONE",eligible


def auc(recall):
    return float(np.trapz(np.asarray(recall),np.log2(PREFIX))/np.log2(16))


def choose_objective(rows,nominal_score):
    # PCL exposes score S; the fixed optimization energy is E=-S.
    chosen=min(rows,key=lambda r:(-float(r["raw_ndt_score_sum"]),int(r["probe_rank"])))
    return chosen,float(chosen["raw_ndt_score_sum"])>nominal_score


def complete_link(poses):
    # Complete-link admission requires every member-pair to satisfy both pose cutoffs.
    n=len(poses);d=np.zeros((n,n))
    for i,j in itertools.combinations(range(n),2):
        dt,dr=separation(poses[i],poses[j]);d[i,j]=d[j,i]=max(dt/.2,dr/2.)
    groups=[(i,) for i in range(n)]
    while len(groups)>1:
        options=[(float(d[np.ix_(a,b)].max()),a,b,i,j)
                 for i,a in enumerate(groups) for j,b in enumerate(groups) if i<j]
        distance,a,b,i,j=min(options)
        if distance>1.:break
        groups=[g for k,g in enumerate(groups) if k not in (i,j)]+[tuple(sorted(a+b))]
        groups.sort()
    return len(groups)


def committed_inputs(paths):
    hashes={}
    for path in paths:
        require(path.is_file(),"missing evaluation archive: "+str(path))
        if ROOT in path.parents:
            frozen=subprocess.check_output(["git","show",START_SHA+":"+str(path.relative_to(ROOT))],cwd=ROOT)
            require(digest(path)==hashlib.sha256(frozen).hexdigest(),"committed evaluation input changed: "+str(path))
        hashes[str(path)]=digest(path)
    return hashes


def historical(targets):
    all_rows=read_csv(H2/"ndt_runs.csv")
    relevant=[r for r in all_rows if r["method"] in ("FULL6D","WEAK2")]
    require(len(relevant)==9*263*2,"H2 reference pools incomplete")
    archived={(int(r["transaction_id"]),int(r["seed_index"])):r for r in read_csv(ARCHIVE/"candidates.csv")
              if int(r["transaction_id"]) in MAJOR and int(r["seed_index"])<263}
    old_admission={(int(r["frame"]),r["method"],int(r["seed_index"])):r
                   for r in read_csv(H2/"terminal_admission.csv") if r["method"] in ("FULL6D","WEAK2")}
    reference={};hits={}
    for row in relevant:
        key=(int(row["frame"]),row["method"],int(row["seed_index"]))
        require(key not in reference,"duplicate historical result")
        reference[key]=row;winner,eligible=admitted(row,targets)
        require(winner==old_admission[key]["recovered_cluster"] and
                len(eligible)==int(old_admission[key]["ambiguous_count"]),"frozen H2 admission parity failed")
        if row["method"]=="FULL6D":
            candidate=archived[key[0],key[2]]
            require(row["terminal_pose_matrix16"]==candidate["final_pose_matrix16"] and
                    row["converged"]==candidate["converged"] and row["iterations"]==candidate["iterations"],
                    "FULL6D archived terminal/status parity failed")
        hits[key]=winner
    permutation_rows=read_csv(H2/"permutation_manifest.csv")
    require(len(permutation_rows)==100,"H2 permutation manifest incomplete")
    permutations=[]
    rng=np.random.Generator(np.random.PCG64(20261009))
    for i,row in enumerate(permutation_rows):
        order=vector(row["seed_indices"],263).astype(int)
        require(int(row["permutation"])==i and sorted(order)==list(range(263)) and
                np.array_equal(order,rng.permutation(263)),"frozen matched permutations changed")
        permutations.append(order)
    curves={};summary_rows=[]
    for method in ("FULL6D","WEAK2"):
        cube=np.zeros((100,9,16))
        for f,tx in enumerate(MAJOR):
            for r,order in enumerate(permutations):
                found=set()
                for i,seed in enumerate(order[:16]):
                    cluster=hits[tx,method,int(seed)]
                    if cluster!="NONE":found.add(cluster)
                    cube[r,f,i]=len(found)/len(targets[tx])
        curves[method]=cube
        counts=np.array([len(targets[tx]) for tx in MAJOR])
        for b in range(1,17):
            macro=cube[:,:,b-1].mean(axis=1);micro=(cube[:,:,b-1]*counts).sum(axis=1)/22
            summary_rows.append(dict(method=method,budget=b,macro_median=float(np.median(macro)),
                macro_p05=float(np.percentile(macro,5)),macro_p95=float(np.percentile(macro,95)),
                micro_median=float(np.median(micro))))
    write_csv(OUT/"historical_reference.csv",summary_rows)
    summary={m:dict(auc_common_1_16=float(np.median(np.trapz(c,np.log2(PREFIX),axis=2)/4.,axis=0).mean()),
                   macro_b16_median=float(np.median(c[:,:,-1].mean(axis=1)))) for m,c in curves.items()}
    previous=json.loads((H2/"results.json").read_text())
    summary["auc_original_1_263"]=previous["auc"]
    summary["matched_recall_cost70_FULL6D"] = previous["matched_recall_cost"]["FULL6D"]["0.7"]
    exact_costs=[]
    for order in permutations:
        found={tx:set() for tx in MAJOR}
        reached=None
        for b,seed in enumerate(order,1):
            for tx in MAJOR:
                hit=hits[tx,"FULL6D",int(seed)]
                if hit!="NONE":found[tx].add(hit)
            if np.mean([len(found[tx])/len(targets[tx]) for tx in MAJOR])>=.70:
                reached=b;break
        require(reached is not None,"FULL6D unexpectedly fails70% at full pool")
        exact_costs.append(reached)
    write_csv(OUT/"historical_full70_exact_prefix.csv",[dict(permutation=i,exact_prefix_calls70=b) for i,b in enumerate(exact_costs)])
    summary["full70_exact_prefix_secondary"]=dict(median=float(np.median(exact_costs)),
        min=min(exact_costs),max=max(exact_costs),p05=float(np.percentile(exact_costs,5)),
        p95=float(np.percentile(exact_costs,95)),warning="32 is earliest tested ladder checkpoint median, not exact earliest-prefix cost")
    return reference,hits,curves,summary


def proposal_audit():
    """Reconstruct algebra/pose and FPS order independently of the run code."""
    uobs={int(r["transaction_id"]):r for r in read_csv(ARCHIVE/"dual_u.csv")}
    random={(int(r["frame"]),int(r["random_rep"])):vector(r["basis_rowmajor"],12).reshape(6,2)
            for r in read_csv(OUT/"random_subspaces.csv")}
    groups={};max_formula=max_coord=max_complement=max_pose_t=max_pose_r=0.
    for row in read_csv(OUT/"conditioned_proposals.csv"):
        tx=int(row["frame"]);rep=int(row["random_rep"]);method=row["method"]
        eig=vector(uobs[tx]["curvature_eigenvectors_rowmajor"],36).reshape(6,6)
        q=eig[:,:2] if method=="COND_WEAK2" else eig[:,4:] if method=="COND_STRONG2" else random[tx,rep]
        eta=vector(row["eta_original"],6);pred=vector(row["eta_pred"],6)
        actual=vector(row["eta_conditioned"],6);coord=np.array([float(row["coord0"]),float(row["coord1"])])
        expected_coord=q.T@(eta-pred);expected=pred+q@expected_coord
        max_formula=max(max_formula,float(np.linalg.norm(actual-expected)))
        max_coord=max(max_coord,float(np.linalg.norm(coord-expected_coord)))
        max_complement=max(max_complement,float(np.linalg.norm((np.eye(6)-q@q.T)@(actual-pred))))
        base=vector(row["nominal_pose_matrix16"],16).reshape(4,4).astype(np.float32)
        reconstructed=base.copy()
        reconstructed[:3,3]+=np.float32(.8)*expected[:3].astype(np.float32)
        reconstructed[:3,:3]=(Rotation.from_rotvec(expected[3:]).as_matrix()@base[:3,:3].astype(float)).astype(np.float32)
        if int(row["seed_index"])==122:
            require(row["start_pose_matrix16"]==row["predictor_pose_matrix16"],"first proposal not exact predictor")
        dt,dr=separation(pose(text(reconstructed.ravel())),pose(row["start_pose_matrix16"]))
        max_pose_t=max(max_pose_t,dt);max_pose_r=max(max_pose_r,dr)
        groups.setdefault((tx,method,rep),[]).append(row)
    require(max(max_formula,max_coord,max_complement)<=1e-12,"conditioned algebra mismatch")
    require(max_pose_t<=1e-5 and max_pose_r<=1e-4,"conditioned algebra/actual-pose mismatch")
    selected={}
    for row in read_csv(OUT/"probe_manifest.csv"):
        selected.setdefault((int(row["frame"]),row["method"],int(row["random_rep"])),[]).append(row)
    require(set(groups)==set(selected)=={(tx,m,r) for tx in FRAME_ORDER for m,r in METHODS},"proposal groups mismatch")
    for key,rows in groups.items():
        by_seed={int(r["seed_index"]):r for r in rows}
        require(len(rows)==263 and sorted(by_seed)==list(range(263)),"pool seed coverage mismatch")
        points=np.array([[float(by_seed[j]["coord0"]),float(by_seed[j]["coord1"])] for j in range(263)])
        pair_d2=((points[:,None,:]-points[None,:,:])**2).sum(axis=2)
        order=[122]
        for _ in range(15):
            remaining=[j for j in range(263) if j not in order]
            distances=np.array([pair_d2[j,order].min() for j in remaining])
            largest=float(distances.max());tol=1e-12*max(1.,abs(largest))
            order.append(next(j for j,d in zip(remaining,distances) if d>=largest-tol))
        manifest_rows=sorted(selected[key],key=lambda r:int(r["probe_rank"]))
        require([int(r["seed_index"]) for r in manifest_rows]==order,"independent FPS order mismatch")
        for row in manifest_rows:
            require(row["start_pose_matrix16"]==by_seed[int(row["seed_index"])]["start_pose_matrix16"],"selected start not from pool")
    probe_lookup={(int(r["frame"]),r["method"],int(r["random_rep"]),int(r["probe_rank"])):r
                  for rows in selected.values() for r in rows}
    runs=read_csv(OUT/"ndt_runs.csv")
    require(len(runs)==len(probe_lookup)==2560,"run/proposal count mismatch")
    seen=set()
    for row in runs:
        key=(int(row["frame"]),row["method"],int(row["random_rep"]),int(row["probe_rank"]))
        require(key not in seen,"duplicate actual call");seen.add(key)
        require(row["start_pose_matrix16"]==probe_lookup[key]["start_pose_matrix16"] and
                row["seed_index"]==probe_lookup[key]["seed_index"],"run/proposal identity mismatch")
    parity=read_csv(OUT/"proposal_roundtrip.csv")
    require(len(parity)==42080 and all(r["pass"]=="1" for r in parity),"proposal round-trip failure")
    return dict(pass_gate=True,proposals=42080,selected=2560,fps_groups=160,
        max_formula_error=max_formula,max_coordinate_error=max_coord,max_complement_error=max_complement,
        independently_reconstructed_pose_max_translation_m=max_pose_t,
        independently_reconstructed_pose_max_rotation_deg=max_pose_r,
        roundtrip_max_translation_m=max(float(r["translation_error_m"]) for r in parity),
        roundtrip_max_rotation_deg=max(float(r["rotation_error_deg"]) for r in parity))


def predictor_audit(targets,reference,hits,pool):
    lookup={(int(r["frame"]),int(r["seed_index"])):r for r in pool if r["method"]=="COND_WEAK2"}
    uobs={int(r["transaction_id"]):r for r in read_csv(ARCHIVE/"dual_u.csv")}
    rows=[]
    for tx,cluster in MISSES:
        q=vector(uobs[tx]["curvature_eigenvectors_rowmajor"],36).reshape(6,6);s=q[:,2:]
        pred=vector(lookup[tx,122]["eta_pred"],6);v_pred=s.T@pred
        successful=[seed for seed in range(263) if hits[tx,"FULL6D",seed]==cluster]
        require(bool(successful),"frozen major basin has no successful FULL6D seed")
        vs=np.vstack([s.T@vector(lookup[tx,j]["eta_original"],6) for j in successful])
        norms=np.linalg.norm(vs,axis=1);distances=np.linalg.norm(vs-v_pred,axis=1)
        rows.append(dict(frame=tx,cluster_id=cluster,full_successful_count=len(successful),v_pred=text(v_pred),
            v_pred_norm=float(np.linalg.norm(v_pred)),distance_to_componentwise_median=float(np.linalg.norm(v_pred-np.median(vs,axis=0))),
            distance_to_nearest_successful_v=float(distances.min()),nearest_successful_seed=successful[int(distances.argmin())],
            successful_v_min=text(vs.min(axis=0)),successful_v_max=text(vs.max(axis=0)),
            successful_v_p05=text(np.percentile(vs,5,axis=0)),successful_v_p50=text(np.median(vs,axis=0)),
            successful_v_p95=text(np.percentile(vs,95,axis=0)),
            successful_v_norm_min=float(norms.min()),successful_v_norm_max=float(norms.max()),
            successful_v_norm_p05=float(np.percentile(norms,5)),successful_v_norm_p50=float(np.median(norms)),
            successful_v_norm_p95=float(np.percentile(norms,95)),
            predictor_norm_empirical_percentile=float(np.mean(norms<=np.linalg.norm(v_pred))),
            distance_p05=float(np.percentile(distances,5)),distance_median=float(np.median(distances)),
            distance_p95=float(np.percentile(distances,95)),oracle_used_in_proposals=0))
    write_csv(OUT/"predictor_v_audit.csv",rows)
    return rows


def self_test():
    require(abs(auc(np.ones(16))-1.)<1e-12 and auc(np.zeros(16))==0.,"AUC normalization regression")
    identity=(np.zeros(3),Rotation.identity())
    p1=(np.array([.1,0,0]),Rotation.identity());p2=(np.array([.25,0,0]),Rotation.identity())
    require(complete_link([identity,p1,p2])==2,"complete-link incorrectly chained endpoints")
    t={368:[("P01",identity),("P02",p1)]}
    row=dict(frame="368",converged="1",terminal_pose_xyz_q_xyzw="0.04;0;0;0;0;0;1")
    winner,eligible=admitted(row,t)
    require(winner=="P01" and len(eligible)==2,"overlapping IDs double-counted")
    row["converged"]="0";require(admitted(row,t)==("NONE",[]),"nonconverged terminal admitted")
    candidates=[dict(raw_ndt_score_sum="10",probe_rank="1"),dict(raw_ndt_score_sum="20",probe_rank="2")]
    chosen,switched=choose_objective(candidates,15.)
    require(chosen["probe_rank"]=="2" and switched,"raw score sign inverted: energy is minus PCL score")
    require(not choose_objective(candidates,20.)[1],"nominal score tie must not switch")
    require(not choose_objective(candidates,21.)[1],"worse candidate must not replace nominal")
    print("R2A_STATISTICS_SELF_TEST=PASS")


def posthoc_gt(selection,nominals,cohort):
    # Selection sidecar is written and hashed before this first read of any GT values.
    require((OUT/"objective_selection_pre_gt.csv").is_file(),"objective choices must be frozen before GT")
    choice_hash=digest(OUT/"objective_selection_pre_gt.csv")
    sys.path.insert(0,str(HERE.parent))
    import p5_i1_posthoc_gt as contract
    import yaml
    require(digest(contract.GT)==contract.EXPECTED_GT_SHA and digest(contract.EXTRINSICS)==contract.EXPECTED_EXTR_SHA,
            "official posthoc GT/calibration hashes differ")
    times,poses=contract.gt_data(contract.GT)
    extr=yaml.safe_load(contract.EXTRINSICS.read_text())
    t_i_l=np.array(extr["laser_to_imu"]["data"],dtype=float).reshape(4,4)
    t_i_l[:3,:3]=Rotation.from_matrix(t_i_l[:3,:3]).as_matrix();t_l_i=np.linalg.inv(t_i_l)
    corrected_path=contract.RUN/"evaluation_inputs/corrected.csv"
    first=next(r for r in read_csv(corrected_path) if float(r["lidar_header_stamp"])>=contract.EVAL_START)
    first_gt=contract.interpolate_gt(times,poses,float(first["lidar_header_stamp"]))
    require(first_gt is not None,"GT lacks historical fixed anchor")
    first_l=contract.parse_pose(";".join(first[k] for k in (
        "final_used_tx","final_used_ty","final_used_tz","final_used_qx","final_used_qy","final_used_qz","final_used_qw")))
    anchor=(first_l@t_l_i)@np.linalg.inv(first_gt)
    old_metrics_path=ARCHIVE/"posthoc_gt/frame_mode_metrics.csv"
    old={int(r["transaction_id"]):r for r in read_csv(old_metrics_path)}
    gt_by_frame={};nominal_errors={};parity=[]
    for tx in FRAME_ORDER:
        raw=contract.interpolate_gt(times,poses,int(cohort[tx]["stamp_ns"])/1e9)
        require(raw is not None,"GT does not bracket frozen frame")
        gt_by_frame[tx]=anchor@raw
        dt,dr=contract.pose_error(contract.parse_pose(nominals[tx])@t_l_i,gt_by_frame[tx])
        nominal_errors[tx]=(dt,dr)
        error_t=abs(dt-float(old[tx]["baseline_raw_translation_gt_error_m"]))
        error_r=abs(dr-float(old[tx]["baseline_raw_rotation_gt_error_deg"]))
        parity.append(dict(frame=tx,translation_error_parity_m=error_t,rotation_error_parity_deg=error_r,
                           pass_parity=int(error_t<=1e-4 and error_r<=1e-3)))
    write_csv(OUT/"gt_contract_parity.csv",parity)
    require(all(r["pass_parity"] for r in parity),"historical GT/map-frame parity failed; no new alignment fitted")
    output=[]
    for row in selection:
        tx=int(row["frame"]);et,er=contract.pose_error(contract.parse_pose(row["selected_pose_matrix16"])@t_l_i,gt_by_frame[tx])
        nt,nr=nominal_errors[tx]
        error=math.hypot(et/.8,math.radians(er));nominal_error=math.hypot(nt/.8,math.radians(nr))
        delta=error-nominal_error
        output.append(dict(row,gt_used="POSTHOC_ONLY",nominal_gt_translation_m=nt,nominal_gt_rotation_deg=nr,
                           selected_gt_translation_m=et,selected_gt_rotation_deg=er,delta_gt_translation_m=et-nt,
                           delta_gt_rotation_deg=er-nr,nominal_combined_error=nominal_error,selected_combined_error=error,
                           posthoc_result="SAME" if abs(delta)<=1e-6 else "IMPROVED" if delta<0 else "WORSE"))
    write_csv(OUT/"objective_selection.csv",output)
    require(digest(OUT/"objective_selection_pre_gt.csv")==choice_hash,"GT altered objective selection")
    stats=[]
    for method,rep in METHODS:
        for budget in BUDGETS:
            subset=[r for r in output if r["method"]==method and int(r["random_rep"])==rep and int(r["budget"])==budget]
            major=[r for r in subset if int(r["frame"]) in MAJOR];healthy=[r for r in subset if int(r["frame"]) not in MAJOR]
            stats.append(dict(method=method,random_rep=rep,budget=budget,
                major_improved=sum(r["posthoc_result"]=="IMPROVED" for r in major),
                major_same=sum(r["posthoc_result"]=="SAME" for r in major),
                major_worse=sum(r["posthoc_result"]=="WORSE" for r in major),
                healthy_left_nominal=sum(int(r["left_nominal"]) for r in healthy),
                healthy_left_and_posthoc_worse=sum(int(r["left_nominal"]) and r["posthoc_result"]=="WORSE" for r in healthy)))
    write_csv(OUT/"objective_selection_summary.csv",stats)
    inputs={str(p):digest(p) for p in (contract.GT,contract.EXTRINSICS,corrected_path,old_metrics_path,
                                     Path(contract.__file__))}
    return stats,dict(gt_used="POSTHOC_ONLY",choice_sha256_before_gt=choice_hash,
        fixed_reference_alignment=text(anchor),anchor_stamp=first["lidar_header_stamp"],
        nominal_error_parity_max_translation=max(r["translation_error_parity_m"] for r in parity),
        nominal_error_parity_max_rotation=max(r["rotation_error_parity_deg"] for r in parity),input_sha256=inputs)


def score_carrier_audit(selection,groups,nominals,cohort):
    # Only exact matching archived pose texts qualify; this is not a new score fit/evaluation.
    zeros={int(row["transaction_id"]):row for row in read_csv(ARCHIVE/"candidates.csv") if row["seed_index"]=="122"}
    exact={tx for tx,row in zeros.items() if row["final_pose_xyz_q_xyzw"]==cohort[tx]["raw_terminal_pose_xyz_q_xyzw"]}
    output=[]
    for row in selection:
        tx=int(row["frame"])
        if tx not in exact:continue
        candidates=groups[tx,row["method"],row["random_rep"]][:row["budget"]]
        alternative_score=float(zeros[tx]["raw_ndt_score_sum"])
        chosen,switched=choose_objective(candidates,alternative_score)
        alternative=chosen["terminal_pose_matrix16"] if switched else nominals[tx]
        dt,dr=separation(pose(row["selected_pose_matrix16"]),pose(alternative))
        output.append(dict(frame=tx,method=row["method"],random_rep=row["random_rep"],budget=row["budget"],
            current_nominal_score=row["nominal_raw_score"],archived_exact_pose_score=alternative_score,
            score_gap=row["nominal_raw_score"]-alternative_score,
            current_selected_rank=row["selected_rank"],archived_score_selected_rank=int(chosen["probe_rank"]) if switched else 0,
            selected_rank_changed=int(int(row["selected_rank"])!=(int(chosen["probe_rank"]) if switched else 0)),
            selected_translation_change_m=dt,selected_rotation_change_deg=dr,
            material_selected_pose_change=int((dt>.2 or dr>2.))))
    write_csv(OUT/"score_carrier_sensitivity.csv",output)
    return dict(exact_pose_comparison_frames=len(exact),choices=len(output),
        max_nominal_score_gap=max(abs(row["score_gap"]) for row in output),
        changed_exact_tie_choices=sum(row["selected_rank_changed"] for row in output),
        max_selected_translation_change_m=max(row["selected_translation_change_m"] for row in output),
        max_selected_rotation_change_deg=max(row["selected_rotation_change_deg"] for row in output),
        materially_changed_selected_pose_count=sum(row["material_selected_pose_change"] for row in output),
        limitation="float U_obs nominal quaternion carrier differs from archived double Pose3d; tiny score differences and exact ties are not robust")


def analyze():
    manifest=verify_manifest();require(manifest["current_run_state"]=="RUN_COMPLETE","NDT must complete before labels/GT evaluation")
    require(digest(OUT/"ndt_runs.csv")==manifest["ndt_runs_sha256"],"NDT result hash changed")
    proposals_checked=proposal_audit()
    input_hashes=committed_inputs([CANONICAL,H1/"major_projection_statistics.csv",H1/"frame_projection_statistics.csv",
        H2/"ndt_runs.csv",H2/"terminal_admission.csv",H2/"permutation_manifest.csv",H2/"results.json"])
    targets={}
    for r in read_csv(CANONICAL):targets.setdefault(int(r["transaction_id"]),[]).append((r["cluster_id"],pose(r["canonical_pose_matrix16"])))
    for values_ in targets.values():values_.sort(key=lambda item:item[0])
    require(tuple(sorted(targets))==MAJOR and sum(map(len,targets.values()))==22,"frozen22/9 oracle changed")
    runs=read_csv(OUT/"ndt_runs.csv");require(len(runs)==2560,"NDT result count mismatch")
    groups={};assigned={};admission=[]
    for row in runs:
        key=(int(row["frame"]),row["method"],int(row["random_rep"]))
        groups.setdefault(key,[]).append(row)
        if key[0] in MAJOR:
            winner,eligible=admitted(row,targets)
            assigned[key+(int(row["probe_rank"]),)]=winner
            admission.append(dict(frame=key[0],method=key[1],random_rep=key[2],probe_rank=row["probe_rank"],
                seed_index=row["seed_index"],converged=row["converged"],assigned_cluster=winner,
                eligible_clusters=";".join(x[1] for x in eligible) or "NONE",eligible_count=len(eligible)))
    for key,rows in groups.items():
        rows.sort(key=lambda r:int(r["probe_rank"]))
        require([int(r["probe_rank"]) for r in rows]==list(range(1,17)),"probe prefix incomplete")
    write_csv(OUT/"terminal_admission.csv",admission)
    recall={};recall_rows=[]
    for method,rep in METHODS:
        cube=np.zeros((9,16))
        for f,tx in enumerate(MAJOR):
            found=set()
            for b in range(1,17):
                winner=assigned[tx,method,rep,b]
                if winner!="NONE":found.add(winner)
                cube[f,b-1]=len(found)/len(targets[tx])
                recall_rows.append(dict(frame=tx,method=method,random_rep=rep,budget=b,major_count=len(targets[tx]),
                    recovered_count=len(found),recovered_clusters=";".join(sorted(found)) or "NONE",recall=cube[f,b-1]))
        recall[method,rep]=cube
    write_csv(OUT/"basin_recall.csv",recall_rows)
    reference,old_hits,old_curves,historical_summary=historical(targets)
    pool=read_csv(OUT/"conditioned_proposals.csv")
    pred_audit=predictor_audit(targets,reference,old_hits,pool)
    macro_auc={key:float(np.mean([auc(row) for row in cube])) for key,cube in recall.items()}
    random_auc=[macro_auc["COND_RANDOM2",rep] for rep in range(3)]
    # Mean of per-frame median replicate AUC matches the historical frame-macro analysis contract.
    random_frame_auc=np.array([[auc(r) for r in recall["COND_RANDOM2",rep]] for rep in range(3)])
    random_median_auc=float(np.median(random_frame_auc,axis=0).mean())
    counts=np.array([len(targets[tx]) for tx in MAJOR]);budget_rows=[]
    for method,rep in METHODS:
        c=recall[method,rep]
        for b in range(1,17):budget_rows.append(dict(method=method,random_rep=rep,budget=b,
            macro_recall=float(c[:,b-1].mean()),micro_recall=float((c[:,b-1]*counts).sum()/22),
            recovered_ids=int(round((c[:,b-1]*counts).sum())),auc_log_budget_1_16=macro_auc[method,rep]))
    for b in range(1,17):
        macros=[float(recall["COND_RANDOM2",rep][:,b-1].mean()) for rep in range(3)]
        micros=[float((recall["COND_RANDOM2",rep][:,b-1]*counts).sum()/22) for rep in range(3)]
        budget_rows.append(dict(method="COND_RANDOM2_MEDIAN",random_rep=-2,budget=b,
            macro_recall=float(np.median(macros)),micro_recall=float(np.median(micros)),recovered_ids="REPLICATE_MEDIAN",
            auc_log_budget_1_16=random_median_auc,macro_min=min(macros),macro_max=max(macros),
            micro_min=min(micros),micro_max=max(micros)))
    write_csv(OUT/"budget_summary.csv",budget_rows)
    weak=recall["COND_WEAK2",-1];strong=recall["COND_STRONG2",-1]
    old_weak_frame=np.median(old_curves["WEAK2"],axis=0)
    key_rows=[];primary_by_budget={}
    for tx,cluster in MISSES:
        old_found=any(old_hits[tx,"WEAK2",j]==cluster for j in range(263))
        for b in BUDGETS:
            recovered=any(assigned[tx,"COND_WEAK2",-1,rank]==cluster for rank in range(1,b+1))
            key_rows.append(dict(frame=tx,cluster_id=cluster,budget=b,old_weak_full263_recovered=int(old_found),
                conditioned_weak_recovered=int(recovered),primary=int((tx,cluster) in MISSES[:3])))
    write_csv(OUT/"h2_missed_cases.csv",key_rows)
    same_index_rows=[];frame_rows=[]
    h1={int(r["frame"]):float(r["R_weak2"]) for r in read_csv(H1/"frame_projection_statistics.csv")}
    for f,tx in enumerate(MAJOR):
        seeds=[int(r["seed_index"]) for r in groups[tx,"COND_WEAK2",-1]]
        same_full=set();same_old=set()
        for b,seed in enumerate(seeds,1):
            for m,found in (("FULL6D",same_full),("WEAK2",same_old)):
                hit=old_hits[tx,m,seed]
                if hit!="NONE":found.add(hit)
            same_index_rows.append(dict(frame=tx,budget=b,seed_index=seed,
                cond_weak_recall=weak[f,b-1],old_weak_same_indices_recall=len(same_old)/len(targets[tx]),
                full_same_indices_recall=len(same_full)/len(targets[tx])))
        random_frame=[auc(recall["COND_RANDOM2",rep][f]) for rep in range(3)]
        frame_rows.append(dict(frame=tx,major_count=len(targets[tx]),rho_W2=h1[tx],
            conditioned_weak_auc=auc(weak[f]),conditioned_strong_auc=auc(strong[f]),
            random_auc_median=float(np.median(random_frame)),random_auc_min=min(random_frame),random_auc_max=max(random_frame),
            old_weak_auc_frame_median=float(np.median([auc(c[f]) for c in old_curves["WEAK2"]])),
            full_auc_frame_median=float(np.median([auc(c[f]) for c in old_curves["FULL6D"]])),
            conditioned_weak_final_recall=weak[f,-1],old_weak_final_recall_median=old_weak_frame[f,-1],
            weak_vs_same_indices_old_final=weak[f,-1]-len(same_old)/len(targets[tx])))
    write_csv(OUT/"same_index_historical_control.csv",same_index_rows)
    write_csv(OUT/"per_frame_discovery.csv",frame_rows)
    same_index_summary={}
    for method,field in (("OLD_WEAK2","old_weak_same_indices_recall"),("FULL6D","full_same_indices_recall")):
        curves=np.array([[float(r[field]) for r in same_index_rows if r["frame"]==tx] for tx in MAJOR])
        same_index_summary[method]=dict(auc_common_1_16=float(np.mean([auc(row) for row in curves])),
            macro_by_budget={str(b):float(curves[:,b-1].mean()) for b in BUDGETS})
    parity=read_csv(OUT/"predictor_parity.csv")
    nominals={int(r["frame"]):r["nominal_pose_matrix16"] for r in parity}
    cohort={int(r["transaction_id"]):r for r in read_csv(ARCHIVE/"frozen/cohort_frozen.csv")}
    selection=[];healthy=[];cost=[]
    for tx in FRAME_ORDER:
        nominal=pose(nominals[tx])
        for method,rep in METHODS:
            rows=groups[tx,method,rep]
            nominal_score=float(rows[0]["nominal_ndt_score_sum"])
            require(all(float(r["nominal_ndt_score_sum"])==nominal_score for r in rows),"nominal objective changed between calls")
            for b in BUDGETS:
                subset=rows[:b];chosen,switched=choose_objective(subset,nominal_score)
                matrix=chosen["terminal_pose_matrix16"] if switched else nominals[tx]
                dt,dr=separation(nominal,pose(matrix))
                selection.append(dict(frame=tx,label="MAJOR" if tx in MAJOR else "NO_MAJOR",method=method,random_rep=rep,
                    budget=b,selected_rank=chosen["probe_rank"] if switched else 0,
                    selected_seed_index=chosen["seed_index"] if switched else "NOMINAL",
                    selected_pose_matrix16=matrix,nominal_raw_score=nominal_score,
                    selected_raw_score=max(nominal_score,float(chosen["raw_ndt_score_sum"])),
                    nominal_objective=-nominal_score,
                    selected_objective=-max(nominal_score,float(chosen["raw_ndt_score_sum"])),
                    objective_switched=int(switched),left_nominal=int(dt>.2 or dr>2),
                    selected_translation_from_nominal_m=dt,selected_rotation_from_nominal_deg=dr,
                    selected_converged=chosen["converged"] if switched else "NOMINAL"))
                if tx not in MAJOR:
                    all_poses=[nominal]+[pose(r["terminal_pose_xyz_q_xyzw"]) for r in subset]
                    converged=[nominal]+[pose(r["terminal_pose_xyz_q_xyzw"]) for r in subset if r["converged"]=="1"]
                    healthy.append(dict(frame=tx,method=method,random_rep=rep,budget=b,
                        terminal_clusters_all=complete_link(all_poses),terminal_clusters_converged=complete_link(converged),
                        additional_clusters=max(0,complete_link(all_poses)-1),
                        best_objective_improvement=max(0.,float(chosen["raw_ndt_score_sum"])-nominal_score),
                        best_candidate_translation_m=float(chosen["translation_from_nominal_m"]),
                        best_candidate_rotation_deg=float(chosen["rotation_from_nominal_deg"]),
                        objective_switched=int(switched),left_nominal=int(dt>.2 or dr>2),
                        iteration_limit_count=sum(int(r["iterations"])>=80 for r in subset),
                        iteration_limit_rate=sum(int(r["iterations"])>=80 for r in subset)/b))
    write_csv(OUT/"healthy_control.csv",healthy)
    write_csv(OUT/"objective_selection_pre_gt.csv",selection)
    carrier_audit=score_carrier_audit(selection,groups,nominals,cohort)
    for method,rep in METHODS:
        for b in BUDGETS:
            subset=[r for tx in FRAME_ORDER for r in groups[tx,method,rep][:b]]
            cost.append(dict(method=method,random_rep=rep,budget=b,calls_per_frame=b,
                mean_runtime_ms_per_probe=float(np.mean([float(r["runtime_ms"]) for r in subset])),
                mean_runtime_ms_per_frame=sum(float(r["runtime_ms"]) for r in subset)/32,
                mean_iterations_per_probe=float(np.mean([int(r["iterations"]) for r in subset])),
                mean_iterations_per_frame=sum(int(r["iterations"]) for r in subset)/32))
    write_csv(OUT/"costs.csv",cost)
    full70=float(json.loads((H2/"results.json").read_text())["matched_recall_cost"]["FULL6D"]["0.7"]["median"])
    decisions=[]
    for b in BUDGETS:
        primary_count=sum(r["conditioned_weak_recovered"] for r in key_rows if r["primary"] and r["budget"]==b)
        gate=dict(A=bool(weak[:,b-1].mean()>=.70),B=bool(macro_auc["COND_WEAK2",-1]>macro_auc["COND_STRONG2",-1]),
            C=bool(macro_auc["COND_WEAK2",-1]>random_median_auc),
            D=bool(np.sum(weak[:,b-1]>=old_weak_frame[:,b-1]-1e-12)>=7),E=primary_count>=2,
            F=bool(weak[:,b-1].mean()>=.70 and b<=16 and full70==32))
        decisions.append(dict(budget=b,gates=gate,pass_gate=all(gate.values()),primary_recovered=primary_count,
            frames_at_least_old_weak=int(np.sum(weak[:,b-1]>=old_weak_frame[:,b-1]-1e-12))))
    best=next((d for d in decisions if d["pass_gate"]),None)
    helps=(macro_auc["COND_WEAK2",-1]>historical_summary["WEAK2"]["auc_common_1_16"] and
           weak[:,-1].mean()>historical_summary["WEAK2"]["macro_b16_median"])
    if best:final="LOW_BUDGET_PREDICTOR_CONDITIONING_SUPPORTED" if best["budget"]<=8 else "PREDICTOR_CONDITIONED_WEAK_SEARCH_SUPPORTED"
    elif weak[:,-1].mean()>=.70 and decisions[-1]["primary_recovered"]>=2 and (not decisions[-1]["gates"]["B"] or not decisions[-1]["gates"]["C"]):
        final="GENERIC_CONDITIONAL_SEARCH_ONLY"
    elif helps and weak[:,-1].mean()<.70:final="PREDICTOR_CONDITIONING_HELPS_BUT_INSUFFICIENT"
    else:final="PREDICTOR_CONDITIONED_WEAK_SEARCH_NOT_SUPPORTED"
    gt_summary,gt_contract=posthoc_gt(selection,nominals,cohort)
    healthy_summary=[]
    for method,rep in METHODS:
        for b in BUDGETS:
            subset=[r for r in healthy if r["method"]==method and r["random_rep"]==rep and r["budget"]==b]
            healthy_summary.append(dict(method=method,random_rep=rep,budget=b,frames=23,
                cluster_count_mean=float(np.mean([r["terminal_clusters_all"] for r in subset])),
                cluster_count_max=max(r["terminal_clusters_all"] for r in subset),
                objective_switch_count=sum(r["objective_switched"] for r in subset),left_nominal_count=sum(r["left_nominal"] for r in subset),
                iteration_limit_rate=float(np.mean([r["iteration_limit_rate"] for r in subset]))))
    write_csv(OUT/"healthy_summary.csv",healthy_summary)
    result=dict(task=manifest["task"],current_run_state="COMPLETE",git=git_state(),final_result=final,
        next="PREDICTOR_CONDITIONED_EVIDENCE_COST_REDUCTION" if best else "VISUAL_INDEPENDENT_NONLOCAL_EVIDENCE_GATE",
        best_budget=best["budget"] if best else None,predictor_parity=dict(pass_gate=True,frames=32,
            max_translation_m=max(float(r["translation_error_m"]) for r in parity),
            max_rotation_deg=max(float(r["rotation_error_deg"]) for r in parity)),
        proposal_contract=proposals_checked,
        macro_auc=dict(COND_WEAK2=macro_auc["COND_WEAK2",-1],COND_STRONG2=macro_auc["COND_STRONG2",-1],
            COND_RANDOM2_FRAME_MEDIAN=random_median_auc,random_replicate_macro_auc=random_auc,
            COND_RANDOM2_REPLICATE_MEDIAN=float(np.median(random_auc)),
            COND_RANDOM2_REPLICATE_MIN=min(random_auc),COND_RANDOM2_REPLICATE_MAX=max(random_auc)),
        historical=historical_summary,same_index_reference=same_index_summary,decisions=decisions,recall_curves=budget_rows,
        missed_cases=key_rows,predictor_v_audit=pred_audit,per_frame=frame_rows,healthy=healthy_summary,
        objective_selection_summary=gt_summary,gt_contract=gt_contract,score_carrier_sensitivity=carrier_audit,cost=cost,
        preparation_seconds=manifest["preparation_seconds"],new_ndt_calls=2560,full6d_new_calls=0,
        nonconverged=sum(r["converged"]=="0" for r in runs),iteration_limit=sum(int(r["iterations"])>=80 for r in runs),
        multiply_eligible_new_terminals=sum(r["eligible_count"]>1 for r in admission),
        admission_warning="single deterministic archive-ID assignment; overlapping frozen admission regions remain",
        gt_used="POSTHOC_ONLY",oracle_used_for_proposals=False,execution_manifest_sha256=digest(OUT/"execution_manifest.json"),
        objective_contract="E=-raw_ndt_score_sum; minimize E, equivalently maximize score; nominal wins ties",
        analysis_correction="OBJECTIVE_SIGN_CORRECTION.md supersedes pre-run THEORY.md raw-score sign only",
        evaluation_input_sha256=input_hashes,analysis_code_sha256=digest(Path(__file__)),artifact_sha256={})
    result["schedule_checkpoint_call_ratio_32_over_supported_B"]=full70/best["budget"] if best else None
    result["online_competitive"]=False
    (OUT/"REPORT.md").write_text(report(result))
    artifacts=[p for p in OUT.rglob("*") if p.is_file() and p.name!="results.json"]
    result["artifact_sha256"]={p.relative_to(OUT).as_posix():digest(p) for p in sorted(artifacts)}
    (OUT/"results.json").write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
    print(json.dumps(dict(FINAL_RESULT=final,BEST_BUDGET=result["best_budget"],AUC=result["macro_auc"],
                         DECISIONS=decisions),indent=2))


def report(result):
    lines=["# P9-R2A Predictor-conditioned weak-subspace search","",f"FINAL_RESULT = `{result['final_result']}`","",
        "Frozen nine-frame/22-ID discovery gate, with 23 no-major safety controls. Oracle labels are evaluation only; GT is post-hoc only.",
        "", "## Predictor/proposal contract", "",
        f"Seed122 predictor parity: max {result['predictor_parity']['max_translation_m']:.6g}m / {result['predictor_parity']['max_rotation_deg']:.6g}deg across32 frames.",
        "`eta = eta_pred + Q Q^T (eta_j-eta_pred)`; first seed122, then deterministic farthest-point selection. The complete 16-call order is frozen. Three R2 Haar bases/frame are reused (PCG64 seed20261007+1000*frame_index+rep).",
        "All methods use the unchanged frozen source/map/PCL1.10 NDT parameters. New calls2560; FULL6D new calls0.",
        "The pre-run THEORY.md contains an incorrect raw-score sign sentence. Its original hashed bytes are preserved; OBJECTIVE_SIGN_CORRECTION.md is the authoritative correction: E=-S, so lowest energy means highest raw PCL score. Corrected offline selection is completed before GT scoring; no NDT output or discovery metric is changed.",
        "", "## Recall (frame macro / ID micro)", "", "|B|Method/rep|Macro|Micro|Random macro range|", "|---:|---|---:|---:|---|"]
    for r in result["recall_curves"]:
        if r["budget"] in BUDGETS:
            span=f"[{r['macro_min']:.6f}, {r['macro_max']:.6f}]" if "macro_min" in r else "—"
            lines.append(f"|{r['budget']}|{r['method']}/{r['random_rep']}|{r['macro_recall']:.6f}|{r['micro_recall']:.6f}|{span}|")
    lines += ["", "## AUC and gates", "", "AUC uses the complete common1..16 nested prefix against log2(B). Historical H2 uses its frozen100 permutations; these historical orders differ from the new deterministic FPS order. Same-index controls are retained separately.",
              "RANDOM frame-median macro AUC and median of three replicate macro AUCs are distinct aggregates; both and all replicate values are reported. The gate uses the former; WEAK exceeds both.",
              "Important matched-index control: conditioned WEAK AUC0.399270 is lower than old zero-complement WEAK AUC0.479157 when both use the new WEAK seed-index order. Thus the prespecified discovery gate passes, but an across-cohort causal advantage from conditioning alone is not established. The two newly recovered2350 IDs remain genuine differences because old WEAK missed them over all263 seeds.",
              "", "```json",json.dumps(dict(auc=result["macro_auc"],historical=result["historical"],same_index_reference=result["same_index_reference"],decisions=result["decisions"]),indent=2),"```",
              "", "## H2 final misses", "", f"Supported budget: {result['best_budget']}; PRIMARY has three IDs but only two independent frames. This is a prespecified ID gate, not three independent samples.",
              "", "|TX/ID|Old WEAK all263|COND WEAK B12|COND WEAK B16|", "|---|---:|---:|---:|"]
    for r in result["missed_cases"]:
        if r["budget"]==16:
            b12=next(x for x in result["missed_cases"] if x["frame"]==r["frame"] and x["cluster_id"]==r["cluster_id"] and x["budget"]==12)
            lines.append(f"|{r['frame']}/{r['cluster_id']}|{r['old_weak_full263_recovered']}|{b12['conditioned_weak_recovered']}|{r['conditioned_weak_recovered']}|")
    lines += ["", "## Predictor complement audit", "", "|TX/ID|Successful FULL seeds|norm v_pred|Distance to median v|Distance to nearest v|", "|---|---:|---:|---:|---:|"]
    for r in result["predictor_v_audit"]:lines.append(f"|{r['frame']}/{r['cluster_id']}|{r['full_successful_count']}|{r['v_pred_norm']:.6g}|{r['distance_to_componentwise_median']:.6g}|{r['distance_to_nearest_successful_v']:.6g}|")
    lines += ["", "## Per-frame discovery", "", "|TX|IDs|H1rho|WEAK AUC|STRONG AUC|RANDOM median [range] AUC|WEAK B16|OLD WEAK B16 median|", "|---:|---:|---:|---:|---:|---|---:|---:|"]
    for r in result["per_frame"]:lines.append(f"|{r['frame']}|{r['major_count']}|{r['rho_W2']:.6f}|{r['conditioned_weak_auc']:.6f}|{r['conditioned_strong_auc']:.6f}|{r['random_auc_median']:.6f} [{r['random_auc_min']:.6f}, {r['random_auc_max']:.6f}]|{r['conditioned_weak_final_recall']:.6f}|{r['old_weak_final_recall_median']:.6f}|")
    lines += ["", "## No-major controls and objective selection", "",
        "New candidate clustering is descriptive and does not recluster the oracle. No-major frames can contain legitimate non-major alternatives; cluster proliferation is not a proven false-positive count.",
        "Minimum energy E=-S is selected among nominal and all finite returns before GT is loaded (equivalently highest raw score). Nominal-reference errors reproduce the existing fixed-anchor GT contract.",
        "The wrong-sign preliminary safety result is withdrawn. Corrected post-hoc scoring below is secondary; candidate discovery support alone does not establish that objective-only pose switching is safe.",
        "Nominal-score carrier sensitivity is retained separately. Nominal U_obs quaternion reconstruction uses float, while archived terminal scores use a normalized double quaternion before Matrix4f. Tiny score gaps/exact tie identities are not robust; the substitution check is limited to byte-identical archived nominal pose texts and does not change material selected-pose geometry.",
        "", "```json",json.dumps(result["score_carrier_sensitivity"],indent=2),"```",
        "", "|B|Method/rep|Healthy mean/max clusters|Objective switches|Left nominal|Iteration-limit rate|Major improved/same/worse (GT)|Healthy left and GT worse|", "|---:|---|---:|---:|---:|---:|---|---:|"]
    for r in result["healthy"]:
        if r["budget"] in (12,16):
            g=next(x for x in result["objective_selection_summary"] if x["method"]==r["method"] and x["random_rep"]==r["random_rep"] and x["budget"]==r["budget"])
            lines.append(f"|{r['budget']}|{r['method']}/{r['random_rep']}|{r['cluster_count_mean']:.3f}/{r['cluster_count_max']}|{r['objective_switch_count']}|{r['left_nominal_count']}|{r['iteration_limit_rate']:.3f}|{g['major_improved']}/{g['major_same']}/{g['major_worse']}|{g['healthy_left_and_posthoc_worse']}|")
    lines += ["", "## Cost", "",f"Proposal preparation/selection: {result['preparation_seconds']:.3f}s total. Costs below are incremental discovery calls; total online calls would add the common nominal call. RANDOM2 has3 independent subspaces, so all-replicate cost is3B/frame.",
        "", "|B|Method/rep|Calls/frame|Mean ms/probe|Mean ms/frame|Mean iterations/probe|", "|---:|---|---:|---:|---:|---:|"]
    for r in result["cost"]:lines.append(f"|{r['budget']}|{r['method']}/{r['random_rep']}|{r['calls_per_frame']}|{r['mean_runtime_ms_per_probe']:.3f}|{r['mean_runtime_ms_per_frame']:.1f}|{r['mean_iterations_per_probe']:.2f}|")
    lines += ["",f"Nonconverged returns: {result['nonconverged']}; iteration-limit returns: {result['iteration_limit']}. Multiply eligible major-frame terminals: {result['multiply_eligible_new_terminals']}.",
        "NOT YET ONLINE-COMPETITIVE: WEAK B12 costs approximately738ms/frame for additional alignment calls alone, before the nominal call and runtime system overhead. The historical32/current12 call ratio2.67 is a checkpoint-schedule ratio, not an exact matched-recall or wall-time speedup. Historical FULL70 exact-prefix median is18 (secondary audit); the predeclared gate retains the original32 checkpoint reference.",
        "Independent post-run audit reconstructs every conditioned proposal and all160 FPS orders; it verifies pool/selected/actual-start identity without calling NDT.",
        "The frozen overlapping-ID admission limitation persists; ID recovery does not certify distinct dynamic minima. Wall time is descriptive across different historical/current runs.",
        "",f"BEST_BUDGET = `{result['best_budget']}`",f"NEXT = `{result['next']}`",""]
    return "\n".join(lines)


def audit():
    manifest=verify_manifest();r=json.loads((OUT/"results.json").read_text())
    require(manifest["current_run_state"]=="RUN_COMPLETE" and r["current_run_state"]=="COMPLETE","run/analysis incomplete")
    require(r["analysis_code_sha256"]==digest(Path(__file__)) and r["execution_manifest_sha256"]==digest(OUT/"execution_manifest.json"),"analysis code/manifest changed")
    for name,h in {**r["evaluation_input_sha256"],**r["gt_contract"]["input_sha256"]}.items():require(digest(name)==h,"evaluation input hash changed: "+name)
    for name,h in r["artifact_sha256"].items():require(digest(OUT/name)==h,"result artifact hash changed: "+name)
    require(digest(OUT/"ndt_runs.csv")==manifest["ndt_runs_sha256"],"NDT output changed")
    counts={"predictor_parity.csv":32,"conditioned_proposals.csv":42080,"probe_manifest.csv":2560,"ndt_runs.csv":2560,
        "basin_recall.csv":9*5*16,"budget_summary.csv":6*16,"predictor_v_audit.csv":6,"healthy_control.csv":23*5*4,
        "objective_selection.csv":32*5*4}
    for name,count in counts.items():require(len(read_csv(OUT/name))==count,"CSV denominator mismatch: "+name)
    require(r["new_ndt_calls"]==2560 and r["full6d_new_calls"]==0 and not r["oracle_used_for_proposals"],"scope/cost violation")
    require(proposal_audit()==r["proposal_contract"],"independent proposal audit/JSON mismatch")
    audit_evaluation(r)
    # Independently recompute macro/micro from the per-frame CSV, including the 22-ID denominator.
    recall_rows=read_csv(OUT/"basin_recall.csv")
    for b in range(1,17):
        for method,rep in METHODS:
            rows=[x for x in recall_rows if x["method"]==method and int(x["random_rep"])==rep and int(x["budget"])==b]
            summary=next(x for x in r["recall_curves"] if x["method"]==method and x["random_rep"]==rep and x["budget"]==b)
            require(abs(np.mean([float(x["recall"]) for x in rows])-summary["macro_recall"])<1e-12 and
                    abs(sum(int(x["recovered_count"]) for x in rows)/22-summary["micro_recall"])<1e-12,
                    "CSV/JSON recall mismatch")
    print(json.dumps(dict(audit="PASS",artifacts=len(r["artifact_sha256"]),new_ndt_calls=2560,gt_used="POSTHOC_ONLY"),indent=2))


def audit_evaluation(result):
    targets={}
    for row in read_csv(CANONICAL):
        targets.setdefault(int(row["transaction_id"]),[]).append((row["cluster_id"],pose(row["canonical_pose_matrix16"])))
    runs=read_csv(OUT/"ndt_runs.csv");groups={}
    for row in runs:groups.setdefault((int(row["frame"]),row["method"],int(row["random_rep"])),[]).append(row)
    for rows in groups.values():rows.sort(key=lambda row:int(row["probe_rank"]))
    admissions={(int(row["frame"]),row["method"],int(row["random_rep"]),int(row["probe_rank"])):row
                for row in read_csv(OUT/"terminal_admission.csv")}
    recovered={};aucs={}
    for method,rep in METHODS:
        curves=[]
        for tx in MAJOR:
            found=set();curve=[]
            for row in groups[tx,method,rep]:
                winner,eligible=admitted(row,targets);key=(tx,method,rep,int(row["probe_rank"]))
                require(winner==admissions[key]["assigned_cluster"] and len(eligible)==int(admissions[key]["eligible_count"]),
                        "raw-terminal admission/CSV mismatch")
                if winner!="NONE":found.add(winner)
                recovered[key]=set(found);curve.append(len(found)/len(targets[tx]))
            curves.append(curve)
        aucs[method,rep]=np.trapz(np.asarray(curves),np.log2(PREFIX),axis=1)/4.
    require(abs(float(aucs["COND_WEAK2",-1].mean())-result["macro_auc"]["COND_WEAK2"])<1e-12 and
            abs(float(aucs["COND_STRONG2",-1].mean())-result["macro_auc"]["COND_STRONG2"])<1e-12,
            "raw-terminal AUC mismatch")
    random=np.vstack([aucs["COND_RANDOM2",rep] for rep in range(3)])
    require(abs(float(np.median(random,axis=0).mean())-result["macro_auc"]["COND_RANDOM2_FRAME_MEDIAN"])<1e-12 and
            np.allclose(random.mean(axis=1),result["macro_auc"]["random_replicate_macro_auc"],atol=1e-12,rtol=0),
            "random replicate AUC mismatch")
    recall_rows={(int(row["frame"]),row["method"],int(row["random_rep"]),int(row["budget"])):row
                 for row in read_csv(OUT/"basin_recall.csv")}
    for key,ids in recovered.items():
        row=recall_rows[key]
        require(int(row["recovered_count"])==len(ids) and row["recovered_clusters"]==(";".join(sorted(ids)) or "NONE"),
                "raw-terminal recall/CSV mismatch")
    choices=read_csv(OUT/"objective_selection_pre_gt.csv")
    final={(int(row["frame"]),row["method"],int(row["random_rep"]),int(row["budget"])):row
           for row in read_csv(OUT/"objective_selection.csv")}
    nominals={int(row["frame"]):row["nominal_pose_matrix16"] for row in read_csv(OUT/"predictor_parity.csv")}
    for row in choices:
        key=(int(row["frame"]),row["method"],int(row["random_rep"]),int(row["budget"]))
        candidates=groups[key[:3]][:key[3]];nominal=float(candidates[0]["nominal_ndt_score_sum"])
        best_score=max([nominal]+[float(x["raw_ndt_score_sum"]) for x in candidates])
        expected=nominals[key[0]] if best_score==nominal else next(x["terminal_pose_matrix16"] for x in candidates if float(x["raw_ndt_score_sum"])==best_score)
        require(row["selected_pose_matrix16"]==expected and float(row["selected_objective"])==-best_score,
                "selection is not minimum exact PCL energy")
        require(all(final[key][field]==value for field,value in row.items()),"GT changed an objective choice")
        delta=float(final[key]["selected_combined_error"])-float(final[key]["nominal_combined_error"])
        require(final[key]["posthoc_result"]==("SAME" if abs(delta)<=1e-6 else "IMPROVED" if delta<0 else "WORSE"),
                "posthoc classification mismatch")
    for summary in result["objective_selection_summary"]:
        major=[row for key,row in final.items() if key[0] in MAJOR and key[1:]==(summary["method"],summary["random_rep"],summary["budget"])]
        require(all(summary["major_"+name.lower()]==sum(row["posthoc_result"]==name.upper() for row in major)
                    for name in ("improved","same","worse")),"posthoc counts mismatch")
    healthy_rows=read_csv(OUT/"healthy_control.csv")
    for row in healthy_rows:
        key=(int(row["frame"]),row["method"],int(row["random_rep"]),int(row["budget"]))
        candidates=groups[key[:3]][:key[3]];nominal=pose(nominals[key[0]])
        all_poses=[nominal]+[pose(x["terminal_pose_xyz_q_xyzw"]) for x in candidates]
        converged=[nominal]+[pose(x["terminal_pose_xyz_q_xyzw"]) for x in candidates if x["converged"]=="1"]
        require(int(row["terminal_clusters_all"])==complete_link(all_poses) and
                int(row["terminal_clusters_converged"])==complete_link(converged),"healthy cluster audit failed")
        require(int(row["left_nominal"])==int(final[key]["left_nominal"]) and
                int(row["objective_switched"])==int(final[key]["objective_switched"]) and
                int(row["iteration_limit_count"])==sum(int(x["iterations"])>=80 for x in candidates),
                "healthy selection/status mismatch")
    for summary in result["healthy"]:
        rows=[x for x in healthy_rows if (x["method"],int(x["random_rep"]),int(x["budget"]))==
              (summary["method"],summary["random_rep"],summary["budget"])]
        require(len(rows)==23 and summary["objective_switch_count"]==sum(int(x["objective_switched"]) for x in rows) and
                summary["left_nominal_count"]==sum(int(x["left_nominal"]) for x in rows) and
                abs(summary["cluster_count_mean"]-np.mean([int(x["terminal_clusters_all"]) for x in rows]))<1e-12,
                "healthy CSV/JSON summary mismatch")
    historical_runs=read_csv(H2/"ndt_runs.csv")
    old_hits={(int(x["frame"]),x["method"],int(x["seed_index"])):admitted(x,targets)[0]
              for x in historical_runs if x["method"] in ("FULL6D","WEAK2")}
    old_curves={m:np.zeros((100,9,16)) for m in ("FULL6D","WEAK2")}
    for permutation,row in enumerate(read_csv(H2/"permutation_manifest.csv")):
        order=vector(row["seed_indices"],263).astype(int)
        for method,curves in old_curves.items():
            for f,tx in enumerate(MAJOR):
                found=set()
                for b,seed in enumerate(order[:16],1):
                    hit=old_hits[tx,method,int(seed)]
                    if hit!="NONE":found.add(hit)
                    curves[permutation,f,b-1]=len(found)/len(targets[tx])
    for method,curves in old_curves.items():
        expected_auc=float(np.median(np.trapz(curves,np.log2(PREFIX),axis=2)/4.,axis=0).mean())
        require(abs(expected_auc-result["historical"][method]["auc_common_1_16"])<1e-12,"historical AUC mismatch")
    old_frame=np.median(old_curves["WEAK2"],axis=0)
    for decision in result["decisions"]:
        b=decision["budget"];weak=np.array([len(recovered[tx,"COND_WEAK2",-1,b])/len(targets[tx]) for tx in MAJOR])
        primary=sum(cluster in recovered[tx,"COND_WEAK2",-1,b] for tx,cluster in MISSES[:3])
        count=int(np.sum(weak>=old_frame[:,b-1]-1e-12))
        expected=dict(A=bool(weak.mean()>=.70),B=bool(aucs["COND_WEAK2",-1].mean()>aucs["COND_STRONG2",-1].mean()),
            C=bool(aucs["COND_WEAK2",-1].mean()>np.median(random,axis=0).mean()),D=count>=7,E=primary>=2,
            F=bool(weak.mean()>=.70 and b<=16 and result["historical"]["matched_recall_cost70_FULL6D"]["median"]==32))
        require(expected==decision["gates"] and all(expected.values())==decision["pass_gate"] and
                primary==decision["primary_recovered"] and count==decision["frames_at_least_old_weak"],"raw-data decision gate mismatch")
    supported=next((x["budget"] for x in result["decisions"] if x["pass_gate"]),None)
    require(supported==result["best_budget"],"earliest supported budget mismatch")


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("stage",choices=("analyze","audit","self-test"));args=parser.parse_args()
    if args.stage=="analyze":analyze()
    elif args.stage=="audit":audit()
    else:self_test()
