#!/usr/bin/env python3
"""Post-hoc, read-only terminal attribution using the unchanged R2B contract."""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import p9_r4_contract as c
import p9_r2b_nonoracle_evidence as online

START = "06ff017831d2654a62c72f854926b9d3b459ce40"
OUT = c.ROOT / "docs/p9_r5_candidate_bottleneck"
LIB = Path("/tmp/p9_r4_release.Eirto1/libp9_r2b_chart.so")
DISAGREE = (2932,3813,3929,2398,1944,3783,2374,184,655,3200,2675,2889,3899,2483,4110,542,3829)
CONTROLS = (3147,3951,167)
INPUTS = ("artifact_hashes.json", "results.json", "heldout_final_cohort.csv",
    "oracle/oracle_candidates.csv", "oracle/oracle_clusters.csv", "oracle/oracle_labels.csv",
    "candidate/conditioned_weak_runs.csv", "candidate/terminal_clusters.csv",
    "candidate/lidar_nonoracle_evidence.csv", "evaluation/candidate_per_frame.csv",
    "evaluation/candidate_admission.csv", "source_recovery/heldout_source_manifest_recovered.csv",
    "lidar_evidence_freeze.json", "cohort_selection_freeze.json")


def load_inputs():
    hashes = {name: c.pinned(c.OUT/name, START) for name in INPUTS}
    old = json.loads((c.OUT/"artifact_hashes.json").read_text())["artifact_sha256"]
    for name in INPUTS:
        if name != "artifact_hashes.json":
            c.require(old[name] == hashes[name], "R4 hash chain mismatch: " + name)
    code = {name: c.pinned(c.HERE/name, START) for name in
            ("p9_r2b_nonoracle_evidence.py", "p9_r4_oracle_labels.py", "p9_r2b_chart.cpp")}
    receipt = json.loads((c.OUT/"lidar_evidence_freeze.json").read_text())
    c.require(c.digest(LIB) == receipt["chart_library_sha256"], "frozen chart binary changed")
    rows = {name: c.read_csv(c.OUT/name) for name in INPUTS if name.endswith(".csv")}
    ids = [int(r["transaction_id"]) for r in rows["heldout_final_cohort.csv"]]
    labels = {int(r["transaction_id"]): r for r in rows["oracle/oracle_labels.csv"]}
    c.require(len(ids) == len(set(ids)) == 96 and set(labels) == set(ids), "cohort identity failed")
    c.require(Counter(r["label"] for r in labels.values()) == {"MAJOR":40,"NO_MAJOR":56}, "labels changed")
    return rows, ids, labels, dict(input_sha256=hashes, frozen_code_sha256=code,
        chart_library_sha256=c.digest(LIB), start_sha=START, gt_loaded=False)


def entries_and_clusters(frame, method, nominal, rows, library):
    adapted = [dict(r, probe_rank=str(int(r["seed_index"])+1)) if method == "FULL263" else dict(r)
               for r in rows]
    clusters, _, evidence = online.evidence_for_frame(frame, len(adapted), nominal,
        adapted, online.chart_function(library))
    nominal_entry = online.entry(0, nominal, float(rows[0]["nominal_ndt_score_sum"]))
    membership = {int(rank): cluster for cluster in clusters for rank in cluster["members_probe_ranks"].split(";")}
    terminals = []
    for row in adapted:
        rank = int(row["probe_rank"])
        point = online.entry(rank, row["terminal_pose_matrix16"], float(row["raw_ndt_score_sum"]),
                             row["terminal_pose_xyz_q_xyzw"], int(row["iterations"]))
        dt, dr = online.separation(nominal_entry["pose"], point["pose"])
        separated = dt > .2 or dr > 2.
        score_ok = point["score"] >= nominal_entry["score"] + online.EPS_SCORE
        cluster = membership.get(rank)
        reasons = []
        if row["converged"] != "1": reasons.append("NONCONVERGED")
        if not separated: reasons.append("TERMINAL_INSIDE_CENTER")
        if not score_ok: reasons.append("TERMINAL_SCORE_REJECTED")
        if cluster:
            if cluster["is_nominal"]: reasons.append("NOMINAL_CLUSTER")
            if not cluster["representative_outside_nominal_ball"]: reasons.append("REPRESENTATIVE_INSIDE_CENTER")
            if cluster["raw_score"] < nominal_entry["score"]+online.EPS_SCORE: reasons.append("REPRESENTATIVE_SCORE_REJECTED")
        terminals.append(dict(frame=frame, method=method, probe_rank=rank, seed_index=int(row["seed_index"]),
            converged=int(row["converged"]), iterations=int(row["iterations"]),
            terminal_pose_matrix16=row["terminal_pose_matrix16"], terminal_pose_xyz_q_xyzw=row["terminal_pose_xyz_q_xyzw"],
            translation_m=dt, rotation_deg=dr, strict_separated=int(separated), raw_score=point["score"],
            nominal_score=nominal_entry["score"], score_difference=point["score"]-nominal_entry["score"],
            score_pass=int(score_ok), cluster_id=cluster["cluster_id"] if cluster else "NOT_CLUSTERED",
            members_probe_ranks=cluster["members_probe_ranks"] if cluster else "",
            representative_rank=cluster["representative_rank"] if cluster else "",
            representative_translation_m=cluster["translation_from_nominal_m"] if cluster else "",
            representative_rotation_deg=cluster["rotation_from_nominal_deg"] if cluster else "",
            representative_score_difference=cluster["score_advantage"] if cluster else "",
            cluster_strict_eligible=int(bool(cluster and cluster["competitive"] and cluster["representative_outside_nominal_ball"])),
            rejection_reasons=";".join(reasons) or "ELIGIBLE"))
    return clusters, evidence, terminals


def funnel(clusters, terminals):
    valid = [r for r in terminals if r["converged"]]
    separated = [r for r in valid if r["strict_separated"]]
    raw_pass = [r for r in separated if r["score_pass"]]
    nonnominal = [r for r in clusters if not r["is_nominal"]]
    inside = [r for r in nonnominal if not r["representative_outside_nominal_ball"]]
    strict = [r for r in nonnominal if r["representative_outside_nominal_ball"]]
    eligible = [r for r in strict if r["competitive"]]
    cause = ("ELIGIBLE" if eligible else "REPRESENTATIVE_SUPPRESSION" if raw_pass else
             "SCORE_REJECTION" if separated else "INSIDE_CENTER_ONLY" if nonnominal else "NO_NONNOMINAL_CLUSTER")
    return dict(converged_terminals=len(valid), raw_strict_separated=len(separated),
        raw_strict_score_rejected=len(separated)-len(raw_pass), raw_strict_score_pass=len(raw_pass),
        nonnominal_clusters=len(nonnominal), inside_center_representatives=len(inside),
        strict_representatives=len(strict), strict_representatives_score_rejected=len(strict)-len(eligible),
        strict_candidate_count=len(eligible),
        raw_eligible_suppressed=sum(not r["cluster_strict_eligible"] for r in raw_pass),
        N0=int(not nonnominal), N1=int(bool(inside)), N2=int(len(separated)>len(raw_pass)),
        N3_raw=int(bool(raw_pass)), N3_after_clustering=int(bool(eligible)), primary_cause=cause)


def self_test():
    matrix = lambda x: online.text(np.array([[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]],float))
    def row(j,x,s,converged="1"):
        return dict(probe_rank=str(j+1), seed_index=str(j), source_points="10", nominal_ndt_score_sum="100",
            converged=converged, terminal_pose_matrix16=matrix(x), terminal_pose_xyz_q_xyzw=f"{x};0;0;0;0;0;1",
            raw_ndt_score_sum=str(s), iterations="80")
    rows=[row(0,.5,100+online.EPS_SCORE),row(1,1,110,"0"),row(2,-.5,99)]
    cs,_,ts=entries_and_clusters(0,"FULL263",matrix(0),rows,LIB)
    f=funnel(cs,ts)
    c.require(f["strict_candidate_count"]==1 and f["raw_strict_score_rejected"]==1 and
              f["converged_terminals"]==2 and ts[0]["iterations"]==80, "admission boundary/status regression")
    # Nearby high-score inside-center representative can suppress a raw separated candidate.
    rows=[row(0,-.05,100),row(1,.19,102),row(2,.21,101)]
    cs,_,ts=entries_and_clusters(0,"FULL263",matrix(0),rows,LIB)
    c.require(funnel(cs,ts)["primary_cause"]=="REPRESENTATIVE_SUPPRESSION", "representative suppression hidden")
    c.require(ts[2]["strict_separated"] and ts[2]["score_pass"] and not ts[2]["cluster_strict_eligible"],
              "raw and representative eligibility incorrectly conflated")
    rows=[row(j,1,101,"0") for j in range(263)]
    rows[-1].update(converged="1")
    cs,_,ts=entries_and_clusters(0,"FULL263",matrix(0),rows,LIB)
    c.require(len(ts)==263 and ts[-1]["probe_rank"]==263 and
              funnel(cs,ts)["strict_candidate_count"]==1, "FULL263 silently truncated to B12")
    print("P9_R5_FULL263_ADMISSION_SELF_TEST=PASS")


def analyze_frame(job):
    tx, label, nominal, weak, full = job
    results = {}
    for method, rows in (("B12",weak),("FULL263",full)):
        cs, evidence, ts = entries_and_clusters(tx, method, nominal, rows, LIB)
        results[method] = dict(clusters=cs, terminals=ts,
            summary=dict(frame=tx,label=label,method=method,**funnel(cs,ts)))
    return tx, results


def group(rows, field="frame"):
    result=defaultdict(list)
    for row in rows: result[int(row[field])].append(row)
    return result


def oracle_comparison(raw, ids, labels, weak):
    groups=group(raw,"transaction_id")
    output=[]
    for tx in ids:
        if labels[tx]["label"]!="MAJOR": continue
        nominal=online.entry(0,weak[tx][0]["nominal_pose_matrix16"],float(weak[tx][0]["nominal_ndt_score_sum"]))
        source=int(weak[tx][0]["source_points"])
        best=max(float(r["best_score"]) for r in groups[tx] if r["stable_mode_candidate"]=="1")
        for row in groups[tx]:
            if row["is_major"]!="1": continue
            score=float(row["best_score"])
            point=online.entry(1,row["representative_pose_matrix16"],score,row["representative_pose_xyz_q_xyzw"])
            dt,dr=online.separation(nominal["pose"],point["pose"])
            ds=score-nominal["score"]
            output.append(dict(frame=tx,cluster_id=row["cluster_id"],source_points=source,
                major_center_pose=row["representative_pose_matrix16"],score=score,nominal_score=nominal["score"],
                signed_score_difference=ds,score_difference_per_source=ds/source,
                exact_score_sign="POSITIVE" if ds>0 else "NEGATIVE" if ds<0 else "ZERO",
                carrier_score_class="POSITIVE" if ds>online.EPS_SCORE else "NEGATIVE" if ds<-online.EPS_SCORE else "NEAR",
                EPS_SCORE=online.EPS_SCORE,online_score_pass=int(score>=nominal["score"]+online.EPS_SCORE),
                translation_m=dt,rotation_deg=dr,online_center_separated=int(dt>.2 or dr>2),
                online_geometry_and_score=int((dt>.2 or dr>2) and score>=nominal["score"]+online.EPS_SCORE),
                best_supported_score=best,oracle_gap_per_source=(best-score)/source,
                oracle_gap_limit=.05*max(1.,abs(best/source)),seed_count=row["seed_count"],
                stable_mode_candidate=row["stable_mode_candidate"]))
    c.require(len(output)==84,"major ID count drift")
    return output


def run():
    tick=time.perf_counter()
    c.require(not (OUT/"analysis_freeze.json").exists(),"R5 analysis already frozen")
    raw,ids,labels,receipt=load_inputs()
    weak=group(raw["candidate/conditioned_weak_runs.csv"])
    full=group(raw["oracle/oracle_candidates.csv"])
    sources={int(r["transaction_id"]):r for r in raw["source_recovery/heldout_source_manifest_recovered.csv"]}
    audits=[];jobs=[]
    for tx in ids:
        weak[tx].sort(key=lambda r:int(r["probe_rank"]))
        full[tx].sort(key=lambda r:int(r["seed_index"]))
        c.require(len(weak[tx])==12 and len(full[tx])==263,"terminal pool incomplete")
        c.require([int(r["probe_rank"]) for r in weak[tx]]==list(range(1,13)) and
                  [int(r["seed_index"]) for r in full[tx]]==list(range(263)),"terminal order invalid")
        s0=weak[tx][0]["nominal_ndt_score_sum"]
        nominal=weak[tx][0]["nominal_pose_matrix16"]
        c.require(all(r["nominal_pose_matrix16"]==nominal for r in weak[tx]),"T0 changed")
        for r in weak[tx]+full[tx]:
            c.require(r["source_hash_actual"]==r["source_hash_expected"]==sources[tx]["prepared_source_hash"] and
                r["source_points"]==sources[tx]["prepared_source_point_count"] and
                r["target_points"]==sources[tx]["target_point_count"] and r["nominal_ndt_score_sum"]==s0 and
                r["converged"] in ("0","1"),"source/S0/convergence contract mismatch")
        audits.append(dict(frame=tx,source_points=sources[tx]["prepared_source_point_count"],
            source_hash=sources[tx]["prepared_source_hash"],nominal_score=s0,nominal_pose_matrix16=nominal,
            B12_rows=12,FULL263_rows=263,parity="PASS"))
        jobs.append((tx,labels[tx]["label"],nominal,weak[tx],full[tx]))
    receipt.update(code_sha256=c.digest(__file__),theory_sha256=c.digest(OUT/"THEORY.md"),
        workers=4,NEW_NDT_CALLS=0,BASELINE_REPLAY_CALLS=0,VISUAL_EXTRACTION=0)
    c.save_json(OUT/"input_manifest.json",receipt)
    c.write_csv(OUT/"input_audit.csv",audits)
    results={}
    with ProcessPoolExecutor(max_workers=4) as pool:
        futures=[pool.submit(analyze_frame,job) for job in jobs]
        for future in as_completed(futures):
            tx,value=future.result();results[tx]=value
            print("R5_OFFLINE_FRAME",tx,"B12",value["B12"]["summary"]["strict_candidate_count"],
                "FULL263",value["FULL263"]["summary"]["strict_candidate_count"],len(results),"/96",flush=True)
    saved_clusters=group(raw["candidate/terminal_clusters.csv"])
    saved_features={int(r["frame"]):r for r in raw["candidate/lidar_nonoracle_evidence.csv"]}
    saved_frame={int(r["frame"]):r for r in raw["evaluation/candidate_per_frame.csv"]}
    parity=[]
    for tx in ids:
        actual=results[tx]["B12"]
        c.require([{k:str(v) for k,v in row.items()} for row in actual["clusters"]]==saved_clusters[tx],
                  "R4 B12 full cluster contract parity failed")
        count=actual["summary"]["strict_candidate_count"]
        c.require(count==int(saved_features[tx]["strict_candidate_count"])==int(saved_frame[tx]["strict_candidate_count"]),
                  "R4 B12 strict count parity failed")
        parity.append(dict(frame=tx,cluster_count=len(actual["clusters"]),strict_candidate_count=count,parity="PASS"))
    b12=[results[tx]["B12"]["summary"] for tx in ids]
    full263=[results[tx]["FULL263"]["summary"] for tx in ids]
    c.write_csv(OUT/"candidate_contract_parity.csv",parity)
    c.write_csv(OUT/"b12_failure_funnel.csv",[r for r in b12 if r["label"]=="MAJOR"])
    c.write_csv(OUT/"full263_strict_candidate.csv",full263)
    terminals=[r for tx in ids for method in ("B12","FULL263") for r in results[tx][method]["terminals"]]
    c.write_csv(OUT/"terminal_diagnostics.csv",terminals)
    full_clusters=[r for tx in ids for r in results[tx]["FULL263"]["clusters"]]
    c.write_csv(OUT/"full263_terminal_clusters.csv",full_clusters)
    comparison=oracle_comparison(raw["oracle/oracle_clusters.csv"],ids,labels,weak)
    c.write_csv(OUT/"oracle_online_score_comparison.csv",comparison)
    admissions={(int(r["frame"]),int(r["probe_rank"])):r for r in raw["evaluation/candidate_admission.csv"]}
    disagreement=[]
    for tx in DISAGREE:
        c.require(int(saved_frame[tx]["recovered_count"])>0 and int(saved_frame[tx]["strict_candidate_count"])==0,
                  "specified ID disagreement frame does not match R4")
        for terminal in results[tx]["B12"]["terminals"]:
            admission=admissions[tx,terminal["probe_rank"]]
            if admission["assigned_id"]!="NONE":
                disagreement.append(dict(terminal,recovered_major_id=admission["assigned_id"],
                    eligible_major_ids=admission["eligible_ids"],eligible_major_count=int(admission["eligible_count"])))
    c.require(set(r["frame"] for r in disagreement)==set(DISAGREE),"missing disagreement detail")
    c.write_csv(OUT/"id_recovery_disagreement.csv",disagreement)
    controls=[]
    for tx in ids:
        if labels[tx]["label"]!="NO_MAJOR":continue
        controls.append(dict(frame=tx,label="NO_MAJOR",B12_strict_candidate_count=results[tx]["B12"]["summary"]["strict_candidate_count"],
            FULL263_strict_candidate_count=results[tx]["FULL263"]["summary"]["strict_candidate_count"],
            B12_primary_cause=results[tx]["B12"]["summary"]["primary_cause"]))
    c.require({r["frame"] for r in controls if r["B12_strict_candidate_count"]>0}==set(CONTROLS),"NO_MAJOR controls changed")
    c.write_csv(OUT/"no_major_controls.csv",controls)
    c.write_csv(OUT/"no_major_positive_details.csv",[r for r in terminals if r["frame"] in CONTROLS])
    counts=[]
    for name,table in (("B12",b12),("FULL263",full263)):
        for label in ("MAJOR","NO_MAJOR"):
            selected=[r for r in table if r["label"]==label]
            counts.append(dict(method=name,label=label,frames=len(selected),
                strict_positive=sum(r["strict_candidate_count"]>0 for r in selected),
                raw_strict_positive=sum(r["raw_strict_separated"]>0 for r in selected),
                raw_strict_score_positive=sum(r["raw_strict_score_pass"]>0 for r in selected),
                **{reason:sum(r["primary_cause"]==reason for r in selected) for reason in
                    ("NO_NONNOMINAL_CLUSTER","INSIDE_CENTER_ONLY","SCORE_REJECTION","REPRESENTATIVE_SUPPRESSION","ELIGIBLE")}))
    c.write_csv(OUT/"summary_statistics.csv",counts)
    c.require(c.digest(__file__)==receipt["code_sha256"],"analysis source changed while running")
    c.save_json(OUT/"analysis_freeze.json",dict(analysis_type="POSTHOC_FAILURE_ATTRIBUTION",start_sha=START,
        offline_wall_seconds=time.perf_counter()-tick,NEW_NDT_CALLS=0,BASELINE_REPLAY_CALLS=0,
        VISUAL_EXTRACTION=0,GT_LOADED=False,artifacts={p.name:c.digest(p) for p in OUT.glob("*.csv")},
        input_manifest_sha256=c.digest(OUT/"input_manifest.json"),code_sha256=c.digest(__file__),
        theory_sha256=c.digest(OUT/"THEORY.md")))
    print(json.dumps(counts,indent=2),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",choices=("self-test","run"))
    args=parser.parse_args()
    self_test() if args.action=="self-test" else run()
