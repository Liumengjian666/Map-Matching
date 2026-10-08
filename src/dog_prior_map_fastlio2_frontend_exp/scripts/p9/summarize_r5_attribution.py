#!/usr/bin/env python3
"""Summarize frozen post-hoc R5 diagnostics; no optimizer, image or GT reader."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import p9_r4_contract as c
from p9_r5_attribution import OUT, START, DISAGREE, CONTROLS, LIB


def load_verified():
    receipt=json.loads((OUT/"analysis_freeze.json").read_text())
    c.require(receipt["start_sha"]==START and not receipt["GT_LOADED"] and
              receipt["NEW_NDT_CALLS"]==receipt["BASELINE_REPLAY_CALLS"]==receipt["VISUAL_EXTRACTION"]==0,
              "analysis scope mismatch")
    for name,sha in receipt["artifacts"].items():
        c.require(c.digest(OUT/name)==sha,"frozen diagnostic changed: "+name)
    c.require(c.digest(c.HERE/"p9_r5_attribution.py")==receipt["code_sha256"] and
        c.digest(OUT/"THEORY.md")==receipt["theory_sha256"] and
        c.digest(OUT/"input_manifest.json")==receipt["input_manifest_sha256"],"analysis freeze chain failed")
    manifest=json.loads((OUT/"input_manifest.json").read_text())
    for name,sha in manifest["input_sha256"].items():
        c.require(c.digest(c.OUT/name)==sha==c.pinned(c.OUT/name,START),"R4 input changed")
    for name,sha in manifest["frozen_code_sha256"].items():
        c.require(c.digest(c.HERE/name)==sha==c.pinned(c.HERE/name,START),"frozen helper changed")
    c.require(c.digest(LIB)==manifest["chart_library_sha256"],"frozen chart library changed")
    return receipt


def derive():
    receipt=load_verified()
    full=c.read_csv(OUT/"full263_strict_candidate.csv")
    b12=c.read_csv(OUT/"b12_failure_funnel.csv")
    controls=c.read_csv(OUT/"no_major_controls.csv")
    comparison=c.read_csv(OUT/"oracle_online_score_comparison.csv")
    disagreements=c.read_csv(OUT/"id_recovery_disagreement.csv")
    terminals=c.read_csv(OUT/"terminal_diagnostics.csv")
    major={int(r["frame"]) for r in b12}
    bp={int(r["frame"]) for r in b12 if int(r["strict_candidate_count"])>0}
    fp={int(r["frame"]) for r in full if r["label"]=="MAJOR" and int(r["strict_candidate_count"])>0}
    original=c.read_csv(c.OUT/"evaluation/candidate_per_frame.csv")
    id_hit={int(r["frame"]) for r in original if r["label"]=="MAJOR" and int(r["recovered_count"])>0}
    cross=dict(ID_and_strict=len(id_hit&bp),ID_only=len(id_hit-bp),strict_only=len(bp-id_hit),
               neither=len(major-id_hit-bp))
    c.require(cross==dict(ID_and_strict=12,ID_only=17,strict_only=5,neither=6),"R4 cross-table changed")
    c.require(len(major)==40 and len(bp)==17 and len(controls)==56 and len(full)==96,
              "frame counts or R4 outcome changed")
    c.require({int(r["frame"]) for r in controls if int(r["B12_strict_candidate_count"])>0}==set(CONTROLS),
              "NO_MAJOR B12 controls changed")
    reasons=Counter(r["primary_cause"] for r in b12)
    score_positive={int(r["frame"]) for r in comparison if r["online_score_pass"]=="1"}
    score_rejected={int(r["frame"]) for r in comparison if r["online_score_pass"]=="0"}
    grouped=defaultdict(list)
    for r in disagreements:grouped[int(r["frame"])].append(r)
    frame_details=[]
    for tx in DISAGREE:
        rows=grouped[tx]
        frame_details.append(dict(frame=tx,recovered_ids=";".join(sorted({r["recovered_major_id"] for r in rows})),
            recovered_terminal_count=len(rows),actual_inside=sum(r["strict_separated"]=="0" for r in rows),
            actual_score_rejected=sum(r["score_pass"]=="0" for r in rows),
            actual_separated_score_rejected=sum(r["strict_separated"]=="1" and r["score_pass"]=="0" for r in rows),
            actual_separated_score_pass=sum(r["strict_separated"]==r["score_pass"]=="1" for r in rows),
            nominal_cluster=sum(r["cluster_id"]=="NOMINAL" for r in rows),
            representative_inside=sum(float(r["representative_translation_m"])<=.2 and
                float(r["representative_rotation_deg"])<=2 for r in rows),
            representative_score_rejected=sum("REPRESENTATIVE_SCORE_REJECTED" in r["rejection_reasons"].split(";") for r in rows),
            frame_primary_cause=next(r["primary_cause"] for r in b12 if int(r["frame"])==tx)))
    valid=[r for r in terminals if r["method"]=="B12" and int(r["frame"]) in major and r["converged"]=="1"]
    raw_counts=dict(terminals=len(valid),strict_separated=sum(int(r["strict_separated"]) for r in valid),
        separated_score_rejected=sum(r["strict_separated"]=="1" and r["score_pass"]=="0" for r in valid),
        separated_score_pass=sum(r["strict_separated"]==r["score_pass"]=="1" for r in valid),
        eligible_raw_suppressed=sum(r["strict_separated"]==r["score_pass"]=="1" and
                                  r["cluster_strict_eligible"]=="0" for r in valid))
    recovered_counts=dict(terminals=len(disagreements),
        inside=sum(r["strict_separated"]=="0" for r in disagreements),
        score_rejected=sum(r["score_pass"]=="0" for r in disagreements),
        inside_and_score_rejected=sum(r["strict_separated"]==r["score_pass"]=="0" for r in disagreements),
        separated_score_rejected=sum(r["strict_separated"]=="1" and r["score_pass"]=="0" for r in disagreements),
        separated_score_pass=sum(r["strict_separated"]==r["score_pass"]=="1" for r in disagreements))
    result=dict(task="PAPER-P9-R5-NONLOCAL-CANDIDATE-BOTTLENECK-ATTRIBUTION",
        analysis_type="POSTHOC FAILURE ATTRIBUTION",branch=c.BRANCH,start_sha=START,worktree=str(c.ROOT),
        NEW_NDT_CALLS=0,BASELINE_REPLAY_CALLS=0,VISUAL_EXTRACTION=0,GT_LOADED=False,PUSH_EXECUTED=False,
        R4_FINAL_RESULT="HELDOUT_LIDAR_CANDIDATE_GENERATOR_NOT_GENERALIZED",
        R4_strict_major_coverage="17/40",offline_analysis_wall_seconds=receipt["offline_wall_seconds"],
        cost_scope="OFFLINE_POSTHOC_ANALYSIS_ONLY; no machine-dog online runtime inference",
        FULL263=dict(MAJOR_strict_positive=len(fp),MAJOR_frames=40,
            NO_MAJOR_strict_positive=sum(int(r["FULL263_strict_candidate_count"])>0 for r in controls),NO_MAJOR_frames=56),
        B12=dict(MAJOR_strict_positive=17,MAJOR_frames=40,NO_MAJOR_strict_positive=3,NO_MAJOR_frames=56),
        id_recovery_cross_table=cross,
        failure_breakdown=dict(reasons),raw_terminal_counts=raw_counts,
        FULL263_failure_breakdown=dict(Counter(r["primary_cause"] for r in full if r["label"]=="MAJOR")),
        disagreement_recovered_terminal_counts=recovered_counts,
        raw_eligible_suppression_examples=[{key:v[key] for key in ("frame","probe_rank","seed_index",
            "rotation_deg","score_difference","representative_rank","representative_rotation_deg")}
            for v in valid if v["strict_separated"]==v["score_pass"]=="1" and v["cluster_strict_eligible"]=="0"],
        no_major_positive_controls=[r for r in controls if int(r["frame"]) in CONTROLS],
        overlapping_frame_flags={key:sum(int(r[key]) for r in b12) for key in ("N0","N1","N2","N3_raw","N3_after_clustering")},
        pool_comparison=dict(both=sorted(bp&fp),B12_only=sorted(bp-fp),FULL263_only=sorted(fp-bp),
            neither=sorted(major-bp-fp),B12_failed_frames=len(major-bp)),
        oracle_vs_online=dict(major_basins=len(comparison),
            carrier_score_classes=dict(Counter(r["carrier_score_class"] for r in comparison)),
            exact_score_signs=dict(Counter(r["exact_score_sign"] for r in comparison)),EPS_SCORE=c.EPS_SCORE,
            center_separated=sum(int(r["online_center_separated"]) for r in comparison),
            frames_with_positive_major=sorted(score_positive),frames_with_rejected_major=sorted(score_rejected),
            frames_all_major_score_rejected=sorted(major-score_positive)),
        disagreement_frame_details=frame_details,
        disagreement_terminal_reason_counts=dict(Counter(reason for r in disagreements for reason in r["rejection_reasons"].split(";"))),
        disagreement_frames_by_primary_cause=dict(Counter(r["frame_primary_cause"] for r in frame_details)))
    # Descriptive mechanisms overlap; this is not a statistical causal decomposition.
    mechanisms=dict(
        finite_pool_search_misses=bool(fp-bp),
        encountered_terminal_eligibility_loss=bool(sum(v for k,v in reasons.items() if k in
            ("SCORE_REJECTION","REPRESENTATIVE_SUPPRESSION","INSIDE_CENTER_ONLY"))),
        oracle_online_definition_mismatch=bool(score_rejected))
    result["mechanisms_observed"]=mechanisms
    if sum(mechanisms.values())>=2:
        result.update(FINAL_RESULT="MIXED_CANDIDATE_BOTTLENECK",NEXT="REASSESS_NONLOCAL_AMBIGUITY_ADMISSION_CONTRACT")
    elif mechanisms["finite_pool_search_misses"]:
        result.update(FINAL_RESULT="SEARCH_REACHABILITY_BOTTLENECK",NEXT="REASSESS_NONLOCAL_CANDIDATE_GENERATOR")
    elif mechanisms["oracle_online_definition_mismatch"]:
        result.update(FINAL_RESULT="ORACLE_ONLINE_DEFINITION_MISMATCH",NEXT="REASSESS_NONLOCAL_AMBIGUITY_ADMISSION_CONTRACT")
    else:
        result.update(FINAL_RESULT="CANDIDATE_ELIGIBILITY_BOTTLENECK",NEXT="REASSESS_NONLOCAL_AMBIGUITY_ADMISSION_CONTRACT")
    return result


def report(r):
    f=r["FULL263"];raw=r["raw_terminal_counts"];o=r["oracle_vs_online"];p=r["pool_comparison"]
    lines=["# R5 post-hoc candidate bottleneck attribution", "",
        "FINAL_RESULT = `"+r["FINAL_RESULT"]+"`", "", "NEXT = `"+r["NEXT"]+"`", "",
        "R4 remains HELDOUT_LIDAR_CANDIDATE_GENERATOR_NOT_GENERALIZED (17/40).",
        "This is POSTHOC FAILURE ATTRIBUTION, not new confirmatory evidence.", "",
        "## Observed coverage and failure mechanisms", "",
        "| Recorded terminal pool | MAJOR strict coverage | NO_MAJOR strict coverage |",
        "| --- | ---: | ---: |", "| B12 | 17/40 | 3/56 |",
        f"| FULL263 | {f['MAJOR_strict_positive']}/40 | {f['NO_MAJOR_strict_positive']}/56 |", "",
        f"B12 and FULL263 both positive: {len(p['both'])}; B12-only: {len(p['B12_only'])}; "
        f"FULL263-only: {len(p['FULL263_only'])}; neither: {len(p['neither'])} MAJOR frames.",
        "Frozen ID-recovery versus strict eligibility cross-table: `"+json.dumps(r["id_recovery_cross_table"],sort_keys=True)+"`.",
        "FULL263 is only the attainable coverage of the existing recorded pool.",
        "The conditioned B12 starts are not a nested subset of FULL263, so this is",
        "not a matched-budget causal efficiency claim or a global reachability bound.", "",
        "FULL263 MAJOR exclusive causes: `"+json.dumps(r["FULL263_failure_breakdown"],sort_keys=True)+"`.", "",
        "Mutually exclusive B12 frame causes:", "",
        "| Cause | MAJOR frames |", "| --- | ---: |"]
    lines += [f"| {name} | {r['failure_breakdown'].get(name,0)} |" for name in
        ("NO_NONNOMINAL_CLUSTER","INSIDE_CENTER_ONLY","SCORE_REJECTION","REPRESENTATIVE_SUPPRESSION","ELIGIBLE")]
    lines += ["",f"Raw B12 MAJOR terminals: {raw['terminals']}; strictly separated: {raw['strict_separated']}; "
        f"strictly separated but score-rejected: {raw['separated_score_rejected']}; "
        f"raw strict+score-pass: {raw['separated_score_pass']}; raw eligible suppressed by representative choice: "
        f"{raw['eligible_raw_suppressed']}.",
        "These terminal counts are not independent statistical samples and are not frame counts.",
        "Overlapping frame flags: `"+json.dumps(r["overlapping_frame_flags"],sort_keys=True)+"`.", "",
        "## Oracle versus online definition", "",
        "Oracle: supported cluster, separated center, finite per-source score gap from the best supported basin.",
        "Online: non-nominal complete-link cluster, strictly separated representative, raw score>=nominal+EPS_SCORE.",
        "EPS_SCORE=2.747604276e-4 remains frozen. No alternate tolerance is evaluated.",
        f"Across {o['major_basins']} major centers: score classes {o['carrier_score_classes']}; "
        f"actual online center separation holds for {o['center_separated']}.",
        f"Frames with some score-rejected major: {len(o['frames_with_rejected_major'])}/40; "
        f"frames with every major center score-rejected: {len(o['frames_all_major_score_rejected'])}/40.",
        "Thus major ID membership and online eligibility are mathematically different events.",
        "If nonlocal ambiguity is intended to include near-optimal alternatives admitted by the oracle,",
        "requiring an objective improvement over T0 expresses a narrower property. This is a semantic",
        "interpretation of the measured definition mismatch, not a claim that a new threshold is valid.", "",
        "## The 17 ID-recovery disagreements", "",
        "Counts below are recovered terminal rows; several rows may recover the same frozen ID.",
        "Rejection columns overlap. Exact poses, score differences, members and representative geometry",
        "are retained for every row in id_recovery_disagreement.csv.", "",
        "| Frame | Recovered IDs | Hit rows | Actual inside | Actual score rejected | Representative inside | Primary frame cause |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- |"]
    lines += [f"| {v['frame']} | {v['recovered_ids']} | {v['recovered_terminal_count']} | {v['actual_inside']} | "
        f"{v['actual_score_rejected']} | {v['representative_inside']} | {v['frame_primary_cause']} |"
        for v in r["disagreement_frame_details"]]
    lines += ["", "Aggregate recovered-terminal rejection reasons: `"+
        json.dumps(r["disagreement_terminal_reason_counts"],sort_keys=True)+"`.", "",
        "Recovered-terminal geometry/score counts: `"+json.dumps(r["disagreement_recovered_terminal_counts"],sort_keys=True)+"`.",
        "Frame cause uses all B12 terminals, not only ID-recovering rows. Raw eligible suppression details:", "",
        "```json",json.dumps(r["raw_eligible_suppression_examples"],indent=2),"```", "",
        "## NO_MAJOR controls and interpretation", "",
        "All 56 controls are listed in no_major_controls.csv. Frames 3147, 3951 and 167 have full",
        "B12 and FULL263 terminal records in no_major_positive_details.csv, including poses, nominal",
        "separation, score difference, cluster membership and rejection/eligibility reason.",
        "NO_MAJOR is the frozen oracle proxy; its strict-positive events are not automatically errors.", "",
        "## Main conclusion and boundaries", "",
        f"In {len(p['FULL263_only'])} MAJOR frames B12 lacks a usable candidate that exists in BASE263; "
        "this supports a finite-pool search shortfall. Separately, the funnel documents eligibility",
        "losses among already reached terminals, and oracle centers can be lower-scoring than T0.",
        "These mechanisms overlap; their counts must not be summed into a causal percentage.",
        "The data do not justify describing the entire 42.5% coverage as a failure to reach any",
        "other terminal, nor as proof that increasing search to 263 universally solves the gate.",
        "The sole proposed next decision is `"+r["NEXT"]+"`.",
        "This report does not implement a new eligibility definition or change R4.", "",
        "## Provenance, validation and cost", "",
        f"Start: {START}; branch: {c.BRANCH}; workspace: {c.ROOT}.",
        "Input manifest pins 14 R4 artifacts to that commit plus the original helper/geometry hashes.",
        "All 96 B12 clusters and strict counts must reproduce their frozen records exactly.",
        "The full263 self-test checks rank263 inclusion, score equality admission, convergence/iteration-limit",
        "handling and a representative-inside suppression example. Required Release/P9 checks and hash",
        "audit are recorded in verification/; the frozen solver/frontend code was not changed.",
        f"Offline analysis wall time: {r['offline_analysis_wall_seconds']:.6f} s (4 processes).",
        "This is offline post-hoc analysis cost, not online robot runtime.",
        "NEW_NDT_CALLS=0; BASELINE_REPLAY_CALLS=0; VISUAL_EXTRACTION=0; GT_LOADED=NO; PUSH_EXECUTED=NO.",
        "The containing Git commit supplies the end SHA, avoiding a self-referential hash.", ""]
    return "\n".join(lines)


def write_outputs():
    r=derive()
    c.write_csv(OUT/"id_disagreement_frame_summary.csv",r["disagreement_frame_details"])
    c.save_json(OUT/"results.json",r)
    (OUT/"REPORT.md").write_text(report(r))
    paths={str(p.relative_to(OUT)):c.digest(p) for p in OUT.rglob("*") if p.is_file() and p.name!="artifact_hashes.json"}
    c.save_json(OUT/"artifact_hashes.json",dict(artifacts=paths,
        sources={str(p.relative_to(c.ROOT)):c.digest(p) for p in
            (Path(__file__).resolve(),c.HERE/"p9_r5_attribution.py",c.HERE/"CMakeLists.txt")}))
    audit()


def audit():
    r=derive();saved=json.loads((OUT/"results.json").read_text())
    c.require(r==saved,"results differ from CSV-derived statistics")
    terminals=c.read_csv(OUT/"terminal_diagnostics.csv")
    c.require(len(terminals)==96*(263+12) and len({(v["method"],v["frame"],v["probe_rank"]) for v in terminals})==len(terminals),
              "terminal inventory incomplete or duplicated")
    parity=c.read_csv(OUT/"candidate_contract_parity.csv")
    c.require(len(parity)==96 and all(v["parity"]=="PASS" for v in parity),"candidate parity incomplete")
    for v in c.read_csv(OUT/"summary_statistics.csv"):
        expected=r[v["method"]]
        c.require(int(v["frames"])==expected[v["label"]+"_frames"] and
            int(v["strict_positive"])==expected[v["label"]+"_strict_positive"],"summary table coverage mismatch")
        causes=("NO_NONNOMINAL_CLUSTER","INSIDE_CENTER_ONLY","SCORE_REJECTION","REPRESENTATIVE_SUPPRESSION","ELIGIBLE")
        c.require(sum(int(v[k]) for k in causes)==int(v["frames"]),"exclusive causes do not partition frames")
    h=json.loads((OUT/"artifact_hashes.json").read_text())
    files={str(p.relative_to(OUT)):p for p in OUT.rglob("*") if p.is_file() and p!=OUT/"artifact_hashes.json"}
    c.require(set(files)==set(h["artifacts"]),"artifact inventory incomplete")
    for name,p in files.items():c.require(c.digest(p)==h["artifacts"][name],"artifact changed: "+name)
    for name,sha in h["sources"].items():c.require(c.digest(c.ROOT/name)==sha,"source changed: "+name)
    c.require((OUT/"REPORT.md").read_text()==report(r),"report differs from results")
    print("P9_R5_CSV_JSON_HASH_AUDIT=PASS",flush=True)
    print(json.dumps({k:r[k] for k in ("FULL263","failure_breakdown","raw_terminal_counts",
        "oracle_vs_online","disagreement_frames_by_primary_cause","FINAL_RESULT","NEXT")},indent=2),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",choices=("write","audit"));args=parser.parse_args()
    write_outputs() if args.action=="write" else audit()
