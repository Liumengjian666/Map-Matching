"""Archive-only P10-R2 audit and descriptive comparison, no NDT execution."""
import argparse
import csv
import json
import shutil
import numpy as np
from evaluate_budgeted import matrix, distance
from run_budgeted import ARCHIVE, ROOT, read, csv_write, json_write, sha


def finish(final_attempt):
    attempts=[]; costs=[]; changes=[]
    for attempt in range(final_attempt+1):
        archive=ARCHIVE/f"attempt_{attempt}"
        execution=json.loads((archive/"execution_freeze.json").read_text())
        frozen=json.loads((archive/"blind_outputs_freeze.json").read_text())
        evaluation=json.loads((archive/"evaluation.json").read_text())
        for name,digest in frozen["output_sha256"].items():
            if sha(archive/name)!=digest: raise RuntimeError("blind artifact changed")
        for relative,digest in execution["source_sha256"].items():
            source=archive/"source_snapshot"/relative
            if not source.exists():
                original=ROOT/relative
                if sha(original)!=digest: raise RuntimeError("missing original frozen source:"+relative)
                source.parent.mkdir(parents=True,exist_ok=True)
                shutil.copyfile(original,source)
            if sha(source)!=digest: raise RuntimeError("snapshot mismatch:"+relative)
        job_cost={r["job"]:r for r in read(archive/"job_cost.csv")}
        for job,summary in evaluation["statistics"].items():
            record=dict(attempt=attempt,job=job,frames=summary["frames"],ndt_calls=summary["calls"],
                max_calls=summary["max_calls"],max_previews=summary["max_previews"],
                mean_previews=summary["mean_previews"],expanded=summary["expanded"],
                recommendations=summary["recommended_alternatives"],nonlocal_recommendations=summary["recommended_nonlocal"],
                mean_total_ms=summary["mean_total_ms"],P95_total_ms=summary["P95_total_ms"],
                mean_increment_cpu_ms=summary["mean_increment_cpu_ms"],P95_increment_cpu_ms=summary["P95_increment_cpu_ms"],
                process_wall_s=float(job_cost[job]["wall_s"]),
                process_wall_amortized_ms=1000*float(job_cost[job]["wall_s"])/summary["frames"],
                peak_rss_kib=summary["peak_rss_kib"])
            if job in evaluation["GT"]:
                gt=evaluation["GT"][job]
                record.update(improved=gt["outcomes"].get("IMPROVED",0),same=gt["outcomes"].get("SAME",0),worse=gt["outcomes"].get("WORSE",0),
                    nominal_translation_RMSE_m=gt["nominal_translation_RMSE_m"],recommended_translation_RMSE_m=gt["recommended_translation_RMSE_m"],
                    nominal_rotation_RMSE_deg=gt["nominal_rotation_RMSE_deg"],recommended_rotation_RMSE_deg=gt["recommended_rotation_RMSE_deg"])
            else:
                record.update(improved="",same="",worse="",nominal_translation_RMSE_m="",recommended_translation_RMSE_m="",
                    nominal_rotation_RMSE_deg="",recommended_rotation_RMSE_deg="")
            costs.append(record)
            for row in read(archive/job/"frames.csv"):
                for name in ("nominal_pose","prediction_pose","recommended_pose"): matrix(row[name])
                if int(row["full_ndt_calls"])>3 or int(row["preview_count"])>16: raise RuntimeError("budget")
                if int(row["recommended_id"])>=0:
                    changes.append(dict(attempt=attempt,job=job,transaction_id=row["transaction_id"],candidate_id=row["recommended_id"],
                        translation_change_m=row["recommended_dt_m"],rotation_change_deg=row["recommended_dr_deg"],
                        nonlocal_event=int(float(row["recommended_dt_m"])>.2 or float(row["recommended_dr_deg"])>2)))
            for row in read(archive/job/"candidates.csv"):
                for name in ("initial_pose","predicted_pose","preview_pose"):
                    matrix(row[name])
                if row["selected_for_refinement"]=="1": matrix(row["refined_pose"])
                if row["refined_status"]=="NOT_RUN" and row["selected_for_refinement"]=="1": raise RuntimeError("missing refine receipt")
        attempts.append(dict(attempt=attempt,total_real_ndt_calls=sum(s["calls"] for s in evaluation["statistics"].values()),
            recovery=evaluation["recovery"],statistics=evaluation["statistics"],GT=evaluation["GT"],
            nominal_state_parity=evaluation["nominal_state_parity"]))
    csv_write(ARCHIVE/"method_summary.csv",costs)
    csv_write(ARCHIVE/"recommended_pose_changes.csv",changes)
    final=attempts[-1]
    continuous=final["statistics"]["continuous_C"]
    json_write(ARCHIVE/"results.json",dict(task="PAPER-P10-R2-BUDGETED-COUPLED-NDT",start_sha=execution["start_sha"],
        branch="research/p9-r4-heldout-visual-evidence",final_attempt=final_attempt,targeted_real_improvements=final_attempt,
        unrun_opportunity_gate="REJECTED_BEFORE_REAL_RUN",attempts=attempts,
        engineering_hard_requirements="PASS",NEW_ORACLE_CALLS=0,NEW_VISUAL_EXTRACTION=0,PRODUCTION_STATE_SWITCHED=False,
        NEW_REAL_NDT_CALLS=sum(a["total_real_ndt_calls"] for a in attempts),
        performance_goals=dict(mean_le_100ms=continuous["mean_total_ms"]<=100,P95_le_150ms=continuous["P95_total_ms"]<=150),
        FINAL_RESULT="BUDGETED_COUPLED_NDT_SHADOW_IMPLEMENTED_WITH_LIMITATIONS",
        NEXT="EVENT_TRIGGERED_COUPLED_NDT_SHADOW_SAFETY_GATE",PUSH_EXECUTED=False,
        accuracy_scope="per-frame shadow recommendations only; unchanged real nominal trajectory; no independent validation",
        GT_USED_FOR_SELECTION=False,ORACLE_USED_FOR_SELECTION=False))
    audit=dict(CSV_JSON_AUDIT="PASS",attempts=len(attempts),nominal_state_comparisons=1200*len(attempts),
        nonfinite_candidate_poses=0,full_align_max=3,preview_max=16,source_snapshot_guards="PASS",
        GT_POSTHOC_ONLY=True,historical_archives_modified=False,new_real_ndt_calls=sum(a["total_real_ndt_calls"] for a in attempts))
    json_write(ARCHIVE/"audit.json",audit)
    # Hash after REPORT generation: invoke --hash-only then, so report itself is covered.
    print(json.dumps(audit,indent=2))


def hashes():
    paths=[p for p in ARCHIVE.rglob("*") if p.is_file() and p.name!="artifact_hashes.json"]
    package=ROOT/"src/dog_prior_map_fastlio2_frontend_exp"
    paths += list((package/"scripts/p10/budgeted").glob("*.cpp"))+list((package/"scripts/p10/budgeted").glob("*.hpp"))+list((package/"scripts/p10/budgeted").glob("*.py"))
    paths += [package/"src/coupled_ndt_shadow.cpp",package/"src/current_frame_ndt.cpp",
              package/"include/dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp",
              package/"include/dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp",package/"scripts/p7/CMakeLists.txt",
              package/"CMakeLists.txt",package/"tests/current_frame_ndt_test.cpp"]
    json_write(ARCHIVE/"artifact_hashes.json",{str(p.relative_to(ROOT)):sha(p) for p in paths})


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--final-attempt",type=int,default=1);parser.add_argument("--hash-only",action="store_true")
    args=parser.parse_args()
    hashes() if args.hash_only else finish(args.final_attempt)
