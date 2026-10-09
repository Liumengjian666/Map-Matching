"""Concise final fact table from immutable evaluated attempts."""
import json
from run_budgeted import read, csv_write, json_write
from run_admission import ARCHIVE
from evaluate_budgeted import matrix, distance


def main():
    rows=[];details={};total_calls=0
    for attempt in (0,1):
        directory=ARCHIVE/f"attempt_{attempt}"
        e=json.loads((directory/"evaluation.json").read_text())
        for name in ("event_admission","guarded_feedback"):
            s=e["statistics"][name];g=e["GT"][name]
            a=read(directory/name/"admission.csv")
            frames={r["transaction_id"]:r for r in read(directory/name/"frames.csv")}
            accepted=[r for r in a if r["admitted"]=="1"]
            separations=[distance(matrix(frames[r["transaction_id"]]["nominal_pose"]),matrix(r["candidate_pose"])) for r in accepted]
            rows.append(dict(attempt=attempt,mode=name,frames=s["frames"],trigger=s["raw_trigger_count"],search=s["search_count"],
                pending=s["pending_created"],temporal=s["temporally_supported"],admitted=s["admitted"],used=s["alternative_used"],
                full_NDT_calls=s["full_ndt_calls"],jet_calls=s["jet_calls"],preview_calls=s["preview_calls"],
                mean_ms=s["frame_processing_logging_ms"]["mean"],P95_ms=s["frame_processing_logging_ms"]["P95"],
                max_ms=s["frame_processing_logging_ms"]["max"],peak_RSS_KiB=s["peak_rss_kib"],
                corrected_translation_RMSE_m=g["corrected_translation_m"]["RMSE"],
                corrected_translation_P95_m=g["corrected_translation_m"]["P95"],corrected_translation_max_m=g["corrected_translation_m"]["max"],
                corrected_rotation_RMSE_deg=g["corrected_rotation_deg"]["RMSE"],
                corrected_rotation_P95_deg=g["corrected_rotation_deg"]["P95"],corrected_rotation_max_deg=g["corrected_rotation_deg"]["max"],
                actual_raw_translation_RMSE_m=g["actual_raw_translation_m"]["RMSE"],actual_raw_rotation_RMSE_deg=g["actual_raw_rotation_deg"]["RMSE"],
                large_jumps=s["large_jumps"],max_update_dt_m=s["max_update_translation_m"],max_update_dr_deg=s["max_update_rotation_deg"],
                max_admitted_nominal_dt_m=max((p[0] for p in separations),default=0),
                max_admitted_nominal_dr_deg=max((p[1] for p in separations),default=0)))
            total_calls+=s["full_ndt_calls"]
        causality=read(directory/"causal_feedback_parity.csv")
        branch=read(directory/"branch_score_parity.csv")
        details["attempt_"+str(attempt)]=dict(code_sha=e["code_sha"],accuracy_improvement_percent=100*e["translation_RMSE_improvement_fraction"],
            admission_outcomes=e["GT"]["admitted_outcomes"],admission_status={n:e["statistics"][n]["admission_status_counts"] for n in ("event_admission","guarded_feedback")},
            first_used_tx=causality[0]["first_alternative_tx"],prediction_divergence_count=sum(r["prediction_differs_from_shadow"]=="1" for r in causality),
            branch_audit_rows=len(branch),max_branch_audit_difference=max((float(r["maximum_difference"]) for r in branch),default=0))
    csv_write(ARCHIVE/"method_summary.csv",rows)
    json_write(ARCHIVE/"handoff_summary.json",dict(methods=rows,details=details,total_real_NDT_calls=total_calls))
    print(json.dumps(dict(methods=rows,details=details,total_real_NDT_calls=total_calls),indent=2))


if __name__=="__main__":main()
