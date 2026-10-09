"""Compact factual R5 report, without new analysis or experiments."""
import json
from run_anchored import ARCHIVE,MODES
from run_budgeted import read
for directory in sorted(ARCHIVE.glob("attempt_*")):
    if not (directory/"evaluation.json").is_file():continue
    e=json.loads((directory/"evaluation.json").read_text());result={}
    for n,s in e["statistics"].items():
        gt=e["GT"][n];a=s["frame_processing_logging_ms"]
        result[n]=dict(frames=s["frames"],anchor_valid=s.get("anchor_valid_frames"),anchor_established=s.get("anchor_established"),
            trigger=s["raw_trigger_count"],search=s["search_count"],pending=s["pending_created"],support=s["temporally_supported"],
            admitted=s.get("admitted",0),used=s.get("alternative_used",0),ndt_calls=s["full_ndt_calls"],jet=s["jet_calls"],preview=s["preview_calls"],
            nonlocal_terminals=s["nonlocal_terminals"],eligible_nonlocal=s["eligible_nonlocal_terminals"],local=s["local_recommendations"],
            mean=a["mean"],P95=a["P95"],max=a["max"],rss=s["peak_rss_kib"],wall=s.get("wall_s"),jumps=s["large_jumps"],
            corrected_translation={k:gt["corrected_translation_m"][k] for k in ("RMSE","P95","max")},
            corrected_rotation={k:gt["corrected_rotation_deg"][k] for k in ("RMSE","P95","max")},
            raw_translation={k:gt["actual_raw_translation_m"][k] for k in ("RMSE","P95","max")},
            raw_rotation={k:gt["actual_raw_rotation_deg"][k] for k in ("RMSE","P95","max")},
            max_update_translation=s.get("max_update_translation_m"),max_update_rotation=s.get("max_update_rotation_deg"),
            admitted_GT=e["GT"]["admitted_outcomes"].get(n))
    print(json.dumps(dict(attempt=e["attempt"],code_sha=e["code_sha"],improvement=e["translation_RMSE_improvement_fraction"],
        anchor_audit_max=e["max_anchor_audit_difference"],methods=result,windows=read(directory/"known_development_windows.csv")),indent=2))
