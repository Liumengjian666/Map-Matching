"""Create the final R3 fact receipt and audit small Git artifacts; no experiments."""
import csv
import json
from pathlib import Path
from run_budgeted import ROOT, sha, read, json_write
from run_event import ARCHIVE, START_SHA


def invalid_constant(value):
    raise RuntimeError("nonfinite JSON literal: " + value)


def audit():
    hashes, json_count, csv_count, csv_rows = {}, 0, 0, 0
    for path in sorted(ARCHIVE.rglob("*")):
        if not path.is_file() or path.name == "artifact_hashes.json":
            continue
        hashes[str(path.relative_to(ARCHIVE))] = sha(path)
        if path.suffix == ".json":
            json.loads(path.read_text(), parse_constant=invalid_constant)
            json_count += 1
        if path.suffix == ".csv":
            with path.open(newline="") as stream:
                rows = csv.reader(stream)
                header = next(rows)
                if len(set(header)) != len(header):
                    raise RuntimeError("duplicate CSV columns: " + str(path))
                for row in rows:
                    if len(row) != len(header):
                        raise RuntimeError("CSV width mismatch: " + str(path))
                    csv_rows += 1
            csv_count += 1
    # Every archived builder output remains byte-exact after post-hoc evaluation.
    archive = ARCHIVE / "attempt_0"
    blind = json.loads((archive / "blind_outputs_freeze.json").read_text())
    for path, digest in blind["output_sha256"].items():
        if sha(archive / path) != digest:
            raise RuntimeError("original blind output changed: " + path)
    freeze = json.loads((archive / "execution_freeze.json").read_text())
    binary = Path(freeze["output_directory"]) / "p10_r3_replay"
    if sha(binary) != freeze["binary_sha256"]:
        raise RuntimeError("execution binary snapshot mismatch")
    for path, digest in freeze["source_sha256"].items():
        if sha(ROOT / path) != digest:
            raise RuntimeError("execution source mismatch: " + path)
    return dict(artifact_sha256=hashes, hashed_files=len(hashes), JSON_files=json_count,
        CSV_files=csv_count, CSV_data_rows=csv_rows, blind_outputs_hash_guard="PASS",
        executed_source_binary_guard="PASS", JSON_finite="PASS", CSV_width="PASS", self_hash_excluded=True)


def finish():
    archive = ARCHIVE / "attempt_0"
    e = json.loads((archive / "evaluation.json").read_text())
    freeze = json.loads((archive / "execution_freeze.json").read_text())
    state = read(archive / "nominal_state_parity.csv")
    guards = read(archive / "engineering_guards.csv")
    if len(state) != 8254 or any(r["exact_parity"] != "1" for r in state):
        raise RuntimeError("full nominal/state parity incomplete")
    if len(guards) != 4127 or any(r["hard_requirements_pass"] != "1" for r in guards):
        raise RuntimeError("full engineering guard incomplete")
    events = {r["transaction_id"]: r for r in read(archive / "event/events.csv")}
    results = dict(task="PAPER-P10-R3-EVENT-TRIGGERED-COUPLED-NDT",
        final_result="EVENT_TRIGGERED_SHADOW_ENGINEERING_PASS_ACCURACY_NOT_ESTABLISHED",
        next="P10_R4_NONORACLE_ADMISSION_VALIDATION", branch="research/p9-r4-heldout-visual-evidence",
        start_sha=START_SHA, code_sha=freeze["code_sha"],
        algorithm_sha="1e43ce746ef4a525321448ec5a442e32079e4205",
        worktree="/tmp/dog_loc_paper_r4_ws.Fq21k2", push_executed=False,
        remote_head="NOT_VERIFIED_THIS_RUN", remote_query_error="Couldn't connect to server",
        engineering_hard_gates="PASS", performance_goals_pass=e["performance_goals_pass"],
        scientific_accuracy_improvement="NOT_ESTABLISHED", trajectory_improved=False, pose_switched=False,
        nonlocal_recommendations=0, nonfinite_recommended=0, real_attempts=1, targeted_performance_improvements=0,
        candidate_budget=dict(initial=8, maximum=16, extra_aligns=2, total_aligns=3, pending_extra_aligns=1),
        nominal_state_parity=dict(rows=len(state), exact=8254, R2_first200_rows=400),
        statistics=e["statistics"], groups=e["groups"], segments=e["segments"],
        pending_episodes=e["pending_episodes"], GT=e["GT"],
        key_frames={tx: events[tx] for tx in ("616", "2350", "3341")},
        total_real_ndt_calls=9256, oracle_calls=0, B12_calls=0, source_recovery_calls=0,
        visual_extraction=0, Corridor_bootstrap_runs=0, GT_USED="POST_FREEZE_ONLY",
        binary_sha256=freeze["binary_sha256"],
        timing_limitations=dict(startup_components="NOT_SEPARATELY_TIMED",
            frame_cost_excludes_own_cost_row=True, process_wall_includes_startup_and_close=True,
            RSS_increment="HIGH_WATER_DIFFERENCE_PROXY_ONLY", worst_case_real_time_guaranteed=False),
        R2_same_200_comparison=dict(old_mean_ms=138.54727551, old_P95_ms=197.02056185,
            new_mean_ms=e["segments"]["TX1_200"]["event"]["frame_processing_logging_ms"]["mean"],
            new_P95_ms=e["segments"]["TX1_200"]["event"]["frame_processing_logging_ms"]["P95"],
            old_NDT_calls=296, new_NDT_calls=200, old_preview_calls=3200, new_preview_calls=16,
            inference="DESCRIPTIVE_DEVELOPMENT_COMPARISON_NOT_INDEPENDENT_VALIDATION"),
        limitations=["25 improved / 27 worse local recommendations; translation RMSE slightly worse",
            "57 temporally supported alternates: 22 improved / 35 worse; temporal support is not correctness",
            "TX4127 GT unavailable; all experimental frames retained; no extrapolation",
            "normal TX1-200 recommendation stays nominal; R2's earlier local benefit is not retained",
            "startup component timers absent; raw process wall and aggregate remainder preserved",
            "Floor01 development data only; no production pose switching or independent generalization claim"])
    json_write(ARCHIVE / "results.json", results)
    manifest = audit()
    json_write(ARCHIVE / "artifact_hashes.json", manifest)
    print(json.dumps({k: v for k, v in manifest.items() if k != "artifact_sha256"}, indent=2))


if __name__ == "__main__":
    finish()
