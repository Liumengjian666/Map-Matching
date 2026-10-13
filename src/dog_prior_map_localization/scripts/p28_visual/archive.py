#!/usr/bin/env python3
"""Archive real P28 evidence; no replay and no GT processing."""
import argparse
import csv
import json
import re
import subprocess
from pathlib import Path
import numpy as np
import yaml
from freeze import sha
from summarize import read, write


def flattened(value, prefix=""):
    result = {}
    for key, item in value.items():
        path = prefix + "/" + key
        if isinstance(item, dict):
            result.update(flattened(item,path))
        else:
            result[path] = item
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo",type=Path,required=True)
    parser.add_argument("--refresh-hashes",action="store_true")
    args = parser.parse_args()
    repo = args.repo.resolve()
    cache = repo / ".p28_r2_mature_visual_module"
    archive = repo / "docs/p28_r2_mature_visual_module"
    if args.refresh_hashes:
        hashes = {str(file.relative_to(repo)):sha(file) for file in sorted(archive.rglob("*"))
                  if file.is_file() and file.name!="artifact_hashes.json"}
        changed = subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--name-only"],text=True).splitlines()
        added = ["src/dog_prior_map_interfaces/msg/VisualDegeneracy.msg",
            "src/dog_prior_map_localization/config/p28_mature_visual.yaml",
            "src/dog_prior_map_localization/include/dog_prior_map_localization/core/msckf_visual_measurement.hpp",
            "src/dog_prior_map_localization/src/core/msckf_visual_measurement.cpp",
            "src/dog_prior_map_localization/src/fusion/msckf_visual.cpp",
            "src/dog_prior_map_localization/tests/msckf_visual_measurement_test.cpp"]
        added += [str(file.relative_to(repo)) for file in (repo / "src/dog_prior_map_localization/scripts/p28_visual").glob("*.py")]
        for relative in changed + added:
            file=repo / relative
            if file.is_file() and file.name!="artifact_hashes.json":
                hashes[relative]=sha(file)
        (archive / "artifact_hashes.json").write_text(json.dumps(hashes,indent=2)+"\n")
        return
    result = json.loads((cache / "posthoc_evaluation/analysis.json").read_text())
    if (archive / "results.json").exists():
        raise ValueError("P28_RESULTS_ALREADY_ARCHIVED")
    result.update(START_SHA="237ae40fa993eba7331ff15cbdf3f14fc583832c",
        repository="https://github.com/Liumengjian666/Map-Matching",
        branch=subprocess.check_output(["git","-C",str(repo),"branch","--show-current"],text=True).strip(),
        PUSH_EXECUTED=False,
        FINAL_RESULT="MATURE_VISUAL_MEASUREMENT_INTEGRATED_ACCURACY_REGRESSION",
        next="Bounded investigation of tail NDT incorrect-local-match detection/acceptance; do not retune visual noise",
        limitations=["Independent published-mathematics adaptation, not official OpenVINS/SKF reproduction",
          "A/B share approved new time/error/noise interface; P24-to-B difference is not SKF-only",
          "NDT covariance damping/pose fusion remains heuristic; full statistical consistency is not proven",
          "Only rotation weak directions were updated; no direct visual position correction",
          "Offline actual-image tracks; real-time camera pipeline and unique-system-memory not measured",
          "Raw camera full hash not rechecked; selected image content and calibration hashes are frozen"])
    roots = {"A":cache / "runs/full_a","B":cache / "runs/full_b"}
    parameters = {k:flattened(yaml.safe_load((root / "rosparams.yaml").read_text())) for k,root in roots.items()}
    diffs = []
    for key in sorted(set(parameters["A"]) | set(parameters["B"])):
        a,b = parameters["A"].get(key),parameters["B"].get(key)
        if a == b:
            continue
        kind = "intervention" if key=="/mature_visual/enable" else (
            "diagnostic_path" if key.startswith("/output/") or key=="/mature_visual/diagnostics_csv" else
            "ROS_metadata" if key=="/run_id" or key.startswith("/roslaunch/uris/") else "UNEXPECTED")
        diffs.append(dict(key=key,A=a,B=b,classification=kind))
    if any(row["classification"]=="UNEXPECTED" for row in diffs):
        raise ValueError("A_B_UNEXPECTED_PARAMETER_DIFFERENCE")
    ndt = {k:read(root / "ndt_diagnostics.csv") for k,root in roots.items()}
    high = {k:[row for row in read(root / "ekf_lineage.csv") if row["event_type"]=="IMU_HIGH_RATE_PUBLISH"] for k,root in roots.items()}
    parity = dict(parameter_contract_pass=True,differences=diffs,
        NDT_timestamps_equal=[row["lidar_header_stamp"] for row in ndt["A"]]==[row["lidar_header_stamp"] for row in ndt["B"]],
        highrate_timestamps_equal=[row["state_stamp"] for row in high["A"]]==[row["state_stamp"] for row in high["B"]],
        binary_hashes_equal=result["runs"]["A"]["receipt"]["binary_hashes"]==result["runs"]["B"]["receipt"]["binary_hashes"])
    if not all(parity[key] for key in ("NDT_timestamps_equal","highrate_timestamps_equal","binary_hashes_equal")):
        raise ValueError("A_B_FROZEN_TIME_BINARY_CONTRACT")
    (archive / "parameter_parity.json").write_text(json.dumps(parity,indent=2)+"\n")
    result["parameter_parity"] = parity
    for name,root in {**roots,"smoke":cache / "runs/smoke_b_fixed"}.items():
        target = archive / "runs" / name
        target.mkdir(parents=True,exist_ok=False)
        for file in root.iterdir():
            if file.is_file() and file.suffix in (".csv",".json",".yaml"):
                # CSV/receipt exact contents, normalized log copies only below.
                (target / file.name).write_bytes(file.read_bytes())
    failed = archive / "startup_failure"
    failed.mkdir()
    for filename in ("execution.json","launch.log"):
        content = (cache / "runs/smoke_b" / filename).read_text(errors="replace")
        content = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", content).replace("\r", "")
        content = "\n".join(line.rstrip() for line in content.splitlines()) + "\n"
        (failed / filename).write_text(content)
    (failed / "diagnosis.json").write_text(json.dumps(dict(playback_started=False,NDT_frames=0,
        observed="NDT process exited code 1 before bag playback; launch stopped",
        diagnosed_condition="P24 mature gate: !visual_guess && !p25_relative_contract",
        fix="Allow explicit P28 time-adapter mode without enabling old PnP",
        second_preflight="B port 11428 bind EADDRINUSE before starting any process; switched unused port 11429"),indent=2)+"\n")
    for file in (cache / "posthoc_evaluation").iterdir():
        if file.suffix==".csv":
            (archive / file.name).write_bytes(file.read_bytes())
    (archive / "error_window_summary.csv").write_bytes((archive / "method_metrics.csv").read_bytes())
    inputs = archive / "visual_inputs"
    inputs.mkdir()
    for filename in ("extraction.json","frontend_summary.csv","tracks.csv"):
        (inputs / filename).write_bytes((cache / "feature_input_validated" / filename).read_bytes())
    v = read(roots["B"] / "msckf_diagnostics.csv")
    origin = 1517157219188978910
    coverage = {window:sum(row["status"].startswith("APPLIED") and lo<=(int(row["image_ns"])-origin)*1e-9<=hi for row in v)
                for window,(lo,hi) in {"3_35":(3,35),"26_35":(26,35),"34_35":(34,35)}.items()}
    result["visual_coverage"] = coverage
    result["direct_translation_correction_max_m"] = max(float(row["correction_m"]) for row in v)
    result["causal_degeneracy_future_count"] = sum(int(row["degeneracy_end_ns"])>int(row["image_ns"]) for row in v)
    result["EXPERIMENT_CONTRACT"] = "PASS"
    result["NUMERICAL_STATUS"] = "PASS_WITHOUT_STATISTICAL_CONSISTENCY_CLAIM"
    result["ACCURACY_STATUS"] = "REGRESSION"
    result["INTEGRATION_STATUS"] = "RUNTIME_HYBRID_ADAPTATION"
    result["SKF_STATUS"] = "SELECTED_GAIN_REPROJECTION_UPDATES_EXECUTED_NOT_OFFICIAL_SKF"
    result["RUNTIME_STATUS"] = "MEASURED_OFFLINE_FRONTEND_NOT_ONLINE_DEPLOYMENT_PASS"
    # Independent recalculation from serialized per-sample errors, not the
    # evaluator's summary function. Preserve all positive and negative metrics.
    samples = read(archive / "per_sample_errors.csv")
    recalculated = []
    for row in read(archive / "method_metrics.csv"):
        lo,hi=map(float,row["window"].split("_"))
        selected = [s for s in samples if s["method"]==row["method"] and s["component"]==row["component"] and lo<=float(s["relative_time_s"])<=hi]
        for unit,field in (("translation_m","translation_error_m"),("rotation_deg","rotation_error_deg")):
            values = np.array([float(s[field]) for s in selected])
            for metric,value in zip(("rmse","p95","max"),(np.sqrt((values**2).mean()),np.percentile(values,95),values.max())):
                recalculated.append(dict(method=row["method"],component=row["component"],window=row["window"],unit=unit,metric=metric,
                    recomputed=float(value),archived=float(row[unit+"_"+metric]),difference=float(value)-float(row[unit+"_"+metric])))
    write(archive / "metric_recalculation.csv",recalculated)
    if any(abs(row["difference"])>1e-12 for row in recalculated):
        raise ValueError("POSTHOC_METRIC_RECALCULATION_FAILED")
    test_log = Path("/tmp/p25-catkin-AY1eHd/build/dog_prior_map_localization/Testing/Temporary/LastTest.log")
    (archive / "test_results.txt").write_text(test_log.read_text().replace("\r", ""))
    # Positive-factor derived dense views at five actual sparse checkpoints.
    numerical = []
    for name,root in roots.items():
        factors = read(root / "sqrt_covariance_factors.csv")
        for checkpoint in dict.fromkeys(row["checkpoint"] for row in factors):
            S=np.zeros((15,15))
            for row in factors:
                if row["checkpoint"]==checkpoint:
                    S[int(row["row"]),int(row["column"])]=float(row["value"])
            numerical.append(dict(run=name,checkpoint=checkpoint,min_factor_diag=float(S.diagonal().min()),
                dense_derived_lambda_min=float(np.linalg.eigvalsh(S@S.T).min()),trace=float((S*S).sum())))
    write(archive / "factor_checkpoints.csv",numerical)
    result["metric_recalculation_max_difference"] = max(abs(row["difference"]) for row in recalculated)
    result["camera_CPU_recorded"] = False
    (archive / "results.json").write_text(json.dumps(result,indent=2)+"\n")
    hashes = {str(file.relative_to(repo)):sha(file) for file in sorted(archive.rglob("*")) if file.is_file() and file.name!="artifact_hashes.json"}
    sources = [p for p in (repo / "src/dog_prior_map_localization/scripts/p28_visual").glob("*.py")]
    sources += [repo / p for p in ("src/dog_prior_map_localization/src/fusion/msckf_visual.cpp",
        "src/dog_prior_map_localization/src/core/msckf_visual_measurement.cpp",
        "src/dog_prior_map_localization/include/dog_prior_map_localization/core/msckf_visual_measurement.hpp",
        "src/dog_prior_map_localization/tests/msckf_visual_measurement_test.cpp")]
    hashes.update({str(file.relative_to(repo)):sha(file) for file in sources})
    (archive / "artifact_hashes.json").write_text(json.dumps(hashes,indent=2)+"\n")
    print(json.dumps(dict(FINAL_RESULT=result["FINAL_RESULT"],visual_updates=result["runs"]["B"]["visual_updates"],coverage=coverage)))


if __name__ == "__main__":
    main()
