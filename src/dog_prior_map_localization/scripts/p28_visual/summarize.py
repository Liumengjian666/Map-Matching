#!/usr/bin/env python3
"""Finite P28 engineering summary; optional GT only after output-freeze gate."""
import argparse
import collections
import csv
import json
import sys
from pathlib import Path
import numpy as np
from freeze import sha


def read(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write(path, records):
    if not records:
        return
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(records)


def timing(values):
    array = np.asarray(values, dtype=float)
    return dict(count=len(array), mean_ms=float(array.mean()),
                p95_ms=float(np.percentile(array, 95)), max_ms=float(array.max()))


def engineering(root):
    receipt = json.loads((root / "execution.json").read_text())
    ndt, oosm, lineage = (read(root / name) for name in
                         ("ndt_diagnostics.csv", "oosm.csv", "ekf_lineage.csv"))
    visual = read(root / "msckf_diagnostics.csv")
    mature = read(root / "mature_diagnostics.csv")
    high = [row for row in lineage if row["event_type"] == "IMU_HIGH_RATE_PUBLISH"]
    xyz = np.array([[float(row[key]) for key in ("p_x", "p_y", "p_z")] for row in high])
    poses = np.array([[float(row[key]) for key in ("p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w")]
                      for row in high])
    jumps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    sqrt = read(root / "sqrt_covariance_summary.csv")
    fixed = read(root / "oosm_fixed_lag.csv")
    applied = [row for row in visual if row["status"] == "APPLIED_REPROJECTION_SELECTED_GAIN"]
    result = dict(receipt=receipt, NDT=len(ndt),
        NDT_converged=sum(row["ndt_has_converged"] == "1" for row in ndt),
        OOSM=dict(collections.Counter(row["oosm_result"] for row in oosm)),
        highrate=len(high), corrected=sum(row["event_type"] == "CORRECTED_PUBLISH" for row in lineage),
        visual_status=dict(collections.Counter(row["status"] for row in visual)),
        visual_updates=len(applied), visual_accepted_tracks=sum(int(row["accepted"]) for row in applied),
        residual_rows=sum(int(row["residual_rows"]) for row in applied),
        nonfinite_pose_values=int((~np.isfinite(poses)).sum()),
        max_adjacent_jump_m=float(jumps.max()), jumps_over_2m=int((jumps > 2).sum()),
        invalid_factors=sum(int(row["invalid"]) for row in sqrt),
        late=sum("LATE" in row["status"] for row in fixed),
        dcreg_valid=sum(row["dcreg_status"].startswith("DCREG") for row in mature),
        extra_aligns=sum(int(row["extra_aligns"]) for row in mature),
        quick_scores=sum(int(row["quick_score_calls"]) for row in mature),
        replacements=sum(int(row["selected_candidate"]) >= 0 for row in mature),
        covariance_scale=read(root / "sqrt_covariance_scale.csv"),
        SKF_processing_all=timing([float(row["update_ms"]) for row in visual]),
        SKF_processing_applied=timing([float(row["update_ms"]) for row in applied]) if applied else None)
    # Timing column names are the existing online contracts, not guessed sums.
    result["ndt_fields"] = list(ndt[0])
    result["sqrt_timing"] = sqrt
    return result


def evaluate(args, methods):
    receipt = json.loads(args.freeze.read_text())
    if receipt["GT_LOADED"] is not False or receipt["protocol"] != "P28_R2_PRE_GT_OUTPUT_FREEZE":
        raise ValueError("P28_PRE_GT_FREEZE_REQUIRED")
    for path, digest in receipt["files_sha256"].items():
        if sha(path) != digest:
            raise ValueError("FROZEN_OUTPUT_CHANGED:" + path)
    for root in methods.values():
        for name in ("ndt_diagnostics.csv", "ekf_lineage.csv", "execution.json", "outputs.bag"):
            if str((root / name).resolve()) not in receipt["files_sha256"]:
                raise ValueError("RUN_NOT_FROZEN")
    historical_metric = args.repo / "docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json"
    if str(historical_metric.resolve()) not in receipt["files_sha256"]:
        raise ValueError("CONTROL_TRANSFORM_NOT_FROZEN")
    if sha(args.gt) != "3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03":
        raise ValueError("GT_HASH_MISMATCH")
    if sha(args.extrinsics) != "59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d":
        raise ValueError("EXTRINSIC_HASH_MISMATCH")
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p10_corridor01_deterministic_oosm"))
    import evaluate_frozen_v2 as frozen
    G = frozen.geometry
    history = historical_metric
    contract = json.loads(history.read_text())
    origin = contract["evaluation_origin_unix_s"]
    R = np.array(contract["fit_transform_rotation"])
    t = np.array(contract["fit_transform_translation_m"])
    gt_times, gt_poses = G.load_gt(args.gt)
    T = G.load_t_imu_lidar(args.extrinsics)
    metrics, samples, windows = [], [], {"3_35": (3,35), "26_35": (26,35), "34_35": (34,35)}
    paired = {}
    for method, root in methods.items():
        diag = read(root / "ndt_diagnostics.csv")
        high, _ = frozen.state_trajectory(root / "ekf_lineage.csv")
        components = {label: frozen.measurement_trajectory(diag,prefix,T,origin)
            for label,prefix in (("Initial_Guess","initial_guess_"),("Raw_NDT","raw_ndt_"),("Final_Used","final_used_"))}
        components["EKF_HighRate"] = high
        for component, records in components.items():
            times, et, er, _ = frozen.summarize_window(frozen.pair_with_gt(records,gt_times,gt_poses,origin,35),R,t)
            if component in paired and not np.array_equal(times,paired[component]):
                raise ValueError("A_B_EVALUATION_TIMESTAMPS_DIFFER")
            paired[component] = times
            for window,(start,end) in windows.items():
                mask = (times >= start) & (times <= end)
                if not mask.any():
                    raise ValueError("EMPTY_FIXED_WINDOW")
                row = dict(method=method,component=component,window=window,count=int(mask.sum()))
                for unit, values in (("translation_m",et[mask]),("rotation_deg",er[mask])):
                    row[unit+"_rmse"] = float(np.sqrt(np.mean(values**2)))
                    row[unit+"_p95"] = float(np.percentile(values,95))
                    row[unit+"_max"] = float(values.max())
                row["over_1m"] = int((et[mask]>1).sum())
                row["over_2m"] = int((et[mask]>2).sum())
                row["over_5m"] = int((et[mask]>5).sum())
                metrics.append(row)
            for i,time in enumerate(times):
                samples.append(dict(method=method,component=component,relative_time_s=float(time),
                    translation_error_m=float(et[i]),rotation_error_deg=float(er[i])))
    comparisons = []
    for a in [row for row in metrics if row["method"]=="A"]:
        b = next(row for row in metrics if row["method"]=="B" and row["component"]==a["component"] and row["window"]==a["window"])
        for field in ("translation_m_rmse","translation_m_p95","translation_m_max","rotation_deg_rmse","rotation_deg_p95","rotation_deg_max"):
            comparisons.append(dict(component=a["component"],window=a["window"],metric=field,
                A=a[field],B=b[field],delta=b[field]-a[field],relative_percent=(b[field]-a[field])/a[field]*100,count=a["count"]))
    write(args.output / "method_metrics.csv", metrics)
    write(args.output / "accuracy_comparison.csv", comparisons)
    write(args.output / "per_sample_errors.csv", samples)
    return dict(contract=contract["evaluation_contract"],origin=origin,fit_rotation=R.tolist(),fit_translation=t.tolist(),
                metrics=metrics, GT_LOADED=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--freeze",type=Path)
    parser.add_argument("--gt",type=Path)
    parser.add_argument("--extrinsics",type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    cache = args.repo / ".p28_r2_mature_visual_module"
    methods = {name:cache / "runs" / directory for name,directory in (("A","full_a"),("B","full_b"))}
    runs = {name:engineering(root) for name,root in methods.items() if (root / "execution.json").exists()}
    runs["smoke"] = engineering(cache / "runs/smoke_b_fixed")
    result = dict(task="P28-R2-MATURE-VISUAL-MODULE-REPLACEMENT",GT_LOADED=False,runs=runs,
        frontend=json.loads((cache / "feature_input_validated/extraction.json").read_text()))
    if args.gt:
        if set(runs) != {"A","B","smoke"} or not args.freeze or not args.extrinsics:
            raise ValueError("FULL_AB_AND_OUTPUT_FREEZE_REQUIRED")
        result["accuracy"] = evaluate(args,methods)
        result["GT_LOADED"] = True
    write(args.output / "skf_update_summary.csv",[dict(run=name,status=status,count=count)
        for name,run in runs.items() for status,count in run["visual_status"].items()])
    write(args.output / "covariance_summary.csv",[dict(run=name,**row) for name,run in runs.items() for row in run["covariance_scale"]])
    write(args.output / "resource_summary.csv",[dict(run=name,node=node,**value)
        for name,run in runs.items() for node,value in run["receipt"]["resources"].items()])
    import rosbag
    runtimes = []
    for name,run in runs.items():
        root = cache / "runs" / ("smoke_b_fixed" if name == "smoke" else "full_"+name.lower())
        values = {key:[] for key in ("align_time_ms","preprocess_time_ms","localization_time_ms")}
        with rosbag.Bag(str(root / "outputs.bag")) as bag:
            for _,message,_ in bag.read_messages(topics=["/dog_livo/diagnostics"]):
                for status in message.status:
                    data = {entry.key:entry.value for entry in status.values}
                    for key in values:
                        if key in data:
                            values[key].append(float(data[key]))
        values["deskew_processing_ms"] = [float(row["deskew_processing_ms"]) for row in read(root / "deskew.csv")
                                           if row["status"]=="PUBLISHED"]
        for key,samples in values.items():
            if samples:
                runtimes.append(dict(run=name,stage=key,**timing(samples)))
        for stage in ("SKF_processing_all","SKF_processing_applied"):
            if run[stage]:
                runtimes.append(dict(run=name,stage=stage,**run[stage]))
        for row in run["sqrt_timing"]:
            if row["event"] in ("IMU_PROPAGATE","MSCKF_UPDATE","NDT_REPLAY_END"):
                runtimes.append(dict(run=name,stage=row["event"],count=int(row["count"]),
                    mean_ms=float(row["mean_ms"]),p95_ms=float(row["p95_ms"]),max_ms=float(row["max_ms"])))
    write(args.output / "runtime_summary.csv",runtimes)
    write(args.output / "visual_residual_summary.csv",[dict(run=name,
        updates=run["visual_updates"],accepted_tracks=run["visual_accepted_tracks"],residual_rows=run["residual_rows"])
        for name,run in runs.items()])
    (args.output / "analysis.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps({name: {key:run[key] for key in ("NDT","highrate","visual_updates","invalid_factors","max_adjacent_jump_m")}
                      for name,run in runs.items()},indent=2))


if __name__ == "__main__":
    main()
