"""R3 evaluation: verify complete blind receipts, then fixed-anchor post-hoc GT."""
import argparse
from collections import Counter
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from run_budgeted import ROOT, SAME, read, sha, csv_write, json_write
from run_event import ARCHIVE
from evaluate_budgeted import matrix, distance


def moments(values):
    a = np.asarray(values, dtype=float)
    if not len(a) or not np.isfinite(a).all():
        raise RuntimeError("empty/nonfinite statistics")
    return dict(mean=float(a.mean()), median=float(np.median(a)),
                P95=float(np.percentile(a, 95)), max=float(a.max()),
                RMSE=float(np.sqrt(np.mean(a*a))))


def verify(archive):
    freeze = json.loads((archive / "execution_freeze.json").read_text())
    blind = json.loads((archive / "blind_outputs_freeze.json").read_text())
    if blind["GT_LOADED"] or blind["ORACLE_LOADED"] or blind["NOMINAL_STATE_SWITCHED"]:
        raise RuntimeError("blind isolation contract failed")
    for path, digest in blind["output_sha256"].items():
        if sha(archive / path) != digest:
            raise RuntimeError("blind output hash mismatch: " + path)
    for path, digest in freeze["source_sha256"].items():
        if sha(ROOT / path) != digest:
            raise RuntimeError("executed algorithm source changed: " + path)
    if sha(ARCHIVE / "THEORY.md") != freeze["theory_sha256"]:
        raise RuntimeError("pre-run contract changed")
    selected = [r["transaction_id"] for r in read(archive / "frame_selection.csv")]
    jobs = {name: {file: read(archive / name / (file + ".csv")) for file in
        ("frames", "events", "frame_cost", "registration", "trajectory", "runtime", "resources", "pending_end")}
        for name in ("control", "event")}
    for name, tables in jobs.items():
        for table in ("frames", "events", "frame_cost", "registration", "trajectory", "runtime"):
            if [r["transaction_id"] for r in tables[table]] != selected:
                raise RuntimeError("incomplete/unordered ledger: " + name + "/" + table)
    parity = []
    for table in ("registration", "trajectory"):
        for a, b in zip(jobs["control"][table], jobs["event"][table]):
            diff = [key for key in a if key != "alignment_ms" and a[key] != b[key]]
            parity.append(dict(transaction_id=a["transaction_id"], table=table,
                exact_parity=int(not diff), different_fields=";".join(diff)))
    csv_write(archive / "nominal_state_parity.csv", parity)
    if not all(r["exact_parity"] for r in parity):
        raise RuntimeError("shadow changed nominal/source/state")
    checks = []
    for f, e in zip(jobs["event"]["frames"], jobs["event"]["events"]):
        normal = e["mode"] == "NORMAL"
        pending = e["mode"] == "PENDING"
        valid = e["recommendation_available"] == "1"
        nominal, recommended = matrix(f["nominal_pose"]), matrix(f["recommended_pose"])
        dt, dr = distance(nominal, recommended)
        ok = (int(f["full_ndt_calls"]) <= 3 and int(f["preview_count"]) <= 16 and
            f["nonfinite_recommended"] == "0" and
            (not valid or (dt <= .200001 and dr <= 2.000001)) and
            (not normal or all(int(f[k]) == 0 for k in ("jet_calls", "preview_score_calls", "terminal_score_calls"))
             and int(f["full_ndt_calls"]) == 1) and
            (not pending or int(f["full_ndt_calls"]) <= 2 and int(f["jet_calls"]) == 0
             and int(f["preview_score_calls"]) == 0 and int(f["preview_count"]) == 0) and
            (e["pending_before"] != "1" or e["mode"] != "SEARCH") and
            (valid or int(f["jet_calls"]) == 0 and int(f["preview_count"]) == 0))
        checks.append(dict(transaction_id=f["transaction_id"], mode=e["mode"], event=e["event"],
            hard_requirements_pass=int(ok), extra_aligns=max(0, int(f["full_ndt_calls"])-1),
            local_recommended=int(valid and int(f["recommended_id"]) >= 0)))
    csv_write(archive / "engineering_guards.csv", checks)
    if not all(r["hard_requirements_pass"] for r in checks):
        raise RuntimeError("budget/zero-call/nonlocal recommendation guard failed")
    # Prior R2's nominal measurements are an immutable regression control.
    prior = ROOT / "docs/p10_r2_budgeted_coupled_ndt/attempt_2/continuous_control"
    for table in ("registration", "trajectory"):
        for a, b in zip(read(prior / (table + ".csv")), jobs["control"][table][:200]):
            if any(a[k] != b[k] for k in a if k != "alignment_ms"):
                raise RuntimeError("prior R2 nominal regression failed")
    return freeze, jobs


def summarize(tables, indices):
    fs = [tables["frames"][i] for i in indices]
    es = [tables["events"][i] for i in indices]
    cs = [tables["frame_cost"][i] for i in indices]
    rs = [tables["runtime"][i] for i in indices]
    if not fs:
        return dict(frames=0, status="NOT_OBSERVED")
    mean = lambda key: float(np.mean([float(f[key]) for f in fs]))
    return dict(frames=len(fs), modes=dict(Counter(e["mode"] for e in es)),
        events=dict(Counter(e["event"] for e in es)),
        nominal_effective=sum(e["recommendation_available"] == "1" for e in es),
        raw_trigger_count=sum(e["innovation_trigger"] == "1" for e in es),
        search_count=sum(e["mode"] == "SEARCH" for e in es),
        full_ndt_calls=sum(int(f["full_ndt_calls"]) for f in fs),
        extra_ndt_calls=sum(max(0, int(f["full_ndt_calls"])-1) for f in fs),
        max_full_ndt_calls=max(int(f["full_ndt_calls"]) for f in fs),
        jet_calls=sum(int(f["jet_calls"]) for f in fs),
        preview_calls=sum(int(f["preview_score_calls"]) for f in fs),
        terminal_score_calls=sum(int(f["terminal_score_calls"]) for f in fs),
        max_previews=max(int(f["preview_count"]) for f in fs),
        local_recommendations=sum(int(f["recommended_id"]) >= 0 for f in fs),
        nonlocal_terminals=sum(int(e["nonlocal_terminals"]) for e in es),
        eligible_nonlocal_terminals=sum(int(e["eligible_nonlocal_terminals"]) for e in es),
        pending_created=sum(e["event"] == "PENDING_CREATED" for e in es),
        pending_first_support=sum(e["event"] == "PENDING_FIRST_SUPPORT" for e in es),
        temporally_supported=sum(e["event"] == "TEMPORALLY_SUPPORTED" for e in es),
        frame_processing_logging_ms=moments([float(c["processing_and_logging_ms"]) for c in cs]),
        frame_cpu_ms=moments([float(c["cpu_ms"]) for c in cs]),
        logging_ms=moments([float(c["logging_ms"]) for c in cs]),
        shadow_ms=moments([float(f["shadow_ms"]) for f in fs]),
        shadow_cpu_ms=moments([float(f["shadow_cpu_ms"]) for f in fs]),
        phase_mean_ms={key: mean(key) for key in ("model_ms", "jet_ms", "solve_ms", "preview_ms", "refinement_ms")},
        baseline_phase_mean_ms={key: float(np.mean([float(r[key]) for r in rs])) for key in
            ("prediction_and_deskew_ms", "cloud_io_ms", "ndt_total_ms", "ndt_alignment_ms", "ikfom_update_ms")})


def episodes(archive, events):
    rows = []; active = None
    for e in events:
        if e["event"] == "PENDING_CREATED":
            if active is not None:
                raise RuntimeError("pending replacement without clearing")
            active = dict(created_tx=e["transaction_id"], candidate_id=e["pending_candidate_id"],
                first_support_tx="", final_tx="", outcome="", confirmation_frames=0)
        elif active is not None and e["pending_before"] == "1":
            active["confirmation_frames"] += int(e["mode"] == "PENDING")
            if e["event"] == "PENDING_FIRST_SUPPORT":
                active["first_support_tx"] = e["transaction_id"]
            if e["pending_after"] == "0":
                active.update(final_tx=e["transaction_id"], outcome=e["event"])
                rows.append(active); active = None
    if active:
        active.update(outcome="UNCONFIRMED_END_OF_SEQUENCE"); rows.append(active)
    if rows:
        csv_write(archive / "pending_episodes.csv", rows)
    return dict(created=len(rows), outcomes=dict(Counter(r["outcome"] for r in rows)),
        max_confirmation_frames=max([r["confirmation_frames"] for r in rows], default=0))


def posthoc(archive, jobs):
    # This entry is reached only after every output digest and engineering guard passes.
    sys.path.insert(0, str(ROOT / "src/dog_prior_map_fastlio2_frontend_exp/scripts"))
    import p5_i1_posthoc_gt as contract
    import yaml
    if sha(contract.GT) != contract.EXPECTED_GT_SHA or sha(contract.EXTRINSICS) != contract.EXPECTED_EXTR_SHA:
        raise RuntimeError("GT/extrinsic hash contract failed")
    receipt = json.loads((ROOT / "docs/p9_r2a_predictor_conditioned_search/results.json").read_text())["gt_contract"]
    anchor = matrix(receipt["fixed_reference_alignment"])
    extr = np.asarray(yaml.safe_load(contract.EXTRINSICS.read_text())["laser_to_imu"]["data"]).reshape(4,4)
    extr[:3,:3] = Rotation.from_matrix(extr[:3,:3]).as_matrix()
    lidar_to_imu = np.linalg.inv(extr)
    times, poses = contract.gt_data(contract.GT)
    rows, executed, temporal, coverage = [], [], [], []
    trajectory = {r["transaction_id"]: r for r in jobs["event"]["trajectory"]}
    for f, e in zip(jobs["event"]["frames"], jobs["event"]["events"]):
        raw = contract.interpolate_gt(times, poses, int(f["stamp_ns"])/1e9)
        coverage.append(dict(transaction_id=f["transaction_id"], stamp_ns=f["stamp_ns"],
            GT_available=int(raw is not None), nominal_effective=e["recommendation_available"],
            status="BRACKETED" if raw is not None else "OUTSIDE_GT_TIME_RANGE_NO_EXTRAPOLATION"))
        if raw is None:
            continue  # Keep the full experimental ledger; only GT is unavailable.
        gt = anchor @ raw
        if e["recommendation_available"] == "1":
            nt, nr = contract.pose_error(matrix(f["nominal_pose"]) @ lidar_to_imu, gt)
            st, sr = contract.pose_error(matrix(f["recommended_pose"]) @ lidar_to_imu, gt)
            rows.append(dict(transaction_id=f["transaction_id"], mode=e["mode"], event=e["event"],
                nominal_translation_m=nt, recommended_translation_m=st, nominal_rotation_deg=nr,
                recommended_rotation_deg=sr, recommended_id=f["recommended_id"],
                outcome="SAME" if abs(st-nt) <= 1e-6 else "IMPROVED" if st < nt else "WORSE"))
            if e["event"] == "TEMPORALLY_SUPPORTED":
                at, ar = contract.pose_error(matrix(e["temporal_terminal"]) @ lidar_to_imu, gt)
                temporal.append(dict(transaction_id=f["transaction_id"], nominal_translation_m=nt,
                    alternate_translation_m=at, nominal_rotation_deg=nr, alternate_rotation_deg=ar,
                    outcome="SAME" if abs(at-nt) <= 1e-6 else "IMPROVED" if at < nt else "WORSE",
                    use="DIAGNOSTIC_ONLY_NOT_RECOMMENDED"))
        t = trajectory[f["transaction_id"]]
        T = np.eye(4); T[:3,3] = [float(t["corrected_imu_"+a]) for a in "xyz"]
        T[:3,:3] = Rotation.from_quat([float(t["corrected_imu_q"+a]) for a in "xyzw"]).as_matrix()
        et, er = contract.pose_error(T, gt)
        executed.append(dict(transaction_id=f["transaction_id"], translation_m=et, rotation_deg=er,
            update_applied=t["lidar_update_applied"], shadow_changed_state=0))
    csv_write(archive / "posthoc_gt.csv", rows)
    csv_write(archive / "gt_coverage.csv", coverage)
    csv_write(archive / "executed_nominal_gt.csv", executed)
    if temporal:
        csv_write(archive / "temporally_supported_gt.csv", temporal)
    # Reproduce the existing R2 fixed-anchor error convention on identical nominal frames.
    old = {r["transaction_id"]: r for r in read(ROOT /
        "docs/p10_r2_budgeted_coupled_ndt/attempt_2/posthoc_gt.csv") if r["job"] == "continuous_C"}
    parity = []
    for r in rows:
        if r["transaction_id"] not in old:
            continue
        previous = old[r["transaction_id"]]
        td = abs(r["nominal_translation_m"]-float(previous["nominal_translation_m"]))
        rd = abs(r["nominal_rotation_deg"]-float(previous["nominal_rotation_deg"]))
        parity.append(dict(transaction_id=r["transaction_id"], translation_diff_m=td, rotation_diff_deg=rd,
            pass_parity=int(td <= 1e-10 and rd <= 1e-8)))
    csv_write(archive / "gt_contract_parity.csv", parity)
    if len(parity) != 200 or not all(r["pass_parity"] for r in parity):
        raise RuntimeError("fixed-anchor R2 GT convention not reproduced")
    json_write(archive / "gt_contract_receipt.json", dict(fixed_anchor=receipt["fixed_reference_alignment"],
        anchor_stamp=receipt["anchor_stamp"], new_alignment_fitted=False, GT_FOR_SELECTION=False,
        input_sha256={str(p): sha(p) for p in (contract.GT, contract.EXTRINSICS)},
        blind_freeze_sha256=sha(archive / "blind_outputs_freeze.json")))
    result = {}
    for name, selected in (("full", rows), ("TX1_200", [r for r in rows if int(r["transaction_id"]) <= 200]),
        ("local_recommended", [r for r in rows if int(r["recommended_id"]) >= 0])):
        result[name] = dict(count=len(selected), outcomes=dict(Counter(r["outcome"] for r in selected)))
        if selected:
            for prefix in ("nominal", "recommended"):
                result[name][prefix+"_translation_m"] = moments([r[prefix+"_translation_m"] for r in selected])
                result[name][prefix+"_rotation_deg"] = moments([r[prefix+"_rotation_deg"] for r in selected])
    result["executed_nominal"] = dict(count=len(executed), translation_m=moments([r["translation_m"] for r in executed]),
        rotation_deg=moments([r["rotation_deg"] for r in executed]), shadow_changed_state=False)
    result["temporal_diagnostic"] = dict(count=len(temporal), outcomes=dict(Counter(r["outcome"] for r in temporal)),
        RECOMMENDED=False, TEMPORAL_SUPPORT_IS_GT_CORRECTNESS=False)
    result["coverage"] = dict(experimental_frames=len(coverage), GT_available=sum(r["GT_available"] for r in coverage),
        unavailable_transactions=[r["transaction_id"] for r in coverage if not r["GT_available"]],
        frames_deleted=False, GT_extrapolated=False)
    return result


def evaluate(attempt):
    archive = ARCHIVE / f"attempt_{attempt}"
    freeze, jobs = verify(archive)
    summaries = {}
    for name, tables in jobs.items():
        indices = list(range(len(tables["frames"])))
        summaries[name] = summarize(tables, indices)
        receipt = json.loads((archive / name / "receipt.json").read_text())
        summaries[name].update(wall_s=receipt["wall_s"],
            amortized_process_wall_ms=receipt["wall_s"]*1000/len(indices),
            peak_rss_kib=int(tables["resources"][0]["peak_rss_kib"]),
            process_cpu_s=float(tables["resources"][0]["user_s"])+float(tables["resources"][0]["system_s"]))
    groups = {mode: summarize(jobs["event"], [i for i, e in enumerate(jobs["event"]["events"]) if e["mode"] == mode])
              for mode in ("NORMAL", "SEARCH", "PENDING", "INVALID")}
    segments = {}
    for key, predicate in (("TX1_200", lambda tx: 1 <= tx <= 200), ("TX616_pm10", lambda tx: abs(tx-616) <= 10),
        ("TX2350_pm10", lambda tx: abs(tx-2350) <= 10), ("TX3341_pm10", lambda tx: abs(tx-3341) <= 10)):
        segments[key] = {name: summarize(tables, [i for i, f in enumerate(tables["frames"])
            if predicate(int(f["transaction_id"]))]) for name, tables in jobs.items()}
    pending = episodes(archive, jobs["event"]["events"])
    result = dict(attempt=attempt, start_sha=freeze["start_sha"], code_sha=freeze["code_sha"],
        engineering="PASS", nominal_source_state_parity="EXACT_ALL_REGISTRATION_AND_TRAJECTORY_ROWS",
        prior_R2_nominal_parity="400/400 EXACT", statistics=summaries, groups=groups, segments=segments,
        pending_episodes=pending, GT=posthoc(archive, jobs),
        performance_goals_pass=summaries["event"]["frame_processing_logging_ms"]["mean"] <= 100 and
            summaries["event"]["frame_processing_logging_ms"]["P95"] <= 150,
        POSE_SWITCHED=False, SINGLE_MAP_INSTANCE=True)
    json_write(archive / "evaluation.json", result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(); parser.add_argument("--attempt", type=int, default=0)
    evaluate(parser.parse_args().attempt)
