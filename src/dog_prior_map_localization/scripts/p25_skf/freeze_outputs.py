#!/usr/bin/env python3
"""Hash the complete P25 A/B runtime evidence before any GT file is opened."""
import argparse
import hashlib
import json
from pathlib import Path

import yaml


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def inventory(root):
    names = ("execution.json", "rosparams.yaml", "ndt_diagnostics.csv", "ekf_lineage.csv",
        "oosm.csv", "oosm_fixed_lag.csv", "deskew.csv", "ekf_runtime.csv",
        "mature_diagnostics.csv", "resources.csv", "outputs.bag", "launch.log",
        "record.log", "playback.log", "skf_measurements.csv", "skf_update_diagnostics.csv")
    output = {}
    for name in names:
        path = root / name
        if path.is_file():
            output[name] = {"path": str(path.resolve()), "size_bytes": path.stat().st_size,
                            "sha256": sha(path)}
    required = ("execution.json", "rosparams.yaml", "ndt_diagnostics.csv", "ekf_lineage.csv",
        "oosm.csv", "oosm_fixed_lag.csv", "deskew.csv", "ekf_runtime.csv",
        "mature_diagnostics.csv", "resources.csv", "outputs.bag", "launch.log",
        "record.log", "playback.log")
    missing = [name for name in required if name not in output]
    if missing:
        raise ValueError(f"P25_RUN_ARTIFACTS_MISSING:{root}:{','.join(missing)}")
    receipt = json.loads((root / "execution.json").read_text())
    if receipt.get("completed") is not True or receipt.get("GT_LOADED") is not False:
        raise ValueError("P25_RUN_NOT_COMPLETE_OR_GT_ALREADY_LOADED:" + str(root))
    return output


def flatten(value, prefix=""):
    if isinstance(value, dict):
        rows = {}
        for key, child in value.items():
            rows.update(flatten(child, prefix + "/" + str(key)))
        return rows
    return {prefix or "/": value}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--a-run", type=Path, required=True)
    parser.add_argument("--b-run", type=Path, required=True)
    parser.add_argument("--visual", type=Path, required=True)
    parser.add_argument("--extraction", type=Path, required=True)
    parser.add_argument("--dataset-config", type=Path, required=True)
    parser.add_argument("--mature-config", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--devel", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    repo = args.repo.resolve()
    a_root, b_root = args.a_run.resolve(), args.b_run.resolve()
    p10_contract = repo / "docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json"
    p19_visual = repo / ".p17_visual_aided_ndt_r1/visual_input/visual.csv"
    files = [args.visual.resolve(), args.extraction.resolve(), args.dataset_config.resolve(),
             args.mature_config.resolve(), args.map.resolve(), args.bag.resolve(),
             p10_contract, p19_visual]
    missing = [str(path) for path in files if not path.is_file()]
    if missing:
        raise ValueError("P25_FREEZE_INPUT_MISSING:" + ";".join(missing))
    extraction = json.loads(args.extraction.read_text())
    if extraction.get("GT_LOADED") is not False:
        raise ValueError("P25_EXTRACTION_RECEIPT_GT_CONTRACT_FAILED")
    if extraction.get("frozen_visual_pose_parity", {}).get("result") != "P4_POSE_PARITY_PASS":
        raise ValueError("P25_P4_FROZEN_VISUAL_PARITY_FAILED")

    input_hashes = {str(path.resolve()): {"size_bytes": path.stat().st_size, "sha256": sha(path)}
                    for path in files}
    output_inventory = {"A_SKF_OFF": inventory(a_root), "B_SKF_ON": inventory(b_root)}
    params_a = flatten(yaml.safe_load((a_root / "rosparams.yaml").read_text()))
    params_b = flatten(yaml.safe_load((b_root / "rosparams.yaml").read_text()))
    ignored_output_paths = {
        "/output/runtime_csv_path", "/output/ndt_diagnostics_csv_path",
        "/output/ekf_prediction_diagnostics_csv_path", "/output/oosm_csv_path",
        "/output/oosm_fixed_lag_csv_path", "/output/imu_deskew_csv_path",
        "/output/mature_integration_diagnostics_csv_path",
        "/output/skf_measurement_csv_path", "/output/skf_update_diagnostics_csv_path",
    }
    differences = {key: {"A": params_a.get(key), "B": params_b.get(key)}
        for key in sorted(set(params_a) | set(params_b))
        if params_a.get(key) != params_b.get(key) and key not in ignored_output_paths}
    expected_differences = {"/mature_integration/skf_enable": {"A": False, "B": True}}
    if differences != expected_differences:
        raise ValueError("P25_A_B_PARAMETER_PARITY_FAILED:" + json.dumps(differences, sort_keys=True))
    runtime_sources = ("src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp",
        "src/dog_prior_map_localization/src/fusion/skf_visual.cpp",
        "src/dog_prior_map_localization/src/core/selective_relative_visual_update.cpp",
        "src/dog_prior_map_localization/src/skf_visual_measurement.cpp",
        "src/dog_prior_map_localization/src/imu_processor.cpp",
        "src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp",
        "src/dog_prior_map_fastlio2_frontend_exp/scripts/p4_i3_visual_frontend.py",
        "src/dog_prior_map_localization/scripts/p17_visual/prepare.py",
        "src/dog_prior_map_localization/launch/dog_prior_map_localization_split.launch",
        "src/dog_prior_map_localization/config/p25_skf_visual_update.yaml")
    source_hashes = {rel: sha(repo / rel) for rel in runtime_sources}
    a_receipt = json.loads((a_root / "execution.json").read_text())
    b_receipt = json.loads((b_root / "execution.json").read_text())
    if a_receipt["binary_hashes"] != b_receipt["binary_hashes"]:
        raise ValueError("P25_A_B_RUNTIME_BINARY_DIFFERENCE")
    if a_receipt.get("skf_enabled") is not False or b_receipt.get("skf_enabled") is not True:
        raise ValueError("P25_AB_SKF_SWITCH_CONTRACT_FAILED")

    historical_contract = json.loads(p10_contract.read_text())
    fixed_evaluation = {
        "origin_unix_s": historical_contract["evaluation_origin_unix_s"],
        "fit_interval_s": historical_contract["fit_interval"],
        "fit_rotation": historical_contract["fit_transform_rotation"],
        "fit_translation_m": historical_contract["fit_transform_translation_m"],
        "contract": historical_contract["evaluation_contract"],
        "source_sha256": sha(p10_contract),
    }
    freeze = {
        "protocol": "P25_PRE_GT_AB_OUTPUT_FREEZE",
        "GT_LOADED": False,
        "A_run": str(a_root),
        "B_run": str(b_root),
        "input_hashes": input_hashes,
        "output_inventory": output_inventory,
        "parameter_parity": {
            "result": "PASS",
            "differences": differences,
            "ignored_run_specific_output_paths": sorted(ignored_output_paths),
        },
        "runtime_source_sha256": source_hashes,
        "runtime_binary_sha256": a_receipt["binary_hashes"],
        "runtime_devel_root": str(args.devel.resolve()),
        "fixed_p10_evaluation_contract": fixed_evaluation,
        "p4_visual_pose_parity": extraction["frozen_visual_pose_parity"],
        "camera_raw_sha256": extraction.get("camera_bag_expected_historical_sha256"),
        "camera_raw_hash_rechecked": False,
        "note": "All A/B outputs frozen before GT; P17 pose parity checked. Raw camera full-file SHA was not reread due prior external-drive I/O wait.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        raise ValueError("P25_PRE_GT_FREEZE_OUTPUT_ALREADY_EXISTS")
    args.output.write_text(json.dumps(freeze, indent=2) + "\n")
    print(json.dumps({"GT_LOADED": False, "A_files": len(output_inventory["A_SKF_OFF"]),
                      "B_files": len(output_inventory["B_SKF_ON"]),
                      "A_NDT_sha256": a_receipt["binary_hashes"]["dog_prior_map_ndt_node_cpp"]["sha256"],
                      "freeze": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
