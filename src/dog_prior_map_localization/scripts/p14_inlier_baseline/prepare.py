#!/usr/bin/env python3
"""Freeze reused assets and run the author's unchanged synthetic fixture function."""
import argparse
import ast
import csv
import hashlib
import json
import pathlib
import subprocess
import numpy as np
import yaml

PIN = "515759c00fe1e2f6d4d4a005e8c3a6b3f71d17ac"
FIRST_SIX = [16, 45, 124, 708, 1285, 1891]


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for part in iter(lambda: f.read(1024 * 1024), b""):
            h.update(part)
    return h.hexdigest()


def author_function(path, name):
    tree = ast.parse(path.read_text())
    fn = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)
    fn.decorator_list = []  # pytest is unnecessary to call the exact fixture body
    scope = {"np": np}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), "exec"), scope)
    return scope[name]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=pathlib.Path, required=True)
    parser.add_argument("--author", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()
    actual = subprocess.check_output(["git", "-C", str(args.author), "rev-parse", "HEAD"], text=True).strip()
    assert actual == PIN
    assert not subprocess.check_output(["git", "-C", str(args.author), "status", "--porcelain"], text=True)
    args.out.mkdir(parents=True, exist_ok=True)
    if (args.out / "freeze.json").exists():
        raise RuntimeError("Freeze exists; do not overwrite experiment inputs")
    docs = args.repo / "docs"
    refs = json.loads((docs / "p11_mature_solid_prior_map_integration_r1/input_freeze.json").read_text())["reference_pcds"]
    stamps = list(csv.DictReader((docs / "p11_mature_solid_prior_map_integration_r1/database_build.csv").open()))
    samples = list(csv.DictReader((docs / "p12_dcreg_solid_minimal_integration_r1/samples.csv").open()))
    hashes = {int(row["frame_index"]): row for row in csv.DictReader((docs / "p12_dcreg_solid_minimal_integration_r1/extraction.csv").open())}
    assert len(refs) == 389 and len(samples) == 24
    reference_rows = []
    for i, (item, stamp) in enumerate(zip(refs, stamps)):
        assert sha(item["path"]) == item["sha256"] and int(stamp["id"]) == i
        reference_rows.append([i, stamp["stamp_sec"], item["path"], item["sha256"]])
    order = {frame: i for i, frame in enumerate(FIRST_SIX)}
    samples.sort(key=lambda row: (0, order[int(row["frame_index"])]) if int(row["frame_index"]) in order else (1, int(row["frame_index"])))
    query_rows = []
    for item in samples:
        frame = int(item["frame_index"])
        assert sha(item["source_pcd"]) == hashes[frame]["sha256"]
        assert item["stamp_ns"] == hashes[frame]["stamp_ns"]
        query_rows.append([frame, item["stamp_ns"], item["source_pcd"], hashes[frame]["sha256"]])
    for name, rows in [("references.csv", reference_rows), ("queries.csv", query_rows)]:
        with (args.out / name).open("w", newline="") as f:
            writer = csv.writer(f, lineterminator="\n")
            writer.writerow(["id", "stamp", "path", "sha256"])
            writer.writerows(rows)
    fixture = author_function(args.author / "tests/conftest.py", "synthetic_cloud")(np.random.default_rng(42))
    fixture.astype("<f8").tofile(args.out / "author_synthetic_f64.bin")
    cfg = yaml.safe_load((args.author / "inlier/config/default.yaml").read_text())
    cfg["gicp"]["skip"] = True
    cfg["verify"]["seed"] = 0
    frozen = {
        "protocol": "P14_AUTHOR_CPP_DEFAULT_YAML_SINGLE_SCAN_V1",
        "upstream_sha": actual,
        "config": cfg,
        "first_six": FIRST_SIX,
        "query_order": [row[0] for row in query_rows],
        "reference_count": 389,
        "query_count": 24,
        "omp_threads": 1,
        "backend": "DIRECT_AUTHOR_CPP_STATIC_LIBRARY",
        "preprocessing": "0.5m first-point voxel, original point order; C++ compatibility adapter checked against author function",
        "transform": "p_reference = T_sensor * p_query; no map-frame pose conversion",
        "reference_encoding_passes": 1,
        "after_six_gate": "continue only if at least one positive MINT and BEAM candidate exists; no token/positive candidate after six => stop",
        "gicp": "NOT_RUN (optional; no full-cloud registration calls)",
        "multiframe_accumulation": False,
        "gt_loaded": False,
        "position_accuracy": "NOT_EVALUATED_NO_INDEPENDENT_LABELS",
        "inputs": {name: sha(args.out / name) for name in ["references.csv", "queries.csv", "author_synthetic_f64.bin"]},
        "author_default_yaml_sha256": sha(args.author / "inlier/config/default.yaml"),
    }
    (args.out / "freeze.json").write_text(json.dumps(frozen, indent=2) + "\n")
    print("FREEZE_PASS references=389 queries=24 first_six=" + str(FIRST_SIX))


if __name__ == "__main__":
    main()
