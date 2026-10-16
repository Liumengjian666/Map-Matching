#!/usr/bin/env python3
"""Freeze P46-only sample provenance before any GT access."""
import csv
import hashlib
import json
import pathlib
import sys


def sha256(path):
    h = hashlib.sha256()
    with pathlib.Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_csv(path):
    with pathlib.Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fields):
    with pathlib.Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main():
    if len(sys.argv) != 5:
        raise SystemExit("usage: freeze_samples.py DATASET_ROOT P46_RUN_DIR TILE_MANIFEST OUT_DIR")
    dataset = pathlib.Path(sys.argv[1]).resolve()
    run = pathlib.Path(sys.argv[2]).resolve()
    tile_manifest_path = pathlib.Path(sys.argv[3]).resolve()
    out = pathlib.Path(sys.argv[4]).resolve()
    if not dataset.is_dir() or not run.is_dir() or not tile_manifest_path.is_file():
        raise RuntimeError("dataset, P46 run, and P45 tile manifest must already exist")
    repo_root = pathlib.Path(__file__).resolve().parents[3]
    p46_freeze_path = repo_root / "docs/p46_newer_college_real_tile_switch/run_frozen_manifest.json"
    p46_freeze = json.loads(p46_freeze_path.read_text())
    if pathlib.Path(p46_freeze["run_dir"]).resolve() != run:
        raise RuntimeError("P46 run directory differs from the committed frozen manifest")
    for artifact in p46_freeze["runtime_outputs"]:
        path = pathlib.Path(artifact["path"]).resolve()
        if not path.is_file() or path.stat().st_size != int(artifact["size_bytes"]):
            raise RuntimeError("P46 frozen output is missing or changed in size: " + str(path))
        if sha256(path) != artifact["sha256"]:
            raise RuntimeError("P46 frozen output SHA256 mismatch: " + str(path))
    out.mkdir(parents=True, exist_ok=True)
    for name in ("sample_manifest.csv", "tile_catalog.csv", "sample_freeze.json"):
        if (out / name).exists():
            raise RuntimeError("refusing to overwrite frozen artifact: " + str(out / name))

    ndt = read_csv(run / "ndt_diagnostics.csv")
    local = read_csv(run / "local_map_diagnostics.csv")
    deskew = read_csv(run / "deskew_diagnostics.csv")
    oosm = read_csv(run / "oosm_statistics.csv")
    publisher = read_csv(run / "publisher_receipts.csv")
    ekf = read_csv(run / "ekf_prediction_diagnostics.csv")
    if not all(len(rows) == 1800 for rows in (ndt, local, deskew, oosm, publisher)):
        raise RuntimeError("P46 frozen logs must each contain exactly 1800 rows")
    rollback = [row for row in ekf if row["event_type"] == "OOSM_ROLLBACK"]
    if len(rollback) != 1800:
        raise RuntimeError("P46 EKF lineage must contain exactly 1800 OOSM_ROLLBACK snapshots")
    if [int(row["oosm_frame_index"]) for row in rollback] != list(range(1800)):
        raise RuntimeError("P46 rollback snapshots do not map one-to-one to frame indices 0..1799")
    if [int(row["frame_index"]) for row in ndt] != list(range(1, 1801)):
        raise RuntimeError("P46 NDT frames are not a canonical 1..1800 sequence")
    if [int(row["scan_index"]) for row in publisher] != list(range(1, 1801)):
        raise RuntimeError("P46 publisher receipts are not a canonical 1..1800 sequence")
    if [int(row["frame_index"]) for row in oosm] != list(range(1, 1801)):
        raise RuntimeError("P46 OOSM statistics are not a canonical 1..1800 sequence")

    switches = [i + 1 for i, row in enumerate(local)
                if i > 0 and row["map_updated"] == "1"]
    if len(switches) != 5:
        raise RuntimeError("expected the five frozen real P46 tile switches; found " + str(switches))
    sample_indices = set(range(1, 11)) | set(range(15, 1801, 15))
    for index in switches:
        sample_indices.update(i for i in range(index - 2, index + 3) if 1 <= i <= 1800)
    ordered = sorted(sample_indices)

    tile_doc = json.loads(tile_manifest_path.read_text())
    catalog = {}
    for item in tile_doc["tiles"]:
        tile_path = pathlib.Path(item["tile_path"]).resolve()
        if not tile_path.is_file():
            raise RuntimeError("missing P45 tile: " + str(tile_path))
        actual = sha256(tile_path)
        if actual != item["sha256"]:
            raise RuntimeError("P45 tile hash mismatch: " + item["tile_id"])
        catalog[item["tile_id"]] = {
            "tile_id": item["tile_id"], "tile_path": str(tile_path),
            "point_count": int(item["point_count"]), "sha256": actual,
        }
    tile_rows = [catalog[key] for key in sorted(catalog)]
    tile_fields = ["tile_id", "tile_path", "point_count", "sha256"]
    write_csv(out / "tile_catalog.csv", tile_rows, tile_fields)

    def indexed(rows, key):
        result = {}
        for row in rows:
            k = row[key]
            if k in result:
                raise RuntimeError("duplicate key in frozen input: " + key + "=" + k)
            result[k] = row
        return result

    ndt_by_frame = indexed(ndt, "frame_index")
    receipt_by_index = indexed(publisher, "scan_index")
    rollback_by_frame = {str(i + 1): row for i, row in enumerate(rollback)}
    sample_rows = []
    selected_hashes = {}
    for index in ordered:
        key = str(index)
        d = ndt_by_frame[key]
        l = local[index - 1]
        s = deskew[index - 1]
        o = oosm[index - 1]
        r = receipt_by_index[key]
        rb = rollback_by_frame[key]
        scan_ns = int(r["scan_stamp_ns"])
        if int(l["scan_ns"]) != scan_ns:
            raise RuntimeError("local map scan time does not match exact publisher timestamp at frame " + key)
        if int(s["scan_index"]) != index:
            raise RuntimeError("Deskew row order mismatch at frame " + key)
        if o["oosm_result"] != "APPLIED":
            raise RuntimeError("P46 OOSM was not applied at sampled frame " + key)
        if int(o["frame_index"]) != index or int(d["frame_index"]) != index:
            raise RuntimeError("P46 frame-index join mismatch at frame " + key)
        if abs(float(d["lidar_header_stamp"]) - scan_ns * 1e-9) > 1e-6:
            raise RuntimeError("P46 NDT sensor timestamp differs from publisher receipt at frame " + key)
        if d["ndt_has_converged"] != "1" or d["cloud_size_after_filter"] != "1400":
            raise RuntimeError("P46 sampled frame lacks frozen CONTROL source/NDT at frame " + key)
        if l["coverage_status"] != "COVERAGE_PASS" or int(l["target_points"]) <= 0:
            raise RuntimeError("P46 sampled frame has invalid local target at frame " + key)
        ids = l["loaded_tile_ids"].split(";") if l["loaded_tile_ids"] else []
        if not ids or any(tile_id not in catalog for tile_id in ids):
            raise RuntimeError("P46 active tile IDs do not resolve through P45 manifest at frame " + key)
        rollback_stamp = float(o["rollback_stamp"])
        if abs(float(rb["rollback_stamp"]) - rollback_stamp) > 1e-7:
            raise RuntimeError("OOSM rollback snapshot does not match OOSM record at frame " + key)
        if int(rb["oosm_frame_index"]) != index - 1 or int(rb["ndt_update_count"]) != index - 1:
            raise RuntimeError("EKF rollback snapshot is not the pre-observation state for frame " + key)
        if abs(float(rb["ndt_measurement_stamp"]) - scan_ns * 1e-9) > 1e-6:
            raise RuntimeError("EKF rollback event is not tied to the exact sampled scan at frame " + key)
        if abs(float(rb["state_stamp"]) - rollback_stamp) > 1e-7:
            raise RuntimeError("EKF rollback state timestamp differs from rollback contract at frame " + key)
        pcd = pathlib.Path(r["pcd_path"]).resolve()
        if not pcd.is_file():
            raise RuntimeError("sampled canonical PCD is absent: " + str(pcd))
        try:
            relative_pcd = pcd.relative_to(dataset)
        except ValueError as exc:
            raise RuntimeError("sampled PCD resolves outside frozen dataset root: " + str(pcd)) from exc
        scan_sec, scan_nsec = divmod(scan_ns, 1_000_000_000)
        if pcd.name != "cloud_%d_%09d.pcd" % (scan_sec, scan_nsec):
            raise RuntimeError("sample PCD filename does not encode its sensor header timestamp: " + str(pcd))
        pcd_hash = sha256(pcd)
        selected_hashes[str(scan_ns)] = pcd_hash

        row = {
            "frame_index": key, "scan_ns": scan_ns, "pcd_path": str(pcd),
            "pcd_relative_path": str(relative_pcd),
            "pcd_sha256": pcd_hash, "expected_source_hash": d["cloud_hash"],
            "scan_start": s["scan_start_stamp"], "scan_end": s["scan_end_stamp"],
            "initial_tx": d["initial_guess_tx"], "initial_ty": d["initial_guess_ty"],
            "initial_tz": d["initial_guess_tz"], "initial_qx": d["initial_guess_qx"],
            "initial_qy": d["initial_guess_qy"], "initial_qz": d["initial_guess_qz"],
            "initial_qw": d["initial_guess_qw"],
            "nominal_tx": d["raw_ndt_tx"], "nominal_ty": d["raw_ndt_ty"],
            "nominal_tz": d["raw_ndt_tz"], "nominal_qx": d["raw_ndt_qx"],
            "nominal_qy": d["raw_ndt_qy"], "nominal_qz": d["raw_ndt_qz"],
            "nominal_qw": d["raw_ndt_qw"],
            "ndt_iterations": d["ndt_iterations"], "ndt_fitness": d["ndt_fitness"],
            "rollback_stamp": rb["rollback_stamp"],
            "rollback_px": rb["p_x"], "rollback_py": rb["p_y"], "rollback_pz": rb["p_z"],
            "rollback_qx": rb["q_x"], "rollback_qy": rb["q_y"],
            "rollback_qz": rb["q_z"], "rollback_qw": rb["q_w"],
            "rollback_vx": rb["v_x"], "rollback_vy": rb["v_y"],
            "rollback_vz": rb["v_z"], "active_tile_ids": ";".join(ids),
            "target_points": l["target_points"], "map_update": l["map_updated"],
            "map_update_reason": l["update_reason"],
        }
        sample_rows.append(row)

    fields = list(sample_rows[0].keys())
    write_csv(out / "sample_manifest.csv", sample_rows, fields)
    freeze = {
        "task": "P47-LOCAL-NDT-DEGENERACY-COUPLED-SHADOW-R1",
        "sampled_before_gt_access": True,
        "scan_count": 1800,
        "sampling_rule": {
            "first_frames_1_to_10": True,
            "every_15th_one_based_frame": list(range(15, 1801, 15)),
            "real_tile_switches_one_based": switches,
            "switch_neighborhood_offsets": [-2, -1, 0, 1, 2],
            "merged_sample_count": len(ordered),
            "frame_indices_one_based": ordered,
        },
        "external_inputs": {
            "p46_run_frozen_manifest_sha256": sha256(p46_freeze_path),
            "p46_ndt_diagnostics_sha256": sha256(run / "ndt_diagnostics.csv"),
            "p46_local_map_diagnostics_sha256": sha256(run / "local_map_diagnostics.csv"),
            "p46_deskew_diagnostics_sha256": sha256(run / "deskew_diagnostics.csv"),
            "p46_oosm_statistics_sha256": sha256(run / "oosm_statistics.csv"),
            "p46_ekf_prediction_diagnostics_sha256": sha256(run / "ekf_prediction_diagnostics.csv"),
            "p46_publisher_receipts_sha256": sha256(run / "publisher_receipts.csv"),
            "p45_tile_manifest_sha256": sha256(tile_manifest_path),
            "sampled_pcd_sha256": selected_hashes,
        },
        "artifacts": {
            "sample_manifest_sha256": sha256(out / "sample_manifest.csv"),
            "tile_catalog_sha256": sha256(out / "tile_catalog.csv"),
        },
        "gt_read": False,
        "ndt_align_calls": 0,
        "source_reconstruction": "PENDING_EXACT_HASH_GATE",
    }
    with (out / "sample_freeze.json").open("x") as stream:
        json.dump(freeze, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print("P47_SAMPLES_FROZEN count=%d switches=%s first=%d last=%d gt_read=false" %
          (len(ordered), switches, ordered[0], ordered[-1]))


if __name__ == "__main__":
    main()
