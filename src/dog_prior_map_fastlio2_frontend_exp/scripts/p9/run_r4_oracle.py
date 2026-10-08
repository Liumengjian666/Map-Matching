#!/usr/bin/env python3
"""Isolated oracle runner/prefix count gate. Evidence builders do not import this file."""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np
from scipy.spatial.transform import Rotation

import p9_r4_contract as c
from p9_r4_oracle_labels import FrozenGeometry, extract
sys.path.insert(0, str(c.HERE.parent))

ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1",
           OMP_NUM_THREADS="1", LD_LIBRARY_PATH="/lib/x86_64-linux-gnu")
ORACLE = c.OUT / "oracle"


def run_logged(command, path):
    with path.open("w") as log:
        process = subprocess.Popen(list(map(str, command)), cwd=c.ROOT, env=ENV, text=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1)
        for line in process.stdout:
            log.write(line); log.flush(); print(line, end="", flush=True)
        c.require(process.wait() == 0, "engine failure; partial artifacts preserved: " + str(path))


def xyzq(row, prefix):
    return ";".join(row[prefix + key] for key in ("x", "y", "z", "qx", "qy", "qz", "qw"))


def pose_errors(a, b):
    x, y = (np.fromstring(value, sep=";") for value in (a, b))
    c.require(x.size == y.size == 7 and np.isfinite(x).all() and np.isfinite(y).all(), "invalid start pose")
    return float(np.linalg.norm(x[:3]-y[:3])), float(np.degrees((Rotation.from_quat(x[3:]).inv()*Rotation.from_quat(y[3:])).magnitude()))


def historical_seed_parity(binary):
    from run_r2a_predictor_search import farthest
    cohort = c.ARCHIVE / "frozen/cohort_frozen.csv"
    path = ORACLE / "historical_generated_pool.csv"
    run_logged([binary, "--pool", cohort, c.ARCHIVE / "dual_u.csv", path], ORACLE / "seed_parity_stdout.log")
    generated = {(int(row["frame"]), int(row["seed_index"])): row for row in c.read_csv(path)}
    archived = [r for r in c.read_csv(c.ARCHIVE / "candidates.csv") if int(r["seed_index"]) < 263]
    c.require(len(archived) == len(generated) == 32*263, "historical seed parity count changed")
    results = []
    for old in archived:
        tx, seed = int(old["transaction_id"]), int(old["seed_index"])
        new = generated[tx, seed]
        for key in ("seed_domain", "seed_dx_m", "seed_dy_m", "seed_dz_m", "seed_roll_deg", "seed_pitch_deg", "seed_yaw_deg"):
            c.require(new[key] == old[key] if key == "seed_domain" else float(new[key]) == float(old[key]),
                      "historical seed geometry/order changed")
        dt, dr = pose_errors(new["start_pose_xyz_q_xyzw"], old["start_pose_xyz_q_xyzw"])
        c.require(dt <= 1e-5 and dr <= 1e-4, "historical generator start-pose parity failure")
        results.append(dict(frame=tx, seed_index=seed, translation_error_m=dt, rotation_error_deg=dr, parity="PASS"))
    c.write_csv(c.OUT / "original_seed_parity.csv", results)
    old_pool = c.ROOT / "docs/p9_r2a_predictor_conditioned_search/conditioned_proposals.csv"
    old_order = c.ROOT / "docs/p9_r2a_predictor_conditioned_search/probe_manifest.csv"
    c.pinned(old_pool); c.pinned(old_order)
    reference = {(int(r["frame"]),int(r["seed_index"])):r for r in c.read_csv(old_pool) if r["method"]=="COND_WEAK2"}
    order = defaultdict(list)
    for row in c.read_csv(old_order):
        if row["method"] == "COND_WEAK2" and int(row["probe_rank"]) <= 12:
            order[int(row["frame"])].append(int(row["seed_index"]))
    groups = defaultdict(list)
    for row in generated.values():
        old = reference[int(row["frame"]),int(row["seed_index"])]
        c.require(np.array_equal(np.fromstring(row["start_pose_matrix16"],sep=";"),
                                 np.fromstring(old["start_pose_matrix16"],sep=";")), "frozen R2A proposal carrier changed")
        groups[int(row["frame"])].append(row)
    for tx, rows in groups.items():
        c.require([seed for _, seed, _, _ in farthest(rows, 12)] == order[tx], "frozen R2A farthest-point order changed")
    return dict(original_seed_count=len(results), start_translation_max=max(r["translation_error_m"] for r in results),
                start_rotation_max=max(r["rotation_error_deg"] for r in results),
                original_seed_parity="PASS", candidate_generator_parity="PASS", new_ndt_calls=0)


def prepare(binary, library):
    receipt = json.loads((c.OUT / "oracle_parity_freeze.json").read_text())
    c.require(receipt["status"] == "PASS" and receipt["frames"] == 24 and receipt["major_ids"] == 22
              and c.digest(library) == receipt["frozen_geometry_library_sha256"]
              and c.digest(c.HERE / "p9_r4_oracle_labels.py") == receipt["extractor_sha256"],
              "exact historical label parity must pass first")
    c.require(not (c.OUT / "execution_manifest.json").exists(), "preparation already frozen")
    ORACLE.mkdir(exist_ok=True); (ORACLE / "runs").mkdir(exist_ok=True)
    parity = historical_seed_parity(binary)
    from p4_i3_visual_increment import requests, RUNTIME
    import sensor_msgs.point_cloud2 as pc2
    expected = json.loads((c.ROOT / "docs/p9_r3_visual_nonlocal_evidence/extraction_manifest.json").read_text())["input_sha256"]
    map_lineage = c.ROOT / "docs/p9_r1b_strong_attractor_closure/input_provenance.json"
    c.pinned(map_lineage)
    for name, sha in json.loads(map_lineage.read_text()).items():
        c.require(name not in expected or expected[name] == sha, "conflicting frozen input lineage")
        expected[name] = sha
    inputs = {}
    for path in (c.REGISTRATION, c.ARCHIVE / "dual_u.csv", c.MAP, RUNTIME):
        value = c.digest(path)
        c.require(str(path) in expected and value == expected[str(path)], "prior input lineage changed: " + str(path))
        inputs[str(path)] = value
    rows = c.read_csv(c.OUT / "heldout_ordered_pool.csv")
    selection = json.loads((c.OUT / "selection_freeze.json").read_text())
    c.require(c.digest(c.OUT / "heldout_ordered_pool.csv") == selection["artifacts"]["heldout_ordered_pool.csv"], "selection freeze changed")
    baseline = {int(row["transaction_id"]):row for row in c.read_csv(c.REGISTRATION)}
    wanted = {int(row["transaction_id"]) for row in rows}
    cache = Path(tempfile.mkdtemp(prefix="p9_r4_raw_xyzf."))
    assets = {}
    for request in requests():
        tx = int(request.transaction_id)
        if tx not in wanted:
            continue
        old = baseline[tx]; cloud = request.cloud_end_frame
        c.require(tx not in assets and request.map_frame == "floor01_map_h1" and
                  request.lidar_frame == "cmu_sp1_velodyne" and str(request.scan_end_ns)==old["stamp_ns"], "raw request identity/stamp mismatch")
        points = np.asarray(list(pc2.read_points(cloud, field_names=("x","y","z"), skip_nans=False)), dtype="<f4")
        c.require(points.shape == (int(cloud.width)*int(cloud.height),3), "raw source point decoder mismatch")
        path = cache / f"tx_{tx}.xyzf"
        points.tofile(path)
        assets[tx] = dict(raw_cloud_file=str(path), raw_source_sha256=c.digest(path), raw_point_count=len(points),
                         cloud_data_sha256=__import__("hashlib").sha256(bytes(cloud.data)).hexdigest())
    c.require(set(assets) == wanted, "selected raw requests missing")
    frames = []
    for row in rows:
        tx = int(row["transaction_id"]); old=baseline[tx]
        c.require(old["converged"]=="1" and old["status"]=="SUCCESS", "frozen nominal terminal invalid; no frame filtering allowed")
        frames.append(dict(pool_rank=row["pool_rank"], transaction_id=tx, frame_id=f"R4_TX{tx:04d}",
            time_s=int(old["stamp_ns"])*1e-9-1660857393.197807074, stamp_ns=old["stamp_ns"],
            initial_pose_xyz_q_xyzw=xyzq(old,"initial_"), raw_terminal_pose_xyz_q_xyzw=xyzq(old,"raw_"),
            saved_raw_pose_xyz_q_xyzw=xyzq(old,"raw_"), prepared_source_hash=old["source_cloud_hash"],
            prepared_source_point_count=old["source_points"], target_point_count=old["target_points"],
            selection_labels="LABEL_BLIND_HASH_POOL", segment="Floor01", wide_targeted=0, **assets[tx]))
    c.write_csv(c.OUT / "heldout_source_manifest.csv", frames)
    run_logged([binary,"--pool",c.OUT/"heldout_source_manifest.csv",c.ARCHIVE/"dual_u.csv",c.OUT/"conditioned_proposal_pool.csv"],
               c.OUT/"proposal_stdout.log")
    code = ("p9_r4_ndt.cpp","p9_r4_codegen.py","p9_r4_contract.py","p9_r4_oracle_labels.py","run_r4_oracle.py",
            "p9_ndt_energy_contract.cpp","p9_r2a_predictor_search.cpp","run_r2a_predictor_search.py")
    manifest=dict(start_sha=c.START_SHA, branch=c.BRANCH, state="PREPARED", build_type="Release", binary=str(binary),
        binary_sha256=c.digest(binary), oracle_library=str(library), oracle_library_sha256=c.digest(library),
        input_sha256=inputs, code_sha256={str(c.HERE/name):c.digest(c.HERE/name) for name in code},
        artifacts={name:c.digest(c.OUT/name) for name in ("heldout_ordered_pool.csv","heldout_source_manifest.csv",
                   "conditioned_proposal_pool.csv","oracle_parity.csv","original_seed_parity.csv")},
        raw_source_sha256={r["raw_cloud_file"]:r["raw_source_sha256"] for r in frames},
        original_seed_parity=parity, oracle_prefix_sizes=list(c.PREFIXES), oracle_calls_per_frame=263,
        candidate_calls_per_frame=12, gt_loaded=False, canonical_poses_used_for_proposals=False)
    c.save_json(c.OUT/"execution_manifest.json",manifest)
    print("R4_PREPARE_PASS pool160 seed_parityPASS candidate_parityPASS NEW_NDT_CALLS=0",flush=True)


def verify():
    manifest=json.loads((c.OUT/"execution_manifest.json").read_text())
    for key in ("input_sha256","code_sha256","raw_source_sha256"):
        for name,expected in manifest[key].items():
            c.require(c.digest(name)==expected,"prepared input/code changed: "+name)
    for name,expected in manifest["artifacts"].items():
        c.require(c.digest(c.OUT/name)==expected,"prepared artifact changed: "+name)
    c.require(c.digest(manifest["binary"])==manifest["binary_sha256"] and
              c.digest(manifest["oracle_library"])==manifest["oracle_library_sha256"],"Release binary changed")
    return manifest


def run_prefix():
    manifest=verify()
    c.require(not (c.OUT/"cohort_selection_freeze.json").exists(),"final prefix already frozen")
    frames=c.read_csv(c.OUT/"heldout_source_manifest.csv")
    geometry=FrozenGeometry(Path(manifest["oracle_library"]))
    all_labels=[];all_clusters=[];all_candidates=[];cost=[]
    previous=0
    for prefix in c.PREFIXES:
        batch=frames[previous:prefix];batch_path=ORACLE/f"batch_{prefix}.csv"
        c.require(not batch_path.exists(),"oracle batch was already started; refusing repeat")
        c.write_csv(batch_path,batch)
        tick=time.perf_counter()
        run_logged([manifest["binary"],"--oracle",c.MAP,batch_path,c.OUT/"conditioned_proposal_pool.csv",ORACLE/"runs"],
                   ORACLE/f"engine_{prefix}.log")
        for frame in batch:
            tx=int(frame["transaction_id"]);runs=c.read_csv(ORACLE/"runs"/f"tx_{tx}.csv")
            c.require(len(runs)==263 and all(r["source_hash_actual"]==frame["prepared_source_hash"] and
                      r["source_points"]==frame["prepared_source_point_count"] and r["target_points"]=="549606" for r in runs),"oracle row/input parity fail")
            clusters,label=extract(frame,runs,frame["raw_terminal_pose_xyz_q_xyzw"],geometry)
            all_labels.append(label);all_clusters.extend(clusters);all_candidates.extend(runs)
            print("R4_LABEL_COMPLETE",tx,label["label"],label["major_count"],flush=True)
        major=sum(r["label"]=="MAJOR" for r in all_labels);no_major=prefix-major
        cost.append(dict(prefix=prefix,major=major,no_major=no_major,new_calls=len(batch)*263,wall_seconds=time.perf_counter()-tick))
        for name,rows in (("oracle_candidates.csv",all_candidates),("oracle_clusters.csv",all_clusters),("oracle_labels.csv",all_labels)):
            c.write_csv(ORACLE/name,rows)
        c.write_csv(ORACLE/"prefix_counts.csv",cost)
        if major>=12 and no_major>=40:
            c.write_csv(c.OUT/"heldout_final_cohort.csv",[dict(transaction_id=f["transaction_id"]) for f in frames[:prefix]])
            c.save_json(c.OUT/"cohort_selection_freeze.json",dict(final_prefix=prefix,major=major,no_major=no_major,
                sufficient=True,selection="first sufficient 96/128/160 label COUNTS ONLY",visual_evidence_loaded=False,
                gt_loaded=False,final_cohort_sha256=c.digest(c.OUT/"heldout_final_cohort.csv"),
                oracle_artifact_sha256={name:c.digest(ORACLE/name) for name in ("oracle_candidates.csv","oracle_clusters.csv","oracle_labels.csv")},
                oracle_calls=prefix*263,oracle_wall_seconds=sum(r["wall_seconds"] for r in cost)))
            print("R4_COHORT_COUNTS_PASS",prefix,major,no_major,flush=True)
            return
        previous=prefix
    c.save_json(c.OUT/"cohort_selection_freeze.json",dict(final_prefix=160,major=major,no_major=no_major,sufficient=False,
        selection="96/128/160 label counts only",oracle_calls=160*263,gt_loaded=False,visual_evidence_loaded=False,
        oracle_wall_seconds=sum(r["wall_seconds"] for r in cost)))
    print("HELDOUT_ORACLE_COHORT_INSUFFICIENT",major,no_major,flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=("prepare","run-prefix"))
    parser.add_argument("--binary",type=Path)
    parser.add_argument("--library",type=Path)
    args=parser.parse_args()
    if args.stage=="prepare":
        c.require(args.binary is not None and args.library is not None,"binary/library required")
        prepare(args.binary,args.library)
    else:
        run_prefix()
