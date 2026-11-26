#!/usr/bin/env python3
"""Freeze Q1 inputs, or extract assessment poses in a separate post-seal process."""
import argparse
import csv
import gzip
import hashlib
import importlib.util
import json
from decimal import Decimal
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

DOCS = Path(__file__).resolve().parent
WORKSPACE = DOCS.parents[1]
R1 = Path("/home/jian/p64_r1_ws")
HARD = (Path.home() / ".local/share/Trash/files/Map-Matching-R14-first-loss-heldout"
        / "docs/p63_r10_hdl_imu_full_hard")
OMNI = (Path.home() / "桌面/先验地图定位创新研究工作区/实验记录"
        / "EXP-116_Omni全序列复用对照/docs/paper_research_control/exp_116_full_omni/FORMAL_A")
IILABS = (Path.home() / "rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1")
FEED = IILABS / "gate2_051_omni_sensor_feed"
MANIFEST = DOCS / "qualification_sample_manifest.csv"

SAMPLES = [
    ("H224", "A", "historical attitude harm despite high support"),
    ("H307", "A", "translation error crossed threshold just before failure"),
    ("H309", "A", "historical large harmful correction and high support"),
    ("H310", "A", "next frame remained severely wrong after onset"),
    ("H515", "A", "later attitude harm and lower support"),
    ("H2379", "A", "terminal divergence with native convergence and zero support"),
    ("O164", "B", "first frame of R1 O1 normal independent acquisition"),
    ("O910", "B", "first frame of R1 O2 normal independent acquisition"),
    ("O1550", "B", "first frame of R1 O3 normal independent acquisition"),
    ("O2450", "B", "first frame of R1 O4 normal independent acquisition"),
    ("H314", "C", "large beneficial correction soon after onset"),
    ("H533", "C", "large beneficial correction while still lost"),
]


def sha(path):
    with path.open("rb") as stream:
        return digest_stream(stream)


def digest_stream(stream):
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(block)
    return digest.hexdigest()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read_rows(path):
    open_ = gzip.open if path.suffix == ".gz" else open
    with open_(path, "rt", newline="") as stream:
        return list(csv.DictReader(stream))


def write_rows(path, rows):
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def historical_pcd_adapter():
    path = HARD / "tools/data.py"
    spec = importlib.util.spec_from_file_location("historical_pcd_adapter", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_xyzi(path, xyzi_bytes, count):
    header = ("VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
              f"COUNT 1 1 1 1\nWIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {count}\nDATA binary\n").encode("ascii")
    with path.open("xb") as stream:
        stream.write(header)
        stream.write(xyzi_bytes)


def raw_hard(frame, adapter):
    path = Path(frame["path"])
    assert sha(path) == frame["sha256"]
    xyz, _ = adapter.pcd_xyz(path)
    assert len(xyz) == frame["points"] and digest(xyz.tobytes()) == frame["xyz_sha256"]
    return adapter.xyzi_bytes(xyz), len(xyz), str(path), frame["sha256"]


def raw_omni(frame, stream):
    stream.seek(frame["offset"])
    blob = stream.read(frame["points"] * 16)
    assert len(blob) == frame["points"] * 16
    assert digest(blob) == frame["xyzi_sha256"]
    return blob, frame["points"], f"{FEED / 'scans_xyzi.bin'}@{frame['offset']}", frame["xyzi_sha256"]


def pose_from_native_row(row):
    guess = np.array([float(row[f"guess_{i}{j}"]) for i in range(4) for j in range(4)]).reshape(4, 4)
    final = np.eye(4)
    final[:3, 3] = [float(row[f"t{x}"]) for x in "xyz"]
    final[:3, :3] = Rotation.from_quat([float(row[f"q{x}"]) for x in "xyzw"]).as_matrix()
    assert np.isfinite(guess).all() and np.array_equal(guess[3], [0, 0, 0, 1])
    return guess, final


def verify_trace(trace, config):
    seal = json.loads((trace.parent / "ONLINE_SEAL.json").read_text())
    assert seal["test_reference_values_read"] is False
    for path in (trace, config):
        # Older P63 archived seals name uncompressed files. Match the pinned
        # archive digest separately and the original uncompressed identity here.
        key = path.stem if path.suffix == ".gz" else path.name
        expected = seal["files"][key]
        if path.suffix == ".gz":
            with gzip.open(path, "rb") as stream:
                assert digest_stream(stream) == expected
        else:
            assert sha(path) == expected
    params = json.loads(config.read_text())
    loc, map_node = params["hdl_localization_nodelet"], params["globalmap_server_nodelet"]
    assert loc["reg_method"] == "NDT_OMP" and loc["ndt_neighbor_search_method"] == "DIRECT7"
    assert loc["ndt_resolution"] == 1.0 and loc["downsample_resolution"] == 0.1
    assert map_node["downsample_resolution"] == 0.1 and not map_node["convert_utm_to_local"]
    assert loc["use_imu"] and loc["specify_init_pose"]
    return sha(trace), sha(config)


def numeric_text(matrix):
    return "\t".join(format(float(value), ".17g") for value in matrix.ravel())


def prepare(output, freeze):
    assert not (output / "online.tsv").exists(), "use a fresh output directory"
    if freeze:
        assert not MANIFEST.exists(), "qualification manifest already frozen"
    hard = json.loads((HARD / "INPUT_FREEZE.json").read_text())
    omni = json.loads((FEED / "INPUT.json").read_text())
    hard_frames = {f["raw_ordinal"]: f for f in hard["frames"]}
    omni_frames = {f["raw"]: f for f in omni["scans"]}
    hard_map = R1 / ".git/p64_assets/r29_hard_tiles_exact_xyzi.pcd"
    omni_map = Path(omni["map_file"])
    assert sha(hard_map) == hard["map_sha256"]
    assert sha(omni_map) == omni["map_sha256"]
    hard_trace = HARD / "raw/full_01/NATIVE_NDT.csv.gz"
    omni_trace = OMNI / "NATIVE_NDT.csv"
    hard_trace_hash, hard_config_hash = verify_trace(hard_trace, hard_trace.parent / "EFFECTIVE_PARAMS.json")
    omni_trace_hash, omni_config_hash = verify_trace(omni_trace, OMNI / "EFFECTIVE_PARAMS.json")
    hard_rows = {int(row["stamp_ns"]): row for row in read_rows(hard_trace)}
    omni_rows = {int(row["stamp_ns"]): row for row in read_rows(omni_trace)}
    adapter = historical_pcd_adapter()
    manifest = []
    inputs = []
    output.mkdir(parents=True, exist_ok=True)
    with (FEED / "scans_xyzi.bin").open("rb") as stream:
        for sid, group, reason in SAMPLES:
            raw = int(sid[1:])
            if sid.startswith("H"):
                frame = hard_frames[raw]
                row = hard_rows[frame["stamp_ns"]]
                assert row["source_xyz_sha256"] == frame["source_xyz_sha256"]
                assert row["target_xyz_sha256"] == hard["target_xyz_sha256"]
                blob, count, asset, raw_sha = raw_hard(frame, adapter)
                map_path, map_sha = hard_map, hard["map_sha256"]
                dataset, initial, frame_id = "indoor_hard_01", "P63-R10 reference-assisted raw4 anchor and historical IMU/UKF", "depth_camera_link"
                trace_hash, config_hash = hard_trace_hash, hard_config_hash
            else:
                frame = omni_frames[raw]
                row = omni_rows[frame["stamp_ns"]]
                assert row["target_xyz_sha256"] == "a727d77c2520b9b4e4414ee3b62a564419852bfab89bf94eef1f9ce1ddef620c"
                blob, count, asset, raw_sha = raw_omni(frame, stream)
                map_path, map_sha = omni_map, omni["map_sha256"]
                dataset, initial, frame_id = "nav_a_omni", "EXP-116 A reference-assisted raw13 anchor and historical IMU/UKF", omni["frame_id"]
                trace_hash, config_hash = omni_trace_hash, omni_config_hash
            assert row["map_frozen"] == "1"
            source = output / f"{sid}.pcd"
            assert all(not any(c.isspace() for c in str(path)) for path in (source, map_path))
            write_xyzi(source, blob, count)
            guess, final = pose_from_native_row(row)
            inputs.append(f"{sid}\t{map_path}\t{source}\t{numeric_text(guess)}\t{numeric_text(final)}")
            manifest.append({
                "sample_id": sid, "group": group, "dataset": dataset, "raw": raw,
                "stamp_ns": frame["stamp_ns"], "reason": reason, "protocol": "A",
                "initialization": initial, "frame_id": frame_id, "raw_asset": asset,
                "raw_sha256": raw_sha, "prepared_pcd_sha256": sha(source),
                "expected_source_points": row["source_points"],
                "expected_source_xyz_sha256": row["source_xyz_sha256"],
                "map_pcd": str(map_path), "map_sha256": map_sha,
                "expected_target_points": row["target_points"],
                "expected_target_xyz_sha256": row["target_xyz_sha256"],
                "history_trace_sha256": trace_hash, "history_config_sha256": config_hash,
                "history_probability": row["probability"], "history_iterations": row["iterations"],
                "history_converged": row["converged"],
            })
    if freeze:
        write_rows(MANIFEST, manifest)
    else:
        assert read_rows(MANIFEST) == [{key: str(value) for key, value in row.items()} for row in manifest]
    with (output / "online.tsv").open("x", encoding="utf-8") as stream:
        stream.write("\n".join(inputs) + "\n")


def read_hard_reference():
    path = Path.home() / "rosbag/public_datasets/koide_p62_r29/gt/gt/traj_lidar_indoor_hard_01.txt"
    assert sha(path) == "3696eddcbe884af197b13b8205952793b55c37d9093ab95b96a781ae56c6eeea"
    times, values = [], []
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        stamp, *fields = line.split()
        times.append(int(Decimal(stamp) * 10**9))
        values.append([float(item) for item in fields])
    return np.array(times, dtype=np.int64), np.array(values)


def read_omni_reference(last_query):
    import rosbag

    path = IILABS / "ground-truth/benchmark/livox_mid-360/nav_a_omni/raw_data.bag"
    assert sha(path) == "4d2cb85d45765292f3c101d5d3ad39630138c125bc6c6536201fe803d3d49ea1"
    times, values = [], []
    with rosbag.Bag(str(path), "r") as bag:
        for _, msg, _ in bag.read_messages(topics=["/natnet_ros/eve/pose"]):
            assert msg.header.frame_id == "world"
            stamp = msg.header.stamp.to_nsec()
            pos, quat = msg.pose.position, msg.pose.orientation
            times.append(stamp)
            values.append([pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w])
            if stamp > last_query:
                break
    return np.array(times, dtype=np.int64), np.array(values)


def interpolate(times, track, stamp, omni):
    assert len(times) == len(track) and np.all(np.diff(times) > 0)
    right = min(int(np.searchsorted(times, stamp)), len(times) - 1)
    left = right if times[right] == stamp or not right or stamp > times[-1] else right - 1
    tl, tr = int(times[left]), int(times[right])
    # P63-R10 permits near endpoints, IILABS GATE2-052 requires bracketing.
    if ((omni and not times[0] <= stamp <= times[-1]) or
            tr - tl > 150_000_000 or min(abs(stamp - tl), abs(stamp - tr)) > 50_000_000):
        return None
    lo, hi = track[left], track[right]
    alpha = (stamp - tl) / (tr - tl) if right != left else 0
    rotation = Rotation.from_quat(lo[3:]) if right == left else Slerp(
        [0, 1], Rotation.from_quat([lo[3:], hi[3:]]))([alpha])[0]
    pose = np.eye(4)
    pose[:3, :3] = rotation.as_matrix()
    pose[:3, 3] = (1 - alpha) * lo[:3] + alpha * hi[:3]
    if omni:
        origin = np.array([1.4487025737762451, -3.089052200317383, 0.09234485030174255])
        lever_arm = np.array([0.0, 0.0, 0.4612])
        pose[:3, 3] += -origin + rotation.apply(lever_arm)
    return pose


def references(output, seal_path):
    # A separate invocation validates immutable scoring/candidate artifacts first.
    assert not output.exists(), "reference output already exists"
    seal = json.loads(seal_path.read_text())
    required = {"official/grid.csv", "official/modes.csv", "official/fit.csv", "input/online.tsv"}
    assert required.issubset(seal["files"]), "incomplete online seal"
    for name, expected in seal["files"].items():
        path = seal_path.parent / name
        assert sha(path) == expected, path
    manifest = read_rows(MANIFEST)
    input_ids = [line.split()[0] for line in (seal_path.parent / "input/online.tsv").read_text().splitlines()]
    assert input_ids == [row["sample_id"] for row in manifest]
    omni_latest = max(int(row["stamp_ns"]) for row in manifest if row["sample_id"].startswith("O"))
    tracks = {"H": read_hard_reference(), "O": read_omni_reference(omni_latest)}
    with output.open("x", encoding="utf-8") as stream:
        for row in manifest:
            sid = row["sample_id"]
            pose = interpolate(*tracks[sid[0]], int(row["stamp_ns"]), sid.startswith("O"))
            if pose is not None:
                stream.write(f"{sid}\t{numeric_text(pose)}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prep = commands.add_parser("prepare")
    prep.add_argument("--output", type=Path, default=WORKSPACE / "run/input")
    prep.add_argument("--freeze", action="store_true")
    evaluate = commands.add_parser("references")
    evaluate.add_argument("--output", type=Path, default=WORKSPACE / "run/reference.tsv")
    evaluate.add_argument("--online-seal", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.output.resolve(), args.freeze)
    else:
        references(args.output.resolve(), args.online_seal.resolve())


if __name__ == "__main__":
    main()
