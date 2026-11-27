"""Prepare O164 and four earlier physical acquisitions without reading reference poses."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HOME = Path.home()
Q1 = HOME / "p64_r2_q1_ws"
DATA = HOME / "rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1"
FEED = DATA / "gate2_051_omni_sensor_feed"
MAP = DATA / "gate2_048_diff_map_formal_01/nav_a_diff_prior_voxel0p15.pcd"
TRACE = (HOME / "桌面/先验地图定位创新研究工作区/实验记录/EXP-116_Omni全序列复用对照/"
         "docs/paper_research_control/exp_116_full_omni/FORMAL_A")
RAW_IDS = (160, 161, 162, 163, 164)
RADIUS_M = 12.0
ARTIFICIAL_YAW_DEG = 5.0
MAP_SHA = "c72c5d6f19b622759f421f05c01c3ee09433ed9967a4b2df0f313129c744f158"


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def pose(position, orientation):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(orientation).as_matrix()
    transform[:3, 3] = position
    return transform


def xyz_transform(data, transform):
    out = data.copy()
    out[:, :3] = data[:, :3] @ transform[:3, :3].T + transform[:3, 3]
    return out


def map_xyz():
    assert digest(MAP) == MAP_SHA
    header, payload = MAP.read_bytes().split(b"DATA binary\n", 1)
    assert b"FIELDS x y z" in header
    points = int(next(row.split()[1] for row in header.splitlines() if row.startswith(b"POINTS ")))
    assert len(payload) >= points * 12 and not any(payload[points * 12:])
    return np.frombuffer(payload, dtype="<f4", count=3 * points).reshape(-1, 3)


def read_scans():
    # The scan table sits after a legacy reference-aided anchor in this manifest.
    # Decode only the table of byte ranges and acquisition heads.
    source = (FEED / "INPUT.json").read_text()
    start = source.index('"scans"')
    entries, _ = json.JSONDecoder().raw_decode(source[source.index("[", start):])
    by_id = {row["raw"]: row for row in entries}
    with (FEED / "scans_xyzi.bin").open("rb") as stream:
        result = []
        for raw_id in RAW_IDS:
            row = by_id[raw_id]
            stream.seek(row["offset"])
            payload = stream.read(16 * row["points"])
            assert hashlib.sha256(payload).hexdigest() == row["xyzi_sha256"]
            array = np.frombuffer(payload, dtype="<f4").reshape(-1, 4)
            assert hashlib.sha256(array[:, :3].tobytes()).hexdigest() == row["xyz_sha256"]
            result.append((row, array))
    return result


def pcd(path, cloud):
    data = np.ascontiguousarray(cloud[:, :3], dtype="<f4")
    h = (f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         f"WIDTH {len(data)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(data)}\nDATA binary\n")
    path.write_bytes(h.encode() + data.tobytes())


def prepare(out):
    seal = json.loads((TRACE / "ONLINE_SEAL.json").read_text())
    assert seal["test_reference_values_read"] is False
    for name in ("ONLINE_RESULTS.csv", "NATIVE_NDT.csv", "EFFECTIVE_PARAMS.json"):
        assert digest(TRACE / name) == seal["files"][name]
    q1_seal = json.loads((Q1 / "run/online_seal.json").read_text())
    q1_input = Q1 / "run/input/online.tsv"
    assert digest(q1_input) == q1_seal["files"]["input/online.tsv"]
    sample, map_path, cloud_path, *numbers = next(line.split() for line in q1_input.read_text().splitlines()
                                                   if line.startswith("O164\t"))
    assert sample == "O164" and Path(map_path) == MAP
    meta = Q1 / "docs/p64_r2_q1_candidate_search/qualification_sample_manifest.csv"
    original_case = next(row for row in csv.DictReader(meta.open()) if row["sample_id"] == sample)
    assert int(original_case["raw"]) == RAW_IDS[-1]
    assert digest(Path(cloud_path)) == original_case["prepared_pcd_sha256"]
    nominal = np.asarray(numbers[16:], dtype=float).reshape(4, 4)
    synthetic = nominal.copy()
    synthetic[:3, :3] = Rotation.from_euler("z", ARTIFICIAL_YAW_DEG, degrees=True).as_matrix() @ nominal[:3, :3]

    acquisitions = read_scans()
    history = {int(row["raw_ordinal"]): row for row in csv.DictReader((TRACE / "ONLINE_RESULTS.csv").open())
               if int(row["raw_ordinal"]) in RAW_IDS}
    assert len(history) == len(RAW_IDS)
    archived = []
    for row, array in acquisitions:
        h = history[row["raw"]]
        assert int(h["stamp_ns"]) == row["stamp_ns"]
        assert int(h["last_imu_stamp_ns"]) <= row["stamp_ns"]
        assert int(h["prior_ukf_correction_stamp_ns"]) < row["stamp_ns"]
        assert h["converged"] == "1" and h["core_finite"] == "True"
        assert int(h["input_points"]) == len(array)
        archived.append(pose([float(h[x]) for x in ("px", "py", "pz")],
                             [float(h[x]) for x in ("qx", "qy", "qz", "qw")]))
    h = history[RAW_IDS[-1]]
    ndt = pose([float(h[x]) for x in ("raw_tx", "raw_ty", "raw_tz")],
               [float(h[x]) for x in ("raw_qx", "raw_qy", "raw_qz", "raw_qw")])
    np.testing.assert_allclose(nominal, ndt, atol=1e-7, rtol=0)
    assert int(original_case["stamp_ns"]) == acquisitions[-1][0]["stamp_ns"]
    assert original_case["raw_sha256"] == acquisitions[-1][0]["xyzi_sha256"]
    np.testing.assert_array_equal(Path(cloud_path).read_bytes().split(b"DATA binary\n", 1)[1],
                                  acquisitions[-1][1].tobytes())
    relative = np.linalg.inv(archived[-1]) @ np.asarray(archived)
    np.testing.assert_allclose(relative[-1], np.eye(4), atol=1e-12, rtol=0)
    relative[-1] = np.eye(4)
    world = map_xyz()
    keep = np.linalg.norm(world - nominal[:3, 3], axis=1) <= RADIUS_M
    world = world[keep]
    local_nominal = xyz_transform(world, np.linalg.inv(nominal))
    local_synthetic = xyz_transform(world, np.linalg.inv(synthetic))
    np.testing.assert_allclose(np.linalg.norm(local_nominal, axis=1),
                               np.linalg.norm(local_synthetic, axis=1), rtol=1e-6)
    made = {}
    arrays = {"raw_ordinals": np.asarray(RAW_IDS), "stamp_ns": np.array([r["stamp_ns"] for r, _ in acquisitions]),
              "T_map_nominal": nominal, "T_map_control_yaw5": synthetic,
              "T_nominal_from_scan": relative, "T_map_ukf": np.asarray(archived),
              "map_local_nominal": local_nominal, "map_local_control": local_synthetic,
              "map_local_world": world}
    for j, (row, data) in enumerate(acquisitions):
        key = f"O{row['raw']}"
        arrays[f"scan_{key}"] = data
        aligned = xyz_transform(data, relative[j])
        arrays[f"aligned_{key}"] = aligned
        pcd(out / f"aligned_{key}.pcd", aligned)
        made[key] = {**row, "last_imu_stamp_ns": int(history[row["raw"]]["last_imu_stamp_ns"]),
                     "prior_ukf_correction_stamp_ns": int(history[row["raw"]]["prior_ukf_correction_stamp_ns"]),
                     "shift_m": float(np.linalg.norm(relative[j, :3, 3])),
                     "relative_rotation_deg": float(Rotation.from_matrix(relative[j, :3, :3]).magnitude()*180/np.pi)}
    np.testing.assert_array_equal(arrays["scan_O164"], arrays["aligned_O164"])
    for key in ("map_local_nominal", "map_local_control"):
        pcd(out / f"{key}.pcd", arrays[key])
    np.savez_compressed(out / "omni_observations.npz", **arrays)
    identity = {
        "omni_scan_feed": str(FEED / "scans_xyzi.bin"),
        "formal_A_trace": str(TRACE / "ONLINE_RESULTS.csv"),
        "formal_A_trace_sha256": seal["files"]["ONLINE_RESULTS.csv"],
        "q1_online_sha256": digest(q1_input),
        "map": str(MAP), "map_sha256": MAP_SHA, "radial_crop_m": RADIUS_M,
        "map_cropped_points": int(len(world)), "artificial_control_yaw_degrees_map_axis": ARTIFICIAL_YAW_DEG,
        "used_reference_values": False, "new_optimizations": 0, "trajectory_replays": 0,
        "acquisitions": made,
    }
    identity["outputs"] = {file.name: digest(file) for file in sorted(out.iterdir())}
    (out / "OMNI_INPUT_MANIFEST.json").write_text(json.dumps(identity, indent=2) + "\n")
    print(len(world), [(key, info["stamp_ns"], info["points"]) for key, info in made.items()])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir()
    prepare(args.output)


if __name__ == "__main__":
    main()
