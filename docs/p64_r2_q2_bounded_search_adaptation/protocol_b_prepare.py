"""Freeze a bounded blind Omni query prefix using only its raw sensor bag and Diff map."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag
from scipy import ndimage
from scipy.signal import fftconvolve

DATA = Path("/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1")
MAP = DATA / "gate2_048_diff_map_formal_01/nav_a_diff_prior_voxel0p15.pcd"
SENSOR = DATA / "benchmark/livox_mid-360/livox_nav_a_omni_2025-02-05-16-30-32.bag"
MAP_HASH = "c72c5d6f19b622759f421f05c01c3ee09433ed9967a4b2df0f313129c744f158"
SENSOR_HASH = "7e10c5d139bc435c2066585c2f5d27726c40b13961a26e0236920ae62a3a1fd2"
FRAMES = 30
START_OFFSET_NS = 1_000_000_000
CELL_M = 0.25
YAW_STEP_DEG = 5
HEIGHT_M = 0.4612
LAYERS = [(0.15, 0.65), (0.65, 1.25), (1.25, 2.15)]
LIDAR = "/eve/lidar3d"
IMU = "/eve/livox/imu"
XYZI = np.dtype({"names": ["x", "y", "z", "intensity"], "formats": ["<f4"] * 4,
                 "offsets": [0, 4, 8, 12], "itemsize": 26})


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for piece in iter(lambda: stream.read(4 << 20), b""):
            digest.update(piece)
    return digest.hexdigest()


def write_pcd(path, scan):
    count = len(scan)
    header = (f"VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
              f"COUNT 1 1 1 1\nWIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {count}\nDATA binary\n").encode()
    path.write_bytes(header + scan.tobytes())


def raw_input(directory):
    scans = []
    imus = []
    first_lidar = None
    previous_imu = -1
    with rosbag.Bag(str(SENSOR), "r") as bag:
        for topic, m, _ in bag.read_messages(topics=[LIDAR, IMU]):
            stamp = m.header.stamp.to_nsec()
            if topic == IMU:
                assert m.header.frame_id == "livox_frame" and stamp > previous_imu
                previous_imu = stamp
                a, w = m.linear_acceleration, m.angular_velocity
                assert np.isfinite([a.x, a.y, a.z, w.x, w.y, w.z]).all()
                imus.append([stamp, a.x * 9.80665, a.y * 9.80665, a.z * 9.80665,
                             w.x, w.y, w.z])
                if len(scans) == FRAMES and stamp > scans[-1]["stamp_ns"]:
                    break
                continue
            if first_lidar is None:
                first_lidar = stamp
            if stamp < first_lidar + START_OFFSET_NS or len(scans) == FRAMES:
                continue
            assert m.header.frame_id == "eve/lidar3d" and m.point_step == 26 and not m.is_bigendian
            assert [(f.name, f.offset, f.datatype, f.count) for f in m.fields][:4] == [
                (name, j * 4, 7, 1) for j, name in enumerate(XYZI.names)]
            src = np.frombuffer(m.data, dtype=XYZI)
            ordered = np.ascontiguousarray(np.column_stack([src[n] for n in XYZI.names]), dtype="<f4")
            xyz = ordered[:, :3].astype(np.float64)
            cleaned = ordered[np.isfinite(xyz).all(axis=1) & (np.sum(xyz ** 2, axis=1) > .20 ** 2)]
            path = directory / f"scan_{len(scans):02d}.pcd"
            write_pcd(path, cleaned)
            scans.append({"stamp_ns": stamp, "lidar_seq": m.header.seq, "points": len(cleaned),
                          "file": path.name, "sha256": sha(path)})
            if len(scans) == 1:
                first_xyz = cleaned[:, :3].astype(np.float64)
    assert len(scans) == FRAMES
    start, stop = scans[0]["stamp_ns"], scans[-1]["stamp_ns"]
    saved = [r for r in imus if start < r[0] <= stop]
    assert saved and previous_imu > stop
    with (directory / "imu.tsv").open("w") as out:
        csv.writer(out, delimiter="\t", lineterminator="\n").writerows(saved)
    (directory / "scans.tsv").write_text("".join(f"{s['stamp_ns']} {s['file']}\n" for s in scans))
    return first_xyz, {"first_raw_lidar_stamp_ns": first_lidar, "scans": scans,
                       "imu_samples": len(saved)}


def map_xyz():
    header, xyz = MAP.read_bytes().split(b"DATA binary\n", maxsplit=1)
    assert b"FIELDS x y z\n" in header
    n = int(next(line.split()[1] for line in header.splitlines() if line.startswith(b"POINTS ")))
    assert len(xyz) >= n * 12 and not any(xyz[n * 12:]), "unexpected binary data tail"
    return np.frombuffer(xyz, dtype="<f4", count=n * 3).reshape(-1, 3).astype(np.float64)


def initial_guess(scan, world):
    # choose dominant low horizontal layer by unique XY-cell occupancy
    map_cells = np.floor(world[:, :2] / CELL_M).astype(int)
    low = world[:, 2] < 0.25
    zs = np.floor(world[low, 2] / 0.05).astype(int)
    density = np.unique(np.column_stack((map_cells[low], zs)), axis=0)
    z_bins, floor_counts = np.unique(density[:, 2], return_counts=True)
    ground = (z_bins[np.argmax(floor_counts)] + 0.5) * 0.05
    initial_height = ground + HEIGHT_M
    scan = scan[np.linalg.norm(scan[:, :2], axis=1) <= 12.0]
    map_min = map_cells.min(axis=0)
    size = tuple(map_cells.max(axis=0) - map_min + 1)
    images = []
    qlayers = []
    for lo, hi in LAYERS:
        layer = (world[:, 2] >= ground + lo) & (world[:, 2] < ground + hi)
        occupancy = np.zeros(size, dtype=float)
        occupied = map_cells[layer] - map_min
        occupancy[tuple(occupied.T)] = 1.0
        # distance transform gives bounded partial occupancy credit.
        images.append(np.exp(-0.5 * (ndimage.distance_transform_edt(occupancy == 0) * CELL_M / 0.20) ** 2))
        qlayers.append(scan[(scan[:, 2] + HEIGHT_M >= lo) & (scan[:, 2] + HEIGHT_M < hi), :2])

    scores = []
    queries_per_yaw = []
    half = int(np.ceil(12.0 / CELL_M)) + 1
    for degrees in range(-180, 180, YAW_STEP_DEG):
        theta = np.deg2rad(degrees)
        transform = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        combined = np.zeros(size, dtype=float)
        total = 0
        for query, image in zip(qlayers, images):
            relative = np.unique(np.rint((query @ transform.T) / CELL_M).astype(int), axis=0)
            mask = np.zeros((2 * half + 1, 2 * half + 1), dtype=float)
            mask[tuple((relative + half).T)] = 1.0
            total += len(relative)
            combined += fftconvolve(image, mask[::-1, ::-1], mode="same")
        assert total >= 30
        scores.append(combined / total)
        queries_per_yaw.append(total)
    scores = np.stack(scores)
    main = np.unravel_index(np.argmax(scores), scores.shape)
    yaw0, x0, y0 = main
    gx, gy = np.ogrid[:size[0], :size[1]]
    separation_xy = np.hypot(gx-x0, gy-y0) * CELL_M
    yaw_steps = np.minimum((np.arange(len(scores)) - yaw0) % len(scores),
                           (yaw0 - np.arange(len(scores))) % len(scores))
    separated = (separation_xy[None] >= 1.5) | (yaw_steps[:, None, None] * YAW_STEP_DEG >= 15)
    rival = np.unravel_index(np.argmax(np.where(separated, scores, -np.inf)), scores.shape)
    def describe(index):
        i, x, y = index
        return {"translation": [float((x + map_min[0] + 0.5)*CELL_M),
                                 float((y + map_min[1] + 0.5)*CELL_M), float(initial_height)],
                "yaw_deg": int(i) * YAW_STEP_DEG - 180, "occupancy_score": float(scores[index]),
                "rotated_query_cells": queries_per_yaw[i]}
    best, second = describe(main), describe(rival)
    margin = best["occupancy_score"] - second["occupancy_score"]
    return {"initialization_kind": "blind_first_scan_map_raster_correlation",
            "seed": best, "separated_rival": second, "absolute_margin": margin,
            "map_floor_z_m": float(ground), "occupancy_grid_shape": list(scores.shape),
            "qualifies_initialization": bool(best["occupancy_score"] >= 0.55 and margin >= 0.05)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    out = args.output.resolve()
    assert sha(MAP) == MAP_HASH and sha(SENSOR) == SENSOR_HASH
    out.mkdir()
    scan, metadata = raw_input(out)
    seed = initial_guess(scan, map_xyz())
    metadata.update(seed, map_file=str(MAP), map_sha256=MAP_HASH, sensor_file=str(SENSOR),
                    sensor_sha256=SENSOR_HASH, start_offset_ns=START_OFFSET_NS,
                    scan_list_sha256=sha(out/"scans.tsv"), imu_sha256=sha(out/"imu.tsv"))
    (out/"BLIND_INIT_RECEIPT.json").write_text(json.dumps(metadata, indent=2) + "\n")
    if seed["qualifies_initialization"]:
        h = seed["seed"]
        (out/"anchor.tsv").write_text(" ".join(map(str, [*h["translation"], h["yaw_deg"]])) + "\n")
    print(json.dumps({k: metadata[k] for k in seed}, indent=2))


if __name__ == "__main__":
    main()
