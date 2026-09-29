#!/usr/bin/env python3
"""Make an I6D parameter copy with the frozen official Corridor01 T_imu_lidar.

The I6C v2 input bundle remains byte-for-byte frozen. This derived parameter
copy changes only the final seven T_imu_lidar values because the frozen
runtime quaternion does not match the pinned official calibration.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
INPUT_DIR = ROOT / "results/p6_i6c_framework/input"
MANIFEST = INPUT_DIR / "input_manifest.txt"
PARAMS = INPUT_DIR / "params.txt"
EXTRINSICS = ROOT / "calibration/corridor01_extrinsics.yaml"
REPO = Path(__file__).resolve().parents[3]
OUTPUT = REPO / "src/dog_prior_map_fastlio2_frontend_exp/docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt"
EXPECTED_MANIFEST_SHA256 = "6d722ec6946570cc09d984c1a8ac7ebaaff799f012f9386ae169e3d47747a043"
EXPECTED_PARAMS_SHA256 = "fc7bb4c8758da222771aa0c9d18bb56e554a758a5bf181ea129ef9b86e3b4f48"
EXPECTED_EXTRINSICS_SHA256 = "59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    for path in (MANIFEST, PARAMS, EXTRINSICS):
        if not path.is_file():
            raise FileNotFoundError(path)
    if sha256(MANIFEST) != EXPECTED_MANIFEST_SHA256:
        raise RuntimeError("frozen Corridor01 v2 manifest identity mismatch")
    if sha256(PARAMS) != EXPECTED_PARAMS_SHA256:
        raise RuntimeError("frozen Corridor01 v2 params identity mismatch")
    if sha256(EXTRINSICS) != EXPECTED_EXTRINSICS_SHA256:
        raise RuntimeError("official Corridor01 extrinsics identity mismatch")

    values = [float(token) for token in PARAMS.read_text().split()]
    if len(values) != 27:
        raise RuntimeError("Corridor01 frozen parameter schema is not 27 scalars")
    frozen_rotation = Rotation.from_quat(values[23:27]).as_matrix()
    calibration = yaml.safe_load(EXTRINSICS.read_text(encoding="utf-8"))
    T_imu_lidar = np.asarray(calibration["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    raw_rotation = T_imu_lidar[:3, :3]
    orthogonality_defect = float(np.max(np.abs(raw_rotation.T @ raw_rotation - np.eye(3))))
    if orthogonality_defect > 2e-3 or np.linalg.det(raw_rotation) <= 0:
        raise RuntimeError("official laser_to_imu rotation exceeds P2C rounding allowance")
    u, _, vt = np.linalg.svd(raw_rotation)
    sign = np.eye(3)
    sign[2, 2] = np.linalg.det(u @ vt)
    projected_rotation = u @ sign @ vt
    official_quaternion_xyzw = Rotation.from_matrix(projected_rotation).as_quat()
    rotation_difference_deg = float(np.degrees(
        Rotation.from_matrix(frozen_rotation.T @ projected_rotation).magnitude()))
    official_translation = T_imu_lidar[:3, 3]
    if np.linalg.norm(np.asarray(values[20:23]) - official_translation) > 1e-9:
        raise RuntimeError("frozen and official T_imu_lidar translations differ")

    values[20:23] = official_translation.tolist()
    values[23:27] = official_quaternion_xyzw.tolist()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(" ".join(f"{value:.17g}" for value in values) + "\n",
                            encoding="ascii")
    provenance = args.output.with_suffix(args.output.suffix + ".provenance.txt")
    provenance.write_text(
        "dataset=SuperLoc Corridor01\n"
        f"frozen_v2_input_manifest={MANIFEST}\n"
        f"frozen_v2_input_manifest_sha256={sha256(MANIFEST)}\n"
        f"frozen_v2_params={PARAMS}\n"
        f"frozen_v2_params_sha256={sha256(PARAMS)}\n"
        f"official_extrinsics={EXTRINSICS}\n"
        f"official_extrinsics_sha256={sha256(EXTRINSICS)}\n"
        f"laser_to_imu_rotation_orthogonality_defect_before_projection={orthogonality_defect:.12g}\n"
        "projection=nearest_proper_SO3_by_SVD; documented P2C calibration rule\n"
        f"frozen_vs_official_T_imu_lidar_rotation_difference_deg={rotation_difference_deg:.12g}\n"
        f"T_imu_lidar_translation_xyz={';'.join(f'{x:.17g}' for x in official_translation)}\n"
        f"T_imu_lidar_quaternion_xyzw={';'.join(f'{x:.17g}' for x in official_quaternion_xyzw)}\n"
        "modified_parameter_values=only final 7 T_imu_lidar scalars; frozen v2 input files unchanged\n"
        f"i6d_params_sha256={sha256(args.output)}\n",
        encoding="utf-8",
    )
    print(f"CORRIDOR01_I6D_PARAMS={args.output}")
    print(f"T_IL_ROTATION_DELTA_DEG={rotation_difference_deg:.9f}")
    print(f"I6D_PARAMS_SHA256={sha256(args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
