#!/usr/bin/env python3
import importlib.util
import sys
from pathlib import Path

import numpy as np
import yaml


def main():
    package_root = Path(__file__).resolve().parents[1]
    runner_path = package_root / "scripts/p8/run_corridor01_official_frame_replay.py"
    spec = importlib.util.spec_from_file_location("corridor01_frame_replay", runner_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    workspace = Path(__file__).resolve().parents[3]
    config_path = workspace / "src/dog_prior_map_localization/config/datasets/superloc_corridor01.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    transforms, projections = module.load_frame_contract(config)

    assert config["dataset_name"] == "Corridor01"
    assert config["start_time"] == 67.0
    assert config["imu_preroll"] is True
    assert config["localization_mode"] is True
    assert config["initial_pose"]["interpretation"].startswith("T_world_imu")
    assert np.allclose(
        transforms["T_WORLD_LIDAR"],
        transforms["T_WORLD_IMU"] @ transforms["T_IMU_LIDAR"],
        atol=1e-12,
    )
    assert np.allclose(
        transforms["T_WORLD_CAMERA"],
        transforms["T_WORLD_IMU"] @ transforms["T_IMU_CAMERA"],
        atol=1e-12,
    )
    assert np.allclose(
        transforms["T_NORMALIZED_LIDAR"],
        transforms["T_NORMALIZED_WORLD"] @ transforms["T_WORLD_LIDAR"],
        atol=1e-9,
    )
    assert projections["T_IMU_LIDAR"]["projection_frobenius"] > 1e-4
    assert abs(np.linalg.det(transforms["T_IMU_LIDAR"][:3, :3]) - 1.0) < 1e-10

    epoch = module.resolve_bag_epoch(config)
    assert epoch["first_transaction_id"] == 666
    assert epoch["first_imu"]["header_stamp_ns"] == 1517157286165072000
    assert epoch["transactions"][0]["scan_start_ns"] == 1517157286155932903
    assert epoch["transactions"][0]["point_count"] == 29063
    assert len(epoch["transactions"]) == 99
    imu = module.validate_imu_inputs(config, epoch)
    assert imu["first_scan_pre_anchor"] is True
    assert imu["anchor_imu_sample_present"] is True
    assert imu["pre_scan_imu_stamp_ns"] <= imu["first_scan_start_ns"]
    assert imu["post_scan_imu_stamp_ns"] > imu["first_scan_start_ns"]
    assert imu["static_calibration_samples"] == 200

    print("P8_CORRIDOR01_FRAME_CONTRACT_PASS")
    print("first_tx=666 frames=99 first_imu_header_ns=1517157286165072000")
    print("camera_and_lidar_chains=PASS imu_preroll=PASS gt_used=false")
    return 0


if __name__ == "__main__":
    sys.exit(main())
