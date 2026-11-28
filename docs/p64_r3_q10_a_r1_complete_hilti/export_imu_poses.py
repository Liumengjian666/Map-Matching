"""Export full UKF poses at the original nanosecond times and physical IMU origin."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def export(log, calib, output):
    imu_from_lidar=np.array(json.loads(calib.read_text())["T_I_L"])
    lidar_from_imu=np.linalg.inv(imu_from_lidar)
    with log.open() as inp, output.open("w") as out:
        for r in csv.DictReader(inp):
            pose=np.array([[float(r[f"state{i}{j}"]) for j in range(4)] for i in range(4)])@lidar_from_imu
            sec,ns=divmod(int(r["stamp_ns"]),1_000_000_000)
            parameters=np.r_[pose[:3,3],Rotation.from_matrix(pose[:3,:3]).as_quat()]
            out.write(f"{sec}.{ns:09d} "+" ".join(f"{p:.12g}" for p in parameters)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("log","calibration","output"):
        p.add_argument(n,type=Path)
    a=p.parse_args()
    export(a.log,a.calibration,a.output)
