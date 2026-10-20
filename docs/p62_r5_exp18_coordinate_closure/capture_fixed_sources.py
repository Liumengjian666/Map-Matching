#!/usr/bin/env python3
"""Bounded source snapshots, fixed commits where available. No code is run."""
import hashlib
import json
import argparse
from pathlib import Path
import requests

ROOT = Path(__file__).resolve().parent
RAW = 'https://raw.githubusercontent.com/'
SOURCES = {
    'hesai_issue4_fix.xacro': RAW+'Hilti-Research/hesai_description/64633725c701c2763c5787257f4e32b7bc3067d9/urdf/hesai.urdf.xacro',
    'hesai_parametric.xacro': RAW+'Hilti-Research/hesai_description/ad175515d1d1ad4f5a0905b4fb57770d635928c5/urdf/hesai.urdf.xacro',
    'alphasense.xacro': RAW+'Hilti-Research/alphasense_description/80c1b8675b3b9e03f9de9d1b5f420e5225f67a50/urdf/alphasense.urdf.xacro',
    'mola_hilti2022.sh': RAW+'MOLAorg/mola_lidar_odometry/6e905f0dcec1221be94b747850fba09435db2605/scripts/lib/profiles/hilti2022.sh',
    'mola_rosbag1.yaml': RAW+'MOLAorg/mola_lidar_odometry/6e905f0dcec1221be94b747850fba09435db2605/ros2-launchs/lidar_odometry_from_rosbag1.yaml',
    'official_issue20.html': 'https://github.com/Hilti-Research/hilti-slam-challenge-2022/issues/20',
    'official_issue21.html': 'https://github.com/Hilti-Research/hilti-slam-challenge-2022/issues/21',
    'official_issue23.html': 'https://github.com/Hilti-Research/hilti-slam-challenge-2022/issues/23',
}
CONSUMER_SOURCES = {
    'mola_actual_rosbag1_launch.yaml': RAW+'MOLAorg/mola_lidar_odometry/6e905f0dcec1221be94b747850fba09435db2605/mola-cli-launchs/lidar_odometry_from_rosbag1.yaml',
    'mola_Rosbag1Dataset.cpp': RAW+'MOLAorg/mola_input_rosbag1/d1079e427343b5eff108dce4e9ff34008aadda15/src/Rosbag1Dataset.cpp',
    'mola_Rosbag1Dataset.h': RAW+'MOLAorg/mola_input_rosbag1/d1079e427343b5eff108dce4e9ff34008aadda15/include/mola_input_rosbag1/Rosbag1Dataset.h',
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--consumer', action='store_true')
    args = parser.parse_args()
    output = ROOT/'sources'
    output.mkdir(exist_ok=True)
    records = []
    for name, url in (CONSUMER_SOURCES if args.consumer else SOURCES).items():
        row = {'url': url, 'path': str(output/name)}
        try:
            with requests.get(url, timeout=(10, 30), stream=True) as response:
                row['http_status'] = response.status_code
                response.raise_for_status()
                content = bytearray()
                for chunk in response.iter_content(65536):
                    content.extend(chunk)
                    if len(content)>3000000:
                        raise RuntimeError('3MB per-source limit')
            with (output/name).open('xb') as handle:
                handle.write(content)
            row.update(status='CAPTURED', bytes=len(content), sha256=hashlib.sha256(content).hexdigest())
        except (requests.RequestException, RuntimeError) as error:
            row.update(status='FAILED_NOT_EVIDENCE', error=str(error))
        records.append(row)
        print(name, row['status'], flush=True)
    manifest = 'consumer_source_manifest.json' if args.consumer else 'source_manifest.json'
    with (ROOT/manifest).open('x') as handle:
        json.dump(records, handle, indent=2)
        handle.write('\n')


if __name__=='__main__':
    main()
