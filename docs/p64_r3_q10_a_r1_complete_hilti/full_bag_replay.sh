#!/usr/bin/env bash
set -euo pipefail
d=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$d/../.."
asset=run/assets/exp18_corridor_lower_gallery_2.bag
printf '%s\n' '98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280  run/assets/exp18_corridor_lower_gallery_2.bag' | sha256sum --check
mkdir run/r1/full_bag
source /opt/ros/noetic/setup.bash
previous=docs/p64_r3_q10_public_mechanism_validation
dense=run/assets/exp18_corridor_lower_gallery_2_imu.txt
calib=run/assets/lidar_calibration.yaml
/usr/bin/python3 "$d/asset_info.py" "$asset" "$dense" run/r1/full_bag
count=$(python3 -c 'import json; print(json.load(open("run/r1/full_bag/asset_info.json"))["causally_deskewable_after_anchor"])')
/usr/bin/python3 "$previous/prepare_exp18.py" sensors "$asset" "$dense" "$calib" run/r1/full_bag/input --frames "$count"
/usr/bin/python3 "$d/bag_from_prepared.py" run/r1/full_bag/input run/r1/full_bag/rviz_exp18.bag
