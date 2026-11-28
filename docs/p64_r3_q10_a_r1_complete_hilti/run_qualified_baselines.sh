#!/usr/bin/env bash
set -euo pipefail
d=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(realpath "$d/../..")
cd "$root"
mapdir=$(realpath run/r1/full_map/products)
source_dir=$(realpath run/r1/full_bag/input)
receipts="${1:-run/r1/full_map/qualification/INITIAL_MAP_OVERLAP.csv}"
source /opt/ros/noetic/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
old=docs/p64_r3_q10_public_mechanism_validation
dense=run/assets/exp18_corridor_lower_gallery_2_imu.txt
sparse=run/assets/exp18_corridor_lower_gallery_2_imu_3dof.txt
if ! python3 - "$receipts" <<'PY'
import csv,sys
r=next(x for x in csv.DictReader(open(sys.argv[1])) if x["scan"]=="ROTATION_DESKEW" and x["map"]=="0P15_NDT")
sys.exit(not(float(r["nn_median_m"])<=.20 and int(r["within_0p30"])/int(r["query_points"])>=.50))
PY
then
    /usr/bin/python3 "$d/coordinate_calibration.py" "$mapdir/geometry_map.pcd" "$source_dir" "$dense" run/r1/calibration_isolated
    printf '%s\n' 'Identity gate failed. Review isolated calibration and independent scan overlay before accepting a fixed transform.'
    exit 3
fi
mkdir run/r1/qualified
for count in 10 100; do
   input=run/r1/qualified/input$count
   pair=run/r1/qualified/replay$count
   /usr/bin/python3 "$d/prepare_subset.py" "$source_dir" "$mapdir/map.pcd" "$input" "$count"
   if [ "$count" = 10 ]; then
       run/r1/equiv_build/fixed "$mapdir/map.pcd" "$input/scan_0000.pcd" "$input/anchor.tsv" >run/r1/qualified/fixed_ndt.csv
   fi
   bash "$d/replay_pair.sh" run/r1/equiv_build "$input" "$pair" 11743 "FULL_E57_EXP18_${count}_FRAMES"
done
/usr/bin/python3 "$d/short_map_support.py" run/r1/qualified/input100 run/r1/qualified/replay100/b run/r1/qualified/short_map_support.csv
count=$(wc -l <"$source_dir/scans.tsv")
input=run/r1/qualified/input_full
pair=run/r1/qualified/replay_full
/usr/bin/python3 "$d/prepare_subset.py" "$source_dir" "$mapdir/map.pcd" "$input" "$count"
bash "$d/replay_pair.sh" run/r1/equiv_build "$input" "$pair" 11743 FULL_E57_EXP18_FULL
/usr/bin/python3 "$old/evaluate_exp18.py" "$input" "$mapdir" "$pair/b" "$dense" "$sparse" run/r1/qualified/metrics --scope FULL_E57_IDENTITY_GEOMETRY_PASSED
/usr/bin/python3 "$d/export_imu_poses.py" "$pair/b/trajectory.csv" "$input/input_summary.json" run/r1/qualified/metrics/trajectory_imu_exact_ns.txt
/usr/bin/python3 "$d/reference_checks.py" "$dense" "$sparse" run/r1/qualified/metrics/trajectory_imu_exact_ns.txt run/r1/qualified/metrics
/usr/bin/python3 "$d/official_hilti_evaluation.py" run/hilti_official/evaluation-2022/evaluation.py run/r1/qualified/metrics/trajectory_imu.txt "$dense" "$sparse" run/r1/qualified/official_aligned
/usr/bin/python3 "$d/degeneracy_diagnostics.py" "$input" "$mapdir/map.pcd" run/r1/qualified/metrics "$pair/b" run/r1/qualified/detectors
