#!/usr/bin/env bash
set -euo pipefail
d=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
s=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime
source_build=$(realpath "$1")
mkdir "$2"
b=$(realpath "$2")
cp -r "$source_build/hdl_localization" "$b/"
cp "$source_build/"{pose_estimator.cpp,scoring_profile.hpp,search_reuse.hpp,research_pipeline.cpp} "$b/"
python3 - "$b" <<'PY'
from pathlib import Path
import sys
b=Path(sys.argv[1])
def edit(f,a,c):
 p=b/f;s=p.read_text();assert s.count(a)==1,(f,a);p.write_text(s.replace(a,c))
edit('hdl_localization/pose_estimator.hpp','  unsigned map_updates = 0;', '  unsigned map_updates = 0;\n  Eigen::VectorXf logged_mean() const;')
edit('pose_estimator.cpp','/* getters */','Eigen::VectorXf PoseEstimator::logged_mean() const { return ukf->mean; }\n\n/* getters */')
edit('research_pipeline.cpp','    std::uint64_t ns=0,imu_ns=0,last_imu=0,last_scan=0;', '''    auto states=file(out/"states.csv","frame,stamp_ns,last_scan_ns,first_imu_ns,last_imu_ns,prev,pred,post");
    std::uint64_t ns=0,imu_ns=0,last_imu=0,last_scan=0;''')
edit('research_pipeline.cpp','      last_scan=ns; int imu_n=0;', '''      const auto prev_ns=last_scan;
      const auto previous=tracker.logged_mean();
      const auto first_imu=has_imu && imu_ns<=ns ? imu_ns : 0;
      last_scan=ns; int imu_n=0;''')
edit('research_pipeline.cpp','      tracker.correct(stamp,scan);','      const auto predicted=tracker.logged_mean();\n      tracker.correct(stamp,scan);')
edit('research_pipeline.cpp','      const Pose post=tracker.matrix(); const auto cov_after=tracker.covariance();', '''      const Pose post=tracker.matrix(); const auto cov_after=tracker.covariance();
      states<<frame<<','<<ns<<','<<prev_ns<<','<<first_imu<<','<<last_imu<<',';
      list(states,previous); states<<','; list(states,predicted); states<<',';
      list(states,tracker.logged_mean()); states<<'\\n';''')
PY
c=(-std=c++17 -O2 -DNDEBUG -fopenmp -I"$b" -I"$d/../p64_r3_q10_a_r1_complete_hilti" -I"$d/../p64_r2_q1_candidate_search" -I"$s/src/hdl_localization/include" -I"$s/src/ndt_omp/include" -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/opt/ros/noetic/include)
env -u LD_LIBRARY_PATH g++ "${c[@]}" -c "$b/pose_estimator.cpp" -o "$b/tracker.o"
env -u LD_LIBRARY_PATH g++ "${c[@]}" "$b/research_pipeline.cpp" "$b/tracker.o" -L"$s/devel/lib" "-Wl,-rpath,$s/devel/lib" -lndt_omp -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -lroscpp -lpcl_filters -lpcl_registration -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search -lcrypto -o "$b/trace"
