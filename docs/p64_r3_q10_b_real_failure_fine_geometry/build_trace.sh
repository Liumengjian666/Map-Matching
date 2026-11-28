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
edit('research_pipeline.cpp','||mode=="LOCAL_SHADOW","bad mode"','||mode=="LOCAL_SHADOW"||mode=="Q10B_ICP_CONTROL","bad mode"')
edit('research_pipeline.cpp','    auto states=file', '''    auto interventions=file(out/"interventions.csv","frame,stamp_ns,native_angle_deg,result,support,steps,wall_s");
    auto states=file''')
edit('research_pipeline.cpp','          auto assemble_fine=[&](){','''          if(mode=="Q10B_ICP_CONTROL") {
            const double jump=angle(initial,update);
            if(jump>15.) {
              trigger="ROTATION_JUMP_DIAGNOSTIC";
              const auto event=out/("pointplane_"+std::to_string(frame));
              fs::create_directory(event);
              const auto seedfile=event/"seed.txt", resultfile=event/"result.txt";
              std::ofstream s(seedfile);
              s<<std::setprecision(17)<<tracker_state_previous<<'\\n';s.close();
              const auto command="OPENBLAS_NUM_THREADS=1 timeout 4s /usr/bin/python3 "+quote(helper)+" "+
                quote(fs::absolute(dir/scan_file))+" "+quote(seedfile)+" "+quote(resultfile)+
                " > "+quote(event/"job.log")+" 2>&1";
              const auto start=Clock::now();
              int support=0,steps=0;
              if(std::system(command.c_str())==0) {
                Pose candidate;
                std::ifstream result(resultfile);
                bool ok=bool(result>>support>>steps);
                for(int a=0;a<4;++a)for(int c=0;c<4;++c)ok=ok&&bool(result>>candidate(a,c));
                if(ok&&support>=30&&candidate.allFinite()){
                  update=candidate;proposal="ICP_CONTROL";actual=proposal;
                }
              }
              interventions<<frame<<','<<ns<<','<<jump<<','<<actual<<','<<support<<','<<steps<<','<<elapsed(start)<<'\\n';
            }
            decision_s=elapsed(begun);
            return true;
          }
          auto assemble_fine=[&](){''')
edit('research_pipeline.cpp','      const auto previous=tracker.logged_mean();', '''      const auto previous=tracker.logged_mean();
      const auto tracker_state_previous=tracker.matrix();''')

PY
c=(-std=c++17 -O2 -DNDEBUG -fopenmp -I"$b" -I"$d/../p64_r3_q10_a_r1_complete_hilti" -I"$d/../p64_r2_q1_candidate_search" -I"$s/src/hdl_localization/include" -I"$s/src/ndt_omp/include" -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/opt/ros/noetic/include)
env -u LD_LIBRARY_PATH g++ "${c[@]}" -c "$b/pose_estimator.cpp" -o "$b/tracker.o"
env -u LD_LIBRARY_PATH g++ "${c[@]}" "$b/research_pipeline.cpp" "$b/tracker.o" -L"$s/devel/lib" "-Wl,-rpath,$s/devel/lib" -lndt_omp -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -lroscpp -lpcl_filters -lpcl_registration -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search -lcrypto -o "$b/trace"
