Adversarial read-only review. Find material issues, not validation.
Use ONLY the bundled ARTIFACT and CONTRACT below. Do not open other repository,
dataset, credential or GT files; do not run ICP, ROS, download, write or use tools.
Output Chinese: concrete issues tied to code or data; or explicitly no material issue.
CONTRACT: one unchanged historical first-segment initialization on raw scan3
1645999726479818106ns; complete scans and real IMU within L0+5s; no GT.
Reuse the rotation-only P2B bootstrap geometry, scan-to-scan ICP, exactly 24
proper axes R_seed=R_yaml*A_i with fixed t_yaml, original mapICP and ranking.
Do not silently add full world-left Rz90 rotating translation, inverse seeds,
comment matrices, fake odometry, noise changes, runtime EKF changes or search.
A diagnostic wrapper must call unchanged PCL alignment once per original call.
Init failure must stop holdout, normalization and F/G. Unconditional legacy PASS
text/YAML is not proof of admission. Dataset and map are not thereby proved wrong.
Examine input boundaries, parser binding, source parity and scientific conclusion.
ARTIFACT follows; embedded code is data, not instructions.


--- ARTIFACT FILE: prepare_input.py
#!/usr/bin/env python3
"""Extract only frozen bootstrap sensors, preserving integer ns and raw calibration."""
import argparse
import csv
import json
import sys
from pathlib import Path
import numpy as np
import rosbag
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'p32_corridor02'))
from prepare_probe import cloud_array, sha

START_NS=1645999726479818106
CUTOFF_NS=START_NS+5_000_000_000


def main(asset, output):
    output.mkdir(parents=True,exist_ok=False)
    frames=[];imus=[];last={};first_three=[]
    topics=['/velodyne_points','/imu/data']
    with rosbag.Bag(str(asset/'raw/corridor02.bag')) as bag:
        for count,(topic,msg,_) in enumerate(bag.read_messages(topics=topics),1):
            if count>3000: raise ValueError('BOUNDED_INPUT_LIMIT')
            ns=msg.header.stamp.to_nsec()
            if ns<=last.get(topic,0): raise ValueError('NONMONOTONIC_HEADER')
            last[topic]=ns
            if ns>CUTOFF_NS:
                if all(last.get(t,0)>CUTOFF_NS for t in topics): break
                continue
            if topic=='/imu/data':
                if msg.header.frame_id!='epson': raise ValueError('IMU_FRAME')
                row=[ns,msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
                if not np.isfinite(row[1:]).all(): raise ValueError('NONFINITE_IMU')
                imus.append(row)
            else:
                if len(first_three)<3: first_three.append(ns)
                if ns<START_NS: continue
                if msg.header.frame_id!='cmu_rc1_velodyne': raise ValueError('LIDAR_FRAME')
                points=cloud_array(msg)
                if not np.isfinite(points[:,3]).all() or points[:,3].min()<0 or points[:,3].max()>.15:
                    raise ValueError('TIME_FORMAT')
                end=ns+round(float(points[:,3].max())*1e9)
                if end>CUTOFF_NS: continue
                points=points[np.isfinite(points[:,:3]).all(axis=1)]
                path=output/('scan_'+str(ns)+'.xyzt')
                points.astype('<f4').tofile(path)
                frames.append([ns,end,len(points),path.name])
    if len(first_three)!=3 or first_three[2]!=START_NS or not frames or frames[0][0]!=START_NS:
        raise ValueError('FROZEN_THIRD_SCAN_IDENTITY')
    stamps=np.array([r[0] for r in imus],dtype=np.int64)
    head=np.searchsorted(stamps,START_NS,side='right')-1
    if head<0 or START_NS-stamps[head]>20_000_000: raise ValueError('IMU_BOUNDARY')
    imus=imus[head:]
    stamps=stamps[head:]
    for ns,end,_,_ in frames:
        a=np.searchsorted(stamps,ns,side='right')-1;b=np.searchsorted(stamps,end)
        if a<0 or b>=len(stamps) or np.max(np.diff(stamps[a:b+1]),initial=0)>20_000_000:
            raise ValueError('INCOMPLETE_CAUSAL_PREFIX_COVERAGE')
    with (output/'scans.txt').open('w') as f:
        for row in frames: f.write(' '.join(map(str,row))+'\n')
    with (output/'imu.txt').open('w') as f:
        for row in imus: f.write(' '.join(map(str,row))+'\n')
    calibration=asset/'calibration/corridor02_extrinsics.yaml'
    original=np.array(yaml.safe_load(calibration.read_text())['laser_to_imu']['data']).reshape(4,4)
    original.astype('<f8').tofile(output/'T_imu_lidar.f64')
    result=dict(GT_LOADED=False,first_raw_scan_ns=list(map(str,first_three)),reference_ns=str(START_NS),
        cutoff_ns=str(CUTOFF_NS),scans=len(frames),imu=len(imus),scan_rows=frames,
        first_imu_ns=str(imus[0][0]),last_imu_ns=str(imus[-1][0]),last_scan_end_ns=str(frames[-1][1]),
        raw_calibration=original.tolist(),calibration_sha=sha(calibration),
        time_interpretation='Header scan start plus float seconds, matches R2 observed cadence; not driver sync certification',
        post_cutoff_sensor_measurement_used=False,
        files_sha256={str(p.resolve()):sha(p) for p in output.iterdir() if p.is_file()})
    (output/'extraction.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('reference_ns','cutoff_ns','scans','imu','last_scan_end_ns')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--asset',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);main(**vars(p.parse_args()))


--- ARTIFACT FILE: gyro_input.hpp
#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <vector>
#include <stdexcept>
namespace p32 {
struct Imu {
  uint64_t ns=0;
  Eigen::Vector3d gyro=Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_delta=Eigen::Quaterniond::Identity();
};
// Same trapezoidal SO(3) accumulation and slerp as the existing sensor adapter.
inline void integrate(std::vector<Imu>& samples) {
  for (size_t i=0;i<samples.size();++i) {
    if (!samples[i].gyro.allFinite()) throw std::runtime_error("NONFINITE_IMU");
    if (!i) continue;
    if (samples[i].ns<=samples[i-1].ns || samples[i].ns-samples[i-1].ns>20000000)
      throw std::runtime_error("IMU_GAP_OR_ORDER");
    const double dt=double(samples[i].ns-samples[i-1].ns)*1e-9;
    const Eigen::Vector3d avg=.5*(samples[i].gyro+samples[i-1].gyro);
    const double angle=avg.norm()*dt;
    const Eigen::Quaterniond dq=angle>1e-12
      ? Eigen::Quaterniond(Eigen::AngleAxisd(angle,avg.normalized())) : Eigen::Quaterniond::Identity();
    samples[i].q_delta=(samples[i-1].q_delta*dq).normalized();
  }
}
inline Eigen::Quaterniond orientation(const std::vector<Imu>& samples,uint64_t ns) {
  auto hi=std::lower_bound(samples.begin(),samples.end(),ns,[](const Imu& s,uint64_t n){return s.ns<n;});
  if (hi!=samples.end()&&hi->ns==ns) return hi->q_delta;
  if (hi==samples.begin()||hi==samples.end()) throw std::runtime_error("NO_IMU_EXTRAPOLATION");
  const auto& a=*(hi-1);const auto& b=*hi;
  const double u=double(ns-a.ns)/double(b.ns-a.ns);
  return a.q_delta.slerp(u,b.q_delta).normalized();
}
inline Eigen::Vector3d rotatePoint(const Eigen::Matrix3d& R_i_l,const Eigen::Vector3d& t_i_l,
    const Eigen::Matrix3d& R_i0_ii,const Eigen::Vector3d& p) {
  const Eigen::Matrix3d R_l0_li_rot=R_i_l.transpose()*R_i0_ii*R_i_l;
  const Eigen::Vector3d t_l0_li_rot=R_i_l.transpose()*(R_i0_ii*t_i_l-t_i_l);
  return R_l0_li_rot*p+t_l0_li_rot;
}
}


--- ARTIFACT FILE: input_bridge.cpp
// Offline binding of raw RC1 sensors to the P2B rotation-only bootstrap input.
#include "gyro_input.hpp"
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/SVD>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
int main(int argc,char** argv) {
  if (argc!=2) return 2;
  try {
    const std::string root=argv[1];
    int fd=::open((root+"/bridge.started").c_str(),O_CREAT|O_EXCL|O_WRONLY,0600);
    if (fd<0) throw std::runtime_error("BRIDGE_ALREADY_ATTEMPTED");::close(fd);
    const uint64_t reference=1645999726479818106ULL,cutoff=reference+5000000000ULL;
    std::ifstream input_imu(root+"/imu.txt");std::vector<p32::Imu> imu;
    p32::Imu sample;double ax,ay,az;
    while(input_imu>>sample.ns>>sample.gyro.x()>>sample.gyro.y()>>sample.gyro.z()>>ax>>ay>>az) {
      if(sample.ns>cutoff) throw std::runtime_error("POST_INIT_CUTOFF_IMU");
      imu.push_back(sample);
    }
    if(imu.size()<2) throw std::runtime_error("IMU_MISSING");
    p32::integrate(imu);
    Eigen::Matrix4d T;std::ifstream ext(root+"/T_imu_lidar.f64",std::ios::binary);
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)
      if(!ext.read(reinterpret_cast<char*>(&T(i,j)),8)) throw std::runtime_error("CALIBRATION_READ");
    if(!T.allFinite()||(T.block<3,3>(0,0).transpose()*T.block<3,3>(0,0)-Eigen::Matrix3d::Identity()).norm()>1e-5)
      throw std::runtime_error("INVALID_CALIBRATION");
    // Existing adapter's decimal-rounding handling, never alters the raw file.
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(T.block<3,3>(0,0),Eigen::ComputeFullU|Eigen::ComputeFullV);
    const Eigen::Matrix3d R=svd.matrixU()*svd.matrixV().transpose();
    const Eigen::Vector3d t=T.block<3,1>(0,3);
    rosbag::Bag bag;bag.open(root+"/init_rotation_input.bag",rosbag::bagmode::Write);
    std::ifstream scans(root+"/scans.txt");uint64_t start,end;size_t count;std::string name;int index=0;
    std::ofstream ledger(root+"/bridge.csv");
    ledger<<"scan_ns,end_ns,points,max_point_change_m,translation_model,max_imu_available_ns\n"<<std::setprecision(17);
    while(scans>>start>>end>>count>>name) {
      if(start<reference||end>cutoff||(!index&&start!=reference)) throw std::runtime_error("SCAN_WINDOW");
      std::ifstream points(root+"/"+name,std::ios::binary|std::ios::ate);
      if(!points||points.tellg()!=static_cast<long long>(count*16)) throw std::runtime_error("POINT_BYTE_COUNT");
      points.seekg(0);
      const auto q0=p32::orientation(imu,start);
      sensor_msgs::PointCloud2 cloud;cloud.header.seq=++index;
      cloud.header.stamp=ros::Time(start/1000000000ULL,start%1000000000ULL);
      cloud.header.frame_id="cmu_rc1_velodyne";cloud.height=1;cloud.width=count;cloud.is_dense=true;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(3,"x",1,sensor_msgs::PointField::FLOAT32,"y",1,sensor_msgs::PointField::FLOAT32,
        "z",1,sensor_msgs::PointField::FLOAT32);modifier.resize(count);
      float xyzt[4];double change=0;
      for(size_t k=0;k<count;++k) {
        if(!points.read(reinterpret_cast<char*>(xyzt),16)) throw std::runtime_error("POINT_TRUNCATED");
        const Eigen::Vector3d p(xyzt[0],xyzt[1],xyzt[2]);
        if(!p.allFinite()||!std::isfinite(xyzt[3])||xyzt[3]<0||xyzt[3]>.15) throw std::runtime_error("INVALID_POINT");
        const uint64_t ns=start+static_cast<uint64_t>(std::llround(double(xyzt[3])*1e9));
        if(ns>end||ns>cutoff) throw std::runtime_error("POINT_AFTER_SCAN_OR_CUTOFF");
        const auto qi=p32::orientation(imu,ns);
        const Eigen::Matrix3d relative=q0.toRotationMatrix().transpose()*qi.toRotationMatrix();
        const Eigen::Vector3d corrected=p32::rotatePoint(R,t,relative,p);
        if(!corrected.allFinite()||corrected.norm()>1000) throw std::runtime_error("NONFINITE_OR_EXPLODED_POINT");
        const float xyz[3]={float(corrected.x()),float(corrected.y()),float(corrected.z())};
        std::memcpy(cloud.data.data()+k*12,xyz,12);change=std::max(change,(corrected-p).norm());
      }
      bag.write("/superloc_adapter/points_deskewed",cloud.header.stamp,cloud);
      ledger<<start<<','<<end<<','<<count<<','<<change<<",P2B_ROTATION_ONLY_NO_TRANSLATION_CV,"<<imu.back().ns<<'\n';
    }
    bag.close();std::cout<<"BOOTSTRAP_BRIDGE_PASS scans="<<index<<" reference_ns="<<reference<<'\n';
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}


--- ARTIFACT FILE: initializer.cpp
// The complete historical main executes unchanged; subclass only observes align.
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <iostream>
namespace p32 {
std::ofstream ledger;
int map_calls=0,local_calls=0;
template<typename M> void matrix(std::ostream& out,const M& T) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<T(i,j);
}
}
namespace pcl {
template<typename Source,typename Target>
class P32LoggedICP : public IterativeClosestPoint<Source,Target> {
  using Base=IterativeClosestPoint<Source,Target>;
public:
  void align(pcl::PointCloud<Source>& output,const Eigen::Matrix4f& guess) {
    const bool map=this->getMaxCorrespondenceDistance()==8.;
    if(map&&++p32::map_calls>24)throw std::runtime_error("CANDIDATE_BUDGET");
    if(!map)++p32::local_calls;
    const auto tick=std::chrono::steady_clock::now();
    Base::align(output,guess); // Same solver, same source/target and seed.
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
    if(!map)return;
    const Eigen::Matrix4f final=this->getFinalTransformation();
    pcl::KdTreeFLANN<Target> tree;tree.setInputCloud(this->getInputTarget());
    std::vector<double> distances;std::vector<int> ids(1);std::vector<float> squared(1);size_t inside=0;
    for(const auto& p:output) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      if(tree.nearestKSearch(p,1,ids,squared)==1) {
        const double d=std::sqrt(squared[0]);distances.push_back(d);if(d<=.5)++inside;
      }
    }
    std::sort(distances.begin(),distances.end());
    const double median=distances.empty()?std::numeric_limits<double>::infinity():distances[distances.size()/2];
    const double p90=distances.empty()?std::numeric_limits<double>::infinity():distances[size_t(.9*(distances.size()-1))];
    const double fraction=distances.empty()?0.:double(inside)/distances.size();
    const double error=(guess.block<3,3>(0,0).transpose()*guess.block<3,3>(0,0)-Eigen::Matrix3f::Identity()).norm();
    const double det=guess.block<3,3>(0,0).determinant();
    const double translation=(final.block<3,1>(0,3)-guess.block<3,1>(0,3)).norm();
    const double angle=Eigen::AngleAxisf(guess.block<3,3>(0,0).transpose()*final.block<3,3>(0,0)).angle()*180/M_PI;
    const double diag=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count()-ms;
    p32::ledger<<p32::map_calls-1<<','<<this->getInputSource()->size()<<','<<this->hasConverged()<<','
      <<this->getFitnessScore()<<','<<median<<','<<p90<<','<<fraction<<','<<translation<<','<<angle<<','
      <<det<<','<<error<<','<<(guess.allFinite()&&det>0&&error<=1e-5)<<','<<ms<<','<<diag;
    p32::matrix(p32::ledger,guess);p32::matrix(p32::ledger,final);p32::ledger<<'\n';p32::ledger.flush();
  }
};
}
#define IterativeClosestPoint P32LoggedICP
#define main historicalInitializerMain
#include "legacy/superloc_first_segment_initializer.cpp"
#undef main
#undef IterativeClosestPoint
int main(int argc,char** argv) {
  if(argc<5)return 2;
  p32::ledger.open(std::string(argv[4])+"/candidate_diagnostics.csv");
  p32::ledger<<std::setprecision(17)<<"candidate_id,source_points,converged,fitness,nn_median_m,nn_p90_m,inlier_0_5m,correction_m,correction_deg,seed_determinant,seed_orthogonality_error,seed_SE3_float_tolerance,icp_ms,diagnostics_ms";
  for(const auto prefix:{"seed","result"})for(int i=0;i<4;++i)for(int j=0;j<4;++j)p32::ledger<<','<<prefix<<i<<j;
  p32::ledger<<'\n';
  const int code=historicalInitializerMain(argc,argv);
  std::cout<<"P32_CALL_LEDGER local_icp="<<p32::local_calls<<" map_icp="<<p32::map_calls<<'\n';
  return code;
}


--- ARTIFACT FILE: math_test.cpp
#include "gyro_input.hpp"
#define main historicalTestUnusedMain
#include "legacy/superloc_first_segment_initializer.cpp"
#undef main
int main(int argc,char** argv) {
  if(argc!=2)return 8;
  Eigen::Matrix3f R;Eigen::Vector3f parsed_t;
  if(!loadOfficialPrior(argv[1],R,parsed_t))return 9;
  Eigen::Matrix3f expected;
  expected<<.823800f,-.566853f,-.005503f,.565879f,.822884f,-.051418f,.033675f,.039243f,.998662f;
  const Eigen::Vector3f t(39.929523f,-176.217148f,-5.225427f);
  if((R-expected).norm()!=0||(parsed_t-t).norm()!=0)return 10;
  if(!R.allFinite()||R.determinant()<=0||(R.transpose()*R-Eigen::Matrix3f::Identity()).norm()>1e-5)return 11;
  const auto axes=properAxes();if(axes.size()!=24)return 1;
  for(size_t i=0;i<axes.size();++i) {
    if((axes[i].transpose()*axes[i]-Eigen::Matrix3f::Identity()).norm()!=0||axes[i].determinant()!=1)return 2;
    for(size_t j=0;j<i;++j)if((axes[i]-axes[j]).norm()==0)return 3;
  }
  for(const auto& A:axes){Transform seed=Transform::Identity();seed.linear()=R*A;seed.translation()=t;if(seed.translation()!=t)return 4;}
  std::vector<p32::Imu> imu(11);for(size_t i=0;i<imu.size();++i){imu[i].ns=1000000000+i*10000000;imu[i].gyro=Eigen::Vector3d(0,0,1);}
  p32::integrate(imu);
  const auto q=p32::orientation(imu,1055000000);
  if(std::abs(Eigen::AngleAxisd(q).angle()-.055)>1e-12)return 5;
  const auto zero=p32::rotatePoint(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),Eigen::Vector3d(1,2,3));
  if((zero-Eigen::Vector3d(1,2,3)).norm()>1e-12)return 6;
  bool refused=false;try{p32::orientation(imu,999999999);}catch(...){refused=true;}if(!refused)return 7;
  std::cout<<"P32_TRANSFER_MATH_PASS axes=24 unique_proper fixed_translation gyro_and_point=PASS no_extrapolation=PASS active_yaml=PASS det="<<R.determinant()<<" orthogonality="<<(R.transpose()*R-Eigen::Matrix3f::Identity()).norm()<<'\n';return 0;
}


--- ARTIFACT FILE: run_once.py
#!/usr/bin/env python3
"""Run the unchanged legacy bootstrap once; its unconditional PASS is not a gate."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

MAP_SHA = '6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228'
PRIOR_SHA = 'fc319ad99737a717456e11eb70383d4848eadf7e46587fee0643d6732cf1c3a1'
LEGACY_SHA = 'deb58aeb3bd76f802568469f50d33f91b8026924b3fe299fa93b8fa8f9dc266d'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def run(command, name, output, env):
    resource = output / (name + '_resource.txt')
    timed = ['/usr/bin/time', '-f',
             'user_cpu_sec=%U\nsystem_cpu_sec=%S\npeak_rss_kib=%M\nwall_sec=%e',
             '-o', str(resource)] + list(map(str, command))
    with (output / (name + '.log')).open('x') as log:
        code = subprocess.call(timed, stdout=log, stderr=subprocess.STDOUT, env=env)
    receipt = dict(command=list(map(str, command)), exit_code=code,
                   resource_text=resource.read_text())
    (output / (name + '_execution.json')).write_text(json.dumps(receipt, indent=2) + '\n')
    return code


def main(asset, inputs, build, output, protocol):
    asset, inputs, build, output = (p.resolve() for p in (asset, inputs, build, output))
    # This directory is the one-shot guard. Never replace or reuse it.
    output.mkdir(parents=True, exist_ok=False)
    extraction = json.loads((inputs / 'extraction.json').read_text())
    for filename, expected in extraction['files_sha256'].items():
        if sha(Path(filename)) != expected:
            raise ValueError('EXTRACTED_INPUT_HASH_CHANGED: ' + filename)
    if extraction['GT_LOADED'] or extraction['post_cutoff_sensor_measurement_used']:
        raise ValueError('INPUT_CAUSAL_CONTRACT')
    sources = Path(__file__).resolve().parent
    map_path = asset / 'map/corridor02.pcd'
    prior = asset / 'initial_pose/corridor02.yaml'
    hashes = {str(path): sha(path) for path in
              (map_path, prior, sources / 'legacy/superloc_first_segment_initializer.cpp')}
    if list(hashes.values()) != [MAP_SHA, PRIOR_SHA, LEGACY_SHA]:
        raise ValueError('FROZEN_INPUT_OR_LEGACY_MISMATCH')
    for binary in ('math_test', 'input_bridge', 'initializer'):
        hashes[str(build / binary)] = sha(build / binary)
    hashes[str(protocol.resolve())] = sha(protocol)
    for path in sorted(sources.rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            hashes[str(path)] = sha(path)
    freeze = dict(created_unix_ns=str(time.time_ns()), map_sha256=MAP_SHA,
                  prior_sha256=PRIOR_SHA, hashes=hashes, extraction=extraction,
                  raw_bag_path=str(asset / 'raw/corridor02.bag'),
                  raw_bag_size_bytes=(asset / 'raw/corridor02.bag').stat().st_size,
                  raw_bag_full_sha256_recomputed=False, GT_loaded=False)
    (output / 'input_freeze.json').write_text(json.dumps(freeze, indent=2) + '\n')
    env = os.environ.copy()
    env['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    env['ROS_LOG_DIR'] = str(output / 'ros_logs')
    if run([build / 'math_test', prior], 'math_test', output, env):
        raise ValueError('MATH_OR_ACTIVE_PRIOR_TEST_FAILED')
    if run([build / 'input_bridge', inputs], 'input_bridge', output, env):
        raise ValueError('BOOTSTRAP_INPUT_BRIDGE_FAILED')
    freeze['hashes'][str(inputs / 'init_rotation_input.bag')] = sha(inputs / 'init_rotation_input.bag')
    freeze['hashes'][str(inputs / 'bridge.csv')] = sha(inputs / 'bridge.csv')
    (output / 'input_freeze.json').write_text(json.dumps(freeze, indent=2) + '\n')
    command = [build / 'initializer', inputs / 'init_rotation_input.bag', map_path, prior, output, '5']
    code = run(command, 'initializer', output, env)
    with (output / 'initializer_candidates.csv').open() as stream:
        ranked = list(csv.DictReader(stream))
    with (output / 'candidate_diagnostics.csv').open() as stream:
        diagnostics = {int(r['candidate_id']): r for r in csv.DictReader(stream)}
    if len(ranked) != 24 or len(diagnostics) != 24:
        raise ValueError('ORIGINAL_CANDIDATE_BUDGET_NOT_COMPLETED')
    best = diagnostics[int(ranked[0]['candidate_id'])]
    checks = dict(program_exit_zero=code == 0, converged=best['converged'] == '1',
                  legal_seed=best['seed_SE3_float_tolerance'] == '1',
                  fitness=float(best['fitness']) <= 10,
                  nn_median=float(best['nn_median_m']) <= 1,
                  nn_p90=float(best['nn_p90_m']) <= 3,
                  inlier_0_5m=float(best['inlier_0_5m']) >= .5)
    with (output / 'initializer_registration.csv').open() as stream:
        local = list(csv.DictReader(stream))
    result = dict(program_exit_code=code, legacy_ranked_best=best, checks=checks,
                  initialization_admitted=all(checks.values()),
                  map_icp_calls=len(diagnostics), local_icp_calls=len(local),
                  local_scans_used=1 + sum(int(row['used']) for row in local),
                  method='UNCHANGED_LEGACY_FIRST_SEGMENT_24_PROPER_AXES',
                  candidate_rule='R_seed=R_yaml*A_i; t_seed=t_yaml',
                  legacy_yaml_status='NOT_A_RUNTIME_INITIALIZATION_UNLESS_ADMITTED',
                  GT_loaded=False, independent_validation_executed=False,
                  normalized_map_executed=False, F_executed=False, G_executed=False)
    (output / 'admission.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for key in ('asset', 'inputs', 'build', 'output', 'protocol'):
        parser.add_argument('--' + key, type=Path, required=True)
    main(**vars(parser.parse_args()))


--- ARTIFACT FILE: legacy/superloc_first_segment_initializer.cpp
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Transform = Eigen::Affine3f;

namespace {

Cloud::Ptr voxel(const Cloud::ConstPtr &in, float leaf) {
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(in); vg.setLeafSize(leaf, leaf, leaf); vg.filter(*out);
  return out;
}

double rotationAngle(const Eigen::Matrix3f &R) {
  const double c = std::max(-1.0, std::min(1.0, (static_cast<double>(R.trace()) - 1.0) * 0.5));
  return std::acos(c);
}

std::vector<Eigen::Matrix3f> properAxes() {
  std::vector<Eigen::Matrix3f> out;
  const std::array<int,3> p{0,1,2};
  std::array<int,3> a = p;
  do {
    int inv = 0; for (int i=0;i<3;++i) for (int j=i+1;j<3;++j) if (a[i]>a[j]) ++inv;
    for (int sx : {-1,1}) for (int sy : {-1,1}) for (int sz : {-1,1}) {
      Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
      M(0,a[0])=static_cast<float>(sx); M(1,a[1])=static_cast<float>(sy); M(2,a[2])=static_cast<float>(sz);
      if (M.determinant() > 0.5f) out.push_back(M);
    }
  } while (std::next_permutation(a.begin(), a.end()));
  return out;
}

void writeMatrix(std::ostream &os, const Eigen::Matrix3f &R) {
  os << "[" << std::setprecision(10)
     << R(0,0) << ", " << R(0,1) << ", " << R(0,2) << "; "
     << R(1,0) << ", " << R(1,1) << ", " << R(1,2) << "; "
     << R(2,0) << ", " << R(2,1) << ", " << R(2,2) << "]";
}

bool loadOfficialPrior(const std::string &path, Eigen::Matrix3f &R, Eigen::Vector3f &t) {
  std::ifstream f(path); if (!f) return false;
  std::string line, all; while (std::getline(f,line)) all += line + "\n";
  auto data = [&](const std::string &key, int n) {
    std::vector<float> v; const auto pos = all.find(key); if (pos == std::string::npos) return v;
    const auto b = all.find("data:", pos); if (b == std::string::npos) return v;
    const auto e = all.find(']', b); if (e == std::string::npos) return v;
    std::stringstream ss(all.substr(b + 5, e - b - 5)); char c;
    while (ss && static_cast<int>(v.size()) < n) { float x; if (ss >> x) v.push_back(x); else ss.clear(); ss >> c; }
    return v;
  };
  const auto rv = data("extrinsicRotation_world_darpa",9), tv = data("extrinsicTranslation_world_darpa",3);
  if (rv.size()!=9 || tv.size()!=3) return false;
  R << rv[0],rv[1],rv[2],rv[3],rv[4],rv[5],rv[6],rv[7],rv[8];
  t << tv[0],tv[1],tv[2];
  return true;
}

struct Candidate { int id; Transform T; bool converged; double fitness, nn_median, nn_p90, inlier; };

Candidate registerCandidate(int id, const Transform &seed, const Cloud::ConstPtr &source, const Cloud::ConstPtr &target) {
  pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
  icp.setInputSource(source); icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(8.0); icp.setMaximumIterations(60); icp.setTransformationEpsilon(1e-7); icp.setEuclideanFitnessEpsilon(1e-6);
  Cloud aligned; icp.align(aligned, seed.matrix());
  Candidate c{id, Transform(icp.getFinalTransformation()), icp.hasConverged(), icp.getFitnessScore(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0.0};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree; tree.setInputCloud(target);
  std::vector<float> d; d.reserve(aligned.size()); int in=0; std::vector<int> idx(1); std::vector<float> dist(1);
  for (const auto &p : aligned.points) if (pcl::isFinite(p) && tree.nearestKSearch(p,1,idx,dist)>0) { const double x=std::sqrt(dist[0]); d.push_back(x); if (x<1.0) ++in; }
  if (!d.empty()) { std::sort(d.begin(),d.end()); c.nn_median=d[d.size()/2]; c.nn_p90=d[std::min(d.size()-1,static_cast<std::size_t>(0.90*(d.size()-1)))]; c.inlier=static_cast<double>(in)/d.size(); }
  return c;
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "superloc_first_segment_initializer", ros::init_options::AnonymousName);
  if (argc < 5) {
    std::cerr << "usage: superloc_first_segment_initializer ADAPTER_BAG MAP_PCD OFFICIAL_INIT OUT_DIR [duration_sec]\n";
    return 2;
  }
  const std::string bag_path=argv[1], map_path=argv[2], prior_path=argv[3], out_dir=argv[4];
  const double duration=argc>5 ? std::stod(argv[5]) : 5.0;
  Cloud::Ptr map(new Cloud); if (pcl::io::loadPCDFile(map_path,*map)<0) { std::cerr<<"map read failed\n"; return 3; }
  Cloud::Ptr map_ds=voxel(map,0.20f);
  rosbag::Bag bag; try { bag.open(bag_path,rosbag::bagmode::Read); } catch (...) { std::cerr<<"bag open failed\n"; return 4; }
  const std::vector<std::string> topics{"/superloc_adapter/points_deskewed"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<Cloud::Ptr> scans; std::vector<double> stamps; double first=-1;
  for (const auto &m : view) {
    auto msg=m.instantiate<sensor_msgs::PointCloud2>(); if (!msg) continue;
    const double s=msg->header.stamp.toSec(); if (first<0) first=s; if (s>first+duration) break;
    Cloud::Ptr c(new Cloud); pcl::fromROSMsg(*msg,*c); if (c->empty()) continue;
    scans.push_back(voxel(c,0.20f)); stamps.push_back(s);
  }
  bag.close();
  std::ofstream reg(out_dir+"/initializer_registration.csv");
  reg << "index,stamp,converged,fitness,translation_m,rotation_deg,used\n";
  if (scans.size()<2) { std::cerr<<"not enough scans\n"; return 5; }
  Cloud::Ptr local(new Cloud); *local += *scans.front(); Transform T_first=Transform::Identity(); local->clear(); *local += *scans.front();
  Cloud::Ptr previous=scans.front(); int used=1;
  for (std::size_t i=1;i<scans.size();++i) {
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp; icp.setInputSource(scans[i]); icp.setInputTarget(previous); icp.setMaxCorrespondenceDistance(2.5); icp.setMaximumIterations(40); icp.setTransformationEpsilon(1e-6);
    Cloud aligned; icp.align(aligned,Eigen::Matrix4f::Identity()); const Transform d(icp.getFinalTransformation());
    const double tr=d.translation().norm(), rd=rotationAngle(d.rotation())*180.0/M_PI; const bool ok=icp.hasConverged() && tr<3.0 && rd<45.0 && std::isfinite(icp.getFitnessScore());
    reg << i << ',' << stamps[i] << ',' << (icp.hasConverged()?1:0) << ',' << icp.getFitnessScore() << ',' << tr << ',' << rd << ',' << (ok?1:0) << '\n';
    if (ok) { T_first = T_first * d; Cloud::Ptr transformed(new Cloud); pcl::transformPointCloud(*scans[i],*transformed,T_first); *local += *transformed; previous=scans[i]; ++used; }
  }
  local=voxel(local,0.25f); pcl::io::savePCDFileBinary(out_dir+"/first_segment_local_submap.pcd",*local);
  Eigen::Matrix3f R; Eigen::Vector3f t; if (!loadOfficialPrior(prior_path,R,t)) { std::cerr<<"official prior parse failed\n"; return 6; }
  const auto axes=properAxes(); std::vector<Candidate> cands; cands.reserve(axes.size());
  for (std::size_t i=0;i<axes.size();++i) { Transform seed=Transform::Identity(); seed.linear()=R*axes[i]; seed.translation()=t; cands.push_back(registerCandidate(static_cast<int>(i),seed,local,map_ds)); }
  std::sort(cands.begin(),cands.end(),[](const Candidate&a,const Candidate&b){ if(a.converged!=b.converged)return a.converged>b.converged; return a.fitness<b.fitness; });
  std::ofstream summary(out_dir+"/initializer_candidates.csv"); summary<<"rank,candidate_id,converged,fitness,nn_median,nn_p90,inlier,translation_x,translation_y,translation_z,rotation_angle_deg\n";
  for (std::size_t r=0;r<cands.size();++r) summary<<r+1<<','<<cands[r].id<<','<<(cands[r].converged?1:0)<<','<<cands[r].fitness<<','<<cands[r].nn_median<<','<<cands[r].nn_p90<<','<<cands[r].inlier<<','<<cands[r].T.translation().x()<<','<<cands[r].T.translation().y()<<','<<cands[r].T.translation().z()<<','<<rotationAngle(cands[r].T.rotation())*180.0/M_PI<<'\n';
  const Candidate &best=cands.front(); std::ofstream init(out_dir+"/corridor01_init.yaml");
  init<<"source: FIRST_SEGMENT_WITH_OFFICIAL_PRIOR\n"<<"t_init_start: "<<std::setprecision(15)<<first<<"\n"<<"initialization_interval_sec: "<<duration<<"\n"<<"scans_available: "<<scans.size()<<"\n"<<"scans_used: "<<used<<"\n"<<"GT_used: false\n"<<"future_after_5s_used: false\n"<<"official_prior_semantics: OTHER_OFFICIAL_FRAME_TRANSFORM_UNRESOLVED\n"<<"selected_candidate_id: "<<best.id<<"\n"<<"T_map_lidar:\n  translation: ["<<best.T.translation().x()<<", "<<best.T.translation().y()<<", "<<best.T.translation().z()<<"]\n  rotation_matrix: "; writeMatrix(init,best.T.rotation()); init<<"\n  quaternion_xyzw: ["<<Eigen::Quaternionf(best.T.rotation()).x()<<", "<<Eigen::Quaternionf(best.T.rotation()).y()<<", "<<Eigen::Quaternionf(best.T.rotation()).z()<<", "<<Eigen::Quaternionf(best.T.rotation()).w()<<"]\n";
  std::cout<<"FIRST_SEGMENT_INITIALIZER_PASS scans="<<scans.size()<<" used="<<used<<" candidates="<<cands.size()<<" best_converged="<<best.converged<<" best_fitness="<<best.fitness<<"\n";
  return best.converged ? 0 : 7;
}


--- FROZEN PROTOCOL
# P32-R3冻结预算（执行/GT之前）

START_SHA：b5814cff0fcdab6f3d210915ecaf4452f46aa26d。
沿用研究分支和P10工作区，受保护工作区/历史缓存/报告不改，不自动push。
不读取GT，不运行893秒，不调整P31算法参数或噪声。

## 实际旧方法迁移

读取P2B_ADAPTER_REPORT及现有initializer/normalizer完整源码。
将原initializer、normalizer、holdout和adapter数学来源保留为本轮副本。
初始化器只增加诊断：seed/final矩阵、SE(3)数值残差、0.5m覆盖率、
平移/旋转修正与每候选耗时；原局部子图、ICP参数、候选及排序不改。
一次首5秒scan-to-scan ICP+24次原proper-axis scan-to-map ICP，非单种子NDT。

原真实规则为R_seed_i=R_yaml*A_i，t_seed_i=t_yaml：右乘旋转且平移固定。
不包含整体世界左乘Rz90°同时旋转t的假设，不擅自加入该假设。
只解析生效矩阵，不试逆、注释矩阵或平移网格。

## 输入与因果边界

L0特指原第3扫描，header1645999726479818106ns；NDT/ICP首扫描绑定该ns。
init cutoff=1645999731479818106ns，即L0+5秒。
只接收start>=L0且完整scan end<=cutoff的扫描，保留此前真实IMU边界
样本与截止之前的真实IMU；不把跨cutoff扫描截断或读取截止后IMU积分。
R2已抽查XYZ/time为float32、time约0..0.100839s；保持header/seconds解释。

复用P2B初始化输入的gyro-trapezoidal+SO(3)插值+旋转去畸变数学，
使用RC1原标定数值和既有proper-rotation舍入处理。scan-to-scan ICP
负责首段各扫描的完整相对运动；不把rotation-only称为P31完整Deskew。
当前旧adapter的full-SE3反馈模式会等待此前NDT，不在bootstrap中使用。
不使用EKF公开轨迹重建，不生成虚假odometry，不伪造缺失IMU。
旧initializer收到的bag topic仍为/superloc_adapter/points_deskewed。

## 准入与后续预算

保留旧排序：先converged，再fitness小。旧日志/YAML即使写PASS也不是
本轮验收：最佳候选必须finite、proper到1e-5容差、converged、fitness<=10、
NN median<=1m、P90<=3m且0.5m覆盖>=50%。这是预设非GT工程screen，
不是搜索调参；失败不重跑、不改阈值、不换候选、不归一化地图。

通过才一次5–10秒holdout，原ICP传播/几何检查不改；保留所有样本。
窗口匹配finite、无失败、无严重跳变，aggregate NN median<=1m、
P90<=3m作为P2B历史结果量级参考；不根据holdout重新选候选。
未通过则停止，未执行项NOT_RUN，不伪造结果。

通过后才施加T_N_M=inverse(T_M_L0)，保存点数/bbox/SHA与往返误差。
初始化首5秒为离线预热，结果最早cutoff后可用，不回写为L0时刻在线状态。
holdout决定正式准入最早在L0+10秒；正式F/G从合法启动时刻开始，使用
此前sensor-only运动传播初值，不使用未来验证数据或GT作滤波初值。
仅F30s/G30s各一次，算法、噪声、固定lag、step limit、velocity_blend等不改。
G用自身真实Camera特征；旧PnP路径关闭。完整893s和GT精度评价均禁止。

结果区分：方法迁移/初始化几何、地图归一化、F/G工程运行、GT参考资格。
缺少GT参考点不单独否定合法非GT工程运行；也不宣布认证精度或泛化。


--- ACTUAL OFFICIAL PRIOR (not a candidate list)
%YAML:1.0
# -0.577760 -0.014115 0.816085 176.987457
# 0.816198 -0.005282 0.577749 198.518204
# -0.003845 0.999887 0.014572 -4.614883
# 0.000000 0.000000 0.000000 1.000000

# 0.822928 -0.568138 0.002976 34.413502
# 0.566308 0.819835 -0.084654 -179.706741
# 0.045656 0.071350 0.996406 -4.833658
# 0.000000 0.000000 0.000000 1.000000

# 0.825820 -0.563919 0.004027 39.542725
# 0.562948 0.823934 -0.064987 -175.981827
# 0.033330 0.055934 0.997878 -4.762141
# 0.000000 0.000000 0.000000 1.000000

# 0.823800 -0.566853 -0.005503 39.929523
# 0.565879 0.822884 -0.051418 -176.217148
# 0.033675 0.039243 0.998662 -5.225427
# 0.000000 0.000000 0.000000 1.000000

extrinsicRotation_world_darpa: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [0.823800, -0.566853, -0.005503,
          0.565879, 0.822884, -0.051418,
          0.033675, 0.039243, 0.998662]

extrinsicTranslation_world_darpa: !!opencv-matrix
  rows: 3
  cols: 1
  dt: d
  data: [39.929523, -176.217148, -5.225427]
  #data: [42, -176.217148, -5.225427]


--- INPUT RECEIPT
{
  "GT_LOADED": false,
  "first_raw_scan_ns": [
    "1645999726278098106",
    "1645999726378957987",
    "1645999726479818106"
  ],
  "reference_ns": "1645999726479818106",
  "cutoff_ns": "1645999731479818106",
  "scans": 49,
  "imu": 1001,
  "scan_rows": [
    [
      1645999726479818106,
      1645999726580657319,
      28089,
      "scan_1645999726479818106.xyzt"
    ],
    [
      1645999726580677986,
      1645999726681517437,
      28097,
      "scan_1645999726580677986.xyzt"
    ],
    [
      1645999726681538105,
      1645999726782376364,
      28073,
      "scan_1645999726681538105.xyzt"
    ],
    [
      1645999726782397985,
      1645999726883236482,
      28093,
      "scan_1645999726782397985.xyzt"
    ],
    [
      1645999726883256912,
      1645999726984096363,
      28103,
      "scan_1645999726883256912.xyzt"
    ],
    [
      1645999726984117031,
      1645999727084904268,
      28152,
      "scan_1645999726984117031.xyzt"
    ],
    [
      1645999727084924936,
      1645999727185764387,
      28128,
      "scan_1645999727084924936.xyzt"
    ],
    [
      1645999727185785055,
      1645999727286624268,
      28150,
      "scan_1645999727185785055.xyzt"
    ],
    [
      1645999727286644936,
      1645999727387484387,
      28142,
      "scan_1645999727286644936.xyzt"
    ],
    [
      1645999727387505054,
      1645999727488344267,
      28137,
      "scan_1645999727387505054.xyzt"
    ],
    [
      1645999727488364935,
      1645999727589204386,
      28126,
      "scan_1645999727488364935.xyzt"
    ],
    [
      1645999727589225054,
      1645999727690063313,
      28112,
      "scan_1645999727589225054.xyzt"
    ],
    [
      1645999727690084934,
      1645999727790923431,
      28111,
      "scan_1645999727690084934.xyzt"
    ],
    [
      1645999727790944099,
      1645999727891783312,
      28100,
      "scan_1645999727790944099.xyzt"
    ],
    [
      1645999727891803980,
      1645999727992643431,
      28142,
      "scan_1645999727891803980.xyzt"
    ],
    [
      1645999727992664099,
      1645999728093458489,
      28101,
      "scan_1645999727992664099.xyzt"
    ],
    [
      1645999728093478918,
      1645999728194318369,
      28143,
      "scan_1645999728093478918.xyzt"
    ],
    [
      1645999728194339037,
      1645999728295178488,
      28134,
      "scan_1645999728194339037.xyzt"
    ],
    [
      1645999728295198917,
      1645999728396038368,
      28109,
      "scan_1645999728295198917.xyzt"
    ],
    [
      1645999728396059036,
      1645999728496898487,
      28142,
      "scan_1645999728396059036.xyzt"
    ],
    [
      1645999728496918917,
      1645999728597757414,
      28145,
      "scan_1645999728496918917.xyzt"
    ],
    [
      1645999728597779036,
      1645999728698617295,
      28132,
      "scan_1645999728597779036.xyzt"
    ],
    [
      1645999728698637962,
      1645999728799477413,
      28128,
      "scan_1645999728698637962.xyzt"
    ],
    [
      1645999728799498081,
      1645999728900337294,
      28138,
      "scan_1645999728799498081.xyzt"
    ],
    [
      1645999728900357962,
      1645999729001197413,
      28142,
      "scan_1645999728900357962.xyzt"
    ],
    [
      1645999729001172066,
      1645999729102011279,
      28133,
      "scan_1645999729001172066.xyzt"
    ],
    [
      1645999729102031946,
      1645999729202871397,
      28128,
      "scan_1645999729102031946.xyzt"
    ],
    [
      1645999729202892065,
      1645999729303731278,
      28122,
      "scan_1645999729202892065.xyzt"
    ],
    [
      1645999729303751945,
      1645999729404591396,
      28085,
      "scan_1645999729303751945.xyzt"
    ],
    [
      1645999729404612064,
      1645999729505450323,
      28106,
      "scan_1645999729404612064.xyzt"
    ],
    [
      1645999729505471945,
      1645999729606310442,
      28094,
      "scan_1645999729505471945.xyzt"
    ],
    [
      1645999729606331110,
      1645999729707170323,
      28115,
      "scan_1645999729606331110.xyzt"
    ],
    [
      1645999729707190990,
      1645999729808030441,
      28103,
      "scan_1645999729707190990.xyzt"
    ],
    [
      1645999729808051109,
      1645999729908890322,
      28109,
      "scan_1645999729808051109.xyzt"
    ],
    [
      1645999729908910990,
      1645999730009705380,
      28124,
      "scan_1645999729908910990.xyzt"
    ],
    [
      1645999730009726048,
      1645999730110565261,
      28149,
      "scan_1645999730009726048.xyzt"
    ],
    [
      1645999730110585928,
      1645999730211425379,
      28134,
      "scan_1645999730110585928.xyzt"
    ],
    [
      1645999730211446047,
      1645999730312285260,
      28125,
      "scan_1645999730211446047.xyzt"
    ],
    [
      1645999730312305927,
      1645999730413144424,
      28151,
      "scan_1645999730312305927.xyzt"
    ],
    [
      1645999730413165092,
      1645999730514004305,
      28138,
      "scan_1645999730413165092.xyzt"
    ],
    [
      1645999730514024973,
      1645999730614864424,
      28135,
      "scan_1645999730514024973.xyzt"
    ],
    [
      1645999730614885092,
      1645999730715724305,
      28126,
      "scan_1645999730614885092.xyzt"
    ],
    [
      1645999730715744972,
      1645999730816584423,
      28126,
      "scan_1645999730715744972.xyzt"
    ],
    [
      1645999730816605091,
      1645999730917444304,
      28141,
      "scan_1645999730816605091.xyzt"
    ],
    [
      1645999730917464972,
      1645999731018258408,
      28109,
      "scan_1645999730917464972.xyzt"
    ],
    [
      1645999731018279076,
      1645999731119118289,
      28145,
      "scan_1645999731018279076.xyzt"
    ],
    [
      1645999731119138956,
      1645999731219977453,
      28153,
      "scan_1645999731119138956.xyzt"
    ],
    [
      1645999731219999075,
      1645999731320837334,
      28145,
      "scan_1645999731219999075.xyzt"
    ],
    [
      1645999731320858002,
      1645999731421697453,
      28146,
      "scan_1645999731320858002.xyzt"
    ]
  ],
  "first_imu_ns": "1645999726477872000",
  "last_imu_ns": "1645999731477632000",
  "last_scan_end_ns": "1645999731421697453",
  "raw_calibration": [
    [
      0.999954875339377,
      0.0,
      0.009499857104811,
      0.0
    ],
    [
      0.0,
      1.0,
      0.0,
      0.0
    ],
    [
      -0.009499857104811,
      0.0,
      0.999954875339377,
      0.0
    ],
    [
      0.0,
      0.0,
      0.0,
      1.0
    ]
  ],
  "calibration_sha": "892bf743dc0f51eabfe9d8ae71dc98663753f174910ae9cc40ee244c77a9e217",
  "time_interpretation": "Header scan start plus float seconds, matches R2 observed cadence; not driver sync certification",
  "post_cutoff_sensor_measurement_used": false
}

--- ACTUAL OUTPUT: admission.json
{
  "program_exit_code": 7,
  "legacy_ranked_best": {
    "candidate_id": "2",
    "source_points": "2569",
    "converged": "0",
    "fitness": "85678.30752329457",
    "nn_median_m": "292.34402465820312",
    "nn_p90_m": "306.59817504882812",
    "inlier_0_5m": "0",
    "correction_m": "0",
    "correction_deg": "0",
    "seed_determinant": "0.99999988079071045",
    "seed_orthogonality_error": "2.0917686924804002e-06",
    "seed_SE3_float_tolerance": "1",
    "icp_ms": "801.82723199999998",
    "diagnostics_ms": "403.36761200000012",
    "seed00": "0.82380002737045288",
    "seed01": "0.56685298681259155",
    "seed02": "0.0055030002258718014",
    "seed03": "39.929523468017578",
    "seed10": "0.56587898731231689",
    "seed11": "-0.82288402318954468",
    "seed12": "0.051417998969554901",
    "seed13": "-176.21714782714844",
    "seed20": "0.033675000071525574",
    "seed21": "-0.039243001490831375",
    "seed22": "-0.99866199493408203",
    "seed23": "-5.2254271507263184",
    "seed30": "0",
    "seed31": "0",
    "seed32": "0",
    "seed33": "1",
    "result00": "0.82380002737045288",
    "result01": "0.56685298681259155",
    "result02": "0.0055030002258718014",
    "result03": "39.929523468017578",
    "result10": "0.56587898731231689",
    "result11": "-0.82288402318954468",
    "result12": "0.051417998969554901",
    "result13": "-176.21714782714844",
    "result20": "0.033675000071525574",
    "result21": "-0.039243001490831375",
    "result22": "-0.99866199493408203",
    "result23": "-5.2254271507263184",
    "result30": "0",
    "result31": "0",
    "result32": "0",
    "result33": "1"
  },
  "checks": {
    "program_exit_zero": false,
    "converged": false,
    "legal_seed": true,
    "fitness": false,
    "nn_median": false,
    "nn_p90": false,
    "inlier_0_5m": false
  },
  "initialization_admitted": false,
  "map_icp_calls": 24,
  "local_icp_calls": 48,
  "local_scans_used": 49,
  "method": "UNCHANGED_LEGACY_FIRST_SEGMENT_24_PROPER_AXES",
  "candidate_rule": "R_seed=R_yaml*A_i; t_seed=t_yaml",
  "legacy_yaml_status": "NOT_A_RUNTIME_INITIALIZATION_UNLESS_ADMITTED",
  "GT_loaded": false,
  "independent_validation_executed": false,
  "normalized_map_executed": false,
  "F_executed": false,
  "G_executed": false
}


--- ACTUAL OUTPUT: initializer_candidates.csv
rank,candidate_id,converged,fitness,nn_median,nn_p90,inlier,translation_x,translation_y,translation_z,rotation_angle_deg
1,2,0,85678.3,292.344,306.598,0,39.9295,-176.217,-5.22543,177.279
2,7,0,85704.1,292.366,306.253,0,39.9295,-176.217,-5.22543,92.4512
3,6,0,85824.5,292.327,306.857,0,39.9295,-176.217,-5.22543,97.6679
4,3,0,85850.2,292.944,306.502,0,39.9295,-176.217,-5.22543,34.6467
5,19,0,86744.4,294.99,297.507,0,39.9295,-176.217,-5.22543,142.401
6,9,0,86753.8,295.222,299.436,0,39.9295,-176.217,-5.22543,124.544
7,13,0,87170.5,295.408,297.374,0,39.9295,-176.217,-5.22543,139.514
8,11,0,87334.2,295.628,299.847,0,39.9295,-176.217,-5.22543,178.908
9,23,0,87335,295.415,296.51,0,39.9295,-176.217,-5.22543,93.9548
10,22,0,87335.7,295.298,296.804,0,39.9295,-176.217,-5.22543,153.813
11,17,0,87343.6,295.571,299.717,0,39.9295,-176.217,-5.22543,140.765
12,12,0,87464.9,295.89,297.515,0,39.9295,-176.217,-5.22543,143.64
13,8,0,87666.4,295.69,300.17,0,39.9295,-176.217,-5.22543,177.245
14,18,0,87709.3,295.897,299.693,0,39.9295,-176.217,-5.22543,100.022
15,20,0,87754.7,296.094,297.839,0,39.9295,-176.217,-5.22543,157.748
16,14,0,87919.1,296.52,301.001,0,39.9295,-176.217,-5.22543,103.529
17,15,0,87928.6,296.023,301.087,0,39.9295,-176.217,-5.22543,101.548
18,21,0,88057.8,296.445,298.035,0,39.9295,-176.217,-5.22543,96.2085
19,16,0,88255.5,296.282,300.163,0,39.9295,-176.217,-5.22543,105.018
20,10,0,88298.4,296.198,302.548,0,39.9295,-176.217,-5.22543,55.5493
21,5,0,89387.8,298.835,311.364,0,39.9295,-176.217,-5.22543,156.634
22,0,0,89407.6,298.388,311.474,0,39.9295,-176.217,-5.22543,145.488
23,1,0,89535.4,298.91,311.425,0,39.9295,-176.217,-5.22543,178.825
24,4,0,89555.1,298.962,311.495,0,39.9295,-176.217,-5.22543,154.933


--- ACTUAL OUTPUT: initializer_registration.csv
index,stamp,converged,fitness,translation_m,rotation_deg,used
1,1.646e+09,1,0.00431498,0.00133222,0.0279765,1
2,1.646e+09,1,0.00600486,0.0109653,0.0279765,1
3,1.646e+09,1,0.00327573,0.00320588,0,1
4,1.646e+09,1,0.00428016,0.00582496,0,1
5,1.646e+09,1,0.0042688,0.00385549,0.0279765,1
6,1.646e+09,1,0.00114428,0.000777765,0,1
7,1.646e+09,1,0.00194617,0.00181214,0,1
8,1.646e+09,1,0.00376603,0.000527646,0.0395647,1
9,1.646e+09,1,0.00240581,0.000595835,0,1
10,1.646e+09,1,0.00391405,0.00268005,0.0279765,1
11,1.646e+09,1,0.0009914,0.00127643,0,1
12,1.646e+09,1,0.00395275,0.00328813,0,1
13,1.646e+09,1,0.00145251,0.00048631,0,1
14,1.646e+09,1,0.0048253,0.0042933,0,1
15,1.646e+09,1,0.00175133,0.000706717,0,1
16,1.646e+09,1,0.00357007,0.00190745,0,1
17,1.646e+09,1,0.00374428,0.0134538,0,1
18,1.646e+09,1,0.00504315,0.00238667,0,1
19,1.646e+09,1,0.00302233,0.00470431,0,1
20,1.646e+09,1,0.00353609,0.00324089,0,1
21,1.646e+09,1,0.00344171,0.00181009,0.0395647,1
22,1.646e+09,1,0.004358,0.00164422,0,1
23,1.646e+09,1,0.00268017,0.00366581,0,1
24,1.646e+09,1,0.00433838,0.000822618,0,1
25,1.646e+09,1,0.00272745,0.00344967,0,1
26,1.646e+09,1,0.00232554,0.000908592,0,1
27,1.646e+09,1,0.00364007,0.00461751,0.0279765,1
28,1.646e+09,1,0.00236547,0.00196049,0,1
29,1.646e+09,1,0.00330017,0.00334274,0,1
30,1.646e+09,1,0.00500152,0.00617542,0,1
31,1.646e+09,1,0.00283895,0.000923596,0,1
32,1.646e+09,1,0.00380145,0.00113537,0.0279765,1
33,1.646e+09,1,0.00188487,0.000360485,0,1
34,1.646e+09,1,0.00274092,0.00124638,0,1
35,1.646e+09,1,0.00382588,0.00188689,0,1
36,1.646e+09,1,0.00170075,0.00176687,0,1
37,1.646e+09,1,0.00444033,0.00208303,0.0279765,1
38,1.646e+09,1,0.00138018,0.00247953,0,1
39,1.646e+09,1,0.00114526,0.00130722,0,1
40,1.646e+09,1,0.00116295,0.00229661,0,1
41,1.646e+09,1,0.00374681,0.00724131,0,1
42,1.646e+09,1,0.00119902,0.000615577,0,1
43,1.646e+09,1,0.00400963,0.00372938,0,1
44,1.646e+09,1,0.0039726,0.00274213,0,1
45,1.646e+09,1,0.00302694,0.0039546,0.0279765,1
46,1.646e+09,1,0.00385828,0.00615582,0,1
47,1.646e+09,1,0.00371089,0.000814961,0,1
48,1.646e+09,1,0.00398948,0.00393956,0,1


--- ACTUAL OUTPUT: candidate_diagnostics.csv
candidate_id,source_points,converged,fitness,nn_median_m,nn_p90_m,inlier_0_5m,correction_m,correction_deg,seed_determinant,seed_orthogonality_error,seed_SE3_float_tolerance,icp_ms,diagnostics_ms,seed00,seed01,seed02,seed03,seed10,seed11,seed12,seed13,seed20,seed21,seed22,seed23,seed30,seed31,seed32,seed33,result00,result01,result02,result03,result10,result11,result12,result13,result20,result21,result22,result23,result30,result31,result32,result33
0,2569,0,89407.555885680224,298.3876953125,311.47390747070312,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,845.39041199999997,404.62972200000013,-0.82380002737045288,0.56685298681259155,-0.0055030002258718014,39.929523468017578,-0.56587898731231689,-0.82288402318954468,-0.051417998969554901,-176.21714782714844,-0.033675000071525574,-0.039243001490831375,0.99866199493408203,-5.2254271507263184,0,0,0,1,-0.82380002737045288,0.56685298681259155,-0.0055030002258718014,39.929523468017578,-0.56587898731231689,-0.82288402318954468,-0.051417998969554901,-176.21714782714844,-0.033675000071525574,-0.039243001490831375,0.99866199493408203,-5.2254271507263184,0,0,0,1
1,2569,0,89535.375282819194,298.91000366210938,311.42547607421875,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,803.00944400000003,411.32028500000001,-0.82380002737045288,-0.56685298681259155,0.0055030002258718014,39.929523468017578,-0.56587898731231689,0.82288402318954468,0.051417998969554901,-176.21714782714844,-0.033675000071525574,0.039243001490831375,-0.99866199493408203,-5.2254271507263184,0,0,0,1,-0.82380002737045288,-0.56685298681259155,0.0055030002258718014,39.929523468017578,-0.56587898731231689,0.82288402318954468,0.051417998969554901,-176.21714782714844,-0.033675000071525574,0.039243001490831375,-0.99866199493408203,-5.2254271507263184,0,0,0,1
2,2569,0,85678.30752329457,292.34402465820312,306.59817504882812,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,801.82723199999998,403.36761200000012,0.82380002737045288,0.56685298681259155,0.0055030002258718014,39.929523468017578,0.56587898731231689,-0.82288402318954468,0.051417998969554901,-176.21714782714844,0.033675000071525574,-0.039243001490831375,-0.99866199493408203,-5.2254271507263184,0,0,0,1,0.82380002737045288,0.56685298681259155,0.0055030002258718014,39.929523468017578,0.56587898731231689,-0.82288402318954468,0.051417998969554901,-176.21714782714844,0.033675000071525574,-0.039243001490831375,-0.99866199493408203,-5.2254271507263184,0,0,0,1
3,2569,0,85850.214153427893,292.94442749023438,306.50225830078125,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,819.75205400000004,403.09278500000005,0.82380002737045288,-0.56685298681259155,-0.0055030002258718014,39.929523468017578,0.56587898731231689,0.82288402318954468,-0.051417998969554901,-176.21714782714844,0.033675000071525574,0.039243001490831375,0.99866199493408203,-5.2254271507263184,0,0,0,1,0.82380002737045288,-0.56685298681259155,-0.0055030002258718014,39.929523468017578,0.56587898731231689,0.82288402318954468,-0.051417998969554901,-176.21714782714844,0.033675000071525574,0.039243001490831375,0.99866199493408203,-5.2254271507263184,0,0,0,1
4,2569,0,89555.135877895096,298.96170043945312,311.49484252929688,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,805.23222799999996,426.81461300000012,-0.82380002737045288,0.0055030002258718014,0.56685298681259155,39.929523468017578,-0.56587898731231689,0.051417998969554901,-0.82288402318954468,-176.21714782714844,-0.033675000071525574,-0.99866199493408203,-0.039243001490831375,-5.2254271507263184,0,0,0,1,-0.82380002737045288,0.0055030002258718014,0.56685298681259155,39.929523468017578,-0.56587898731231689,0.051417998969554901,-0.82288402318954468,-176.21714782714844,-0.033675000071525574,-0.99866199493408203,-0.039243001490831375,-5.2254271507263184,0,0,0,1
5,2569,0,89387.794950004871,298.83499145507812,311.36355590820312,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,814.82311000000004,411.7765609999999,-0.82380002737045288,-0.0055030002258718014,-0.56685298681259155,39.929523468017578,-0.56587898731231689,-0.051417998969554901,0.82288402318954468,-176.21714782714844,-0.033675000071525574,0.99866199493408203,0.039243001490831375,-5.2254271507263184,0,0,0,1,-0.82380002737045288,-0.0055030002258718014,-0.56685298681259155,39.929523468017578,-0.56587898731231689,-0.051417998969554901,0.82288402318954468,-176.21714782714844,-0.033675000071525574,0.99866199493408203,0.039243001490831375,-5.2254271507263184,0,0,0,1
6,2569,0,85824.450457984625,292.3272705078125,306.85736083984375,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,819.62913200000003,427.05556699999988,0.82380002737045288,-0.0055030002258718014,0.56685298681259155,39.929523468017578,0.56587898731231689,-0.051417998969554901,-0.82288402318954468,-176.21714782714844,0.033675000071525574,0.99866199493408203,-0.039243001490831375,-5.2254271507263184,0,0,0,1,0.82380002737045288,-0.0055030002258718014,0.56685298681259155,39.929523468017578,0.56587898731231689,-0.051417998969554901,-0.82288402318954468,-176.21714782714844,0.033675000071525574,0.99866199493408203,-0.039243001490831375,-5.2254271507263184,0,0,0,1
7,2569,0,85704.0711487933,292.36569213867188,306.25283813476562,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,794.53812600000003,404.90950800000007,0.82380002737045288,0.0055030002258718014,-0.56685298681259155,39.929523468017578,0.56587898731231689,0.051417998969554901,0.82288402318954468,-176.21714782714844,0.033675000071525574,-0.99866199493408203,0.039243001490831375,-5.2254271507263184,0,0,0,1,0.82380002737045288,0.0055030002258718014,-0.56685298681259155,39.929523468017578,0.56587898731231689,0.051417998969554901,0.82288402318954468,-176.21714782714844,0.033675000071525574,-0.99866199493408203,0.039243001490831375,-5.2254271507263184,0,0,0,1
8,2569,0,87666.404665604321,295.68994140625,300.16983032226562,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,798.50786200000005,402.50806899999986,0.56685298681259155,-0.82380002737045288,0.0055030002258718014,39.929523468017578,-0.82288402318954468,-0.56587898731231689,0.051417998969554901,-176.21714782714844,-0.039243001490831375,-0.033675000071525574,-0.99866199493408203,-5.2254271507263184,0,0,0,1,0.56685298681259155,-0.82380002737045288,0.0055030002258718014,39.929523468017578,-0.82288402318954468,-0.56587898731231689,0.051417998969554901,-176.21714782714844,-0.039243001490831375,-0.033675000071525574,-0.99866199493408203,-5.2254271507263184,0,0,0,1
9,2569,0,86753.776445114825,295.22174072265625,299.43582153320312,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,818.37030500000003,403.69104899999991,-0.56685298681259155,-0.82380002737045288,-0.0055030002258718014,39.929523468017578,0.82288402318954468,-0.56587898731231689,-0.051417998969554901,-176.21714782714844,0.039243001490831375,-0.033675000071525574,0.99866199493408203,-5.2254271507263184,0,0,0,1,-0.56685298681259155,-0.82380002737045288,-0.0055030002258718014,39.929523468017578,0.82288402318954468,-0.56587898731231689,-0.051417998969554901,-176.21714782714844,0.039243001490831375,-0.033675000071525574,0.99866199493408203,-5.2254271507263184,0,0,0,1
10,2569,0,88298.411812110746,296.197998046875,302.54757690429688,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,799.32920100000001,402.704115,0.56685298681259155,0.82380002737045288,-0.0055030002258718014,39.929523468017578,-0.82288402318954468,0.56587898731231689,-0.051417998969554901,-176.21714782714844,-0.039243001490831375,0.033675000071525574,0.99866199493408203,-5.2254271507263184,0,0,0,1,0.56685298681259155,0.82380002737045288,-0.0055030002258718014,39.929523468017578,-0.82288402318954468,0.56587898731231689,-0.051417998969554901,-176.21714782714844,-0.039243001490831375,0.033675000071525574,0.99866199493408203,-5.2254271507263184,0,0,0,1
11,2569,0,87334.200242068895,295.62835693359375,299.846923828125,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,800.10144200000002,422.66370700000004,-0.56685298681259155,0.82380002737045288,0.0055030002258718014,39.929523468017578,0.82288402318954468,0.56587898731231689,0.051417998969554901,-176.21714782714844,0.039243001490831375,0.033675000071525574,-0.99866199493408203,-5.2254271507263184,0,0,0,1,-0.56685298681259155,0.82380002737045288,0.0055030002258718014,39.929523468017578,0.82288402318954468,0.56587898731231689,0.051417998969554901,-176.21714782714844,0.039243001490831375,0.033675000071525574,-0.99866199493408203,-5.2254271507263184,0,0,0,1
12,2569,0,87464.918611935573,295.88970947265625,297.51516723632812,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,807.13709200000005,406.97081900000001,-0.0055030002258718014,-0.82380002737045288,0.56685298681259155,39.929523468017578,-0.051417998969554901,-0.56587898731231689,-0.82288402318954468,-176.21714782714844,0.99866199493408203,-0.033675000071525574,-0.039243001490831375,-5.2254271507263184,0,0,0,1,-0.0055030002258718014,-0.82380002737045288,0.56685298681259155,39.929523468017578,-0.051417998969554901,-0.56587898731231689,-0.82288402318954468,-176.21714782714844,0.99866199493408203,-0.033675000071525574,-0.039243001490831375,-5.2254271507263184,0,0,0,1
13,2569,0,87170.538600257889,295.40805053710938,297.37350463867188,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,794.58240499999999,403.10989600000005,0.0055030002258718014,-0.82380002737045288,-0.56685298681259155,39.929523468017578,0.051417998969554901,-0.56587898731231689,0.82288402318954468,-176.21714782714844,-0.99866199493408203,-0.033675000071525574,0.039243001490831375,-5.2254271507263184,0,0,0,1,0.0055030002258718014,-0.82380002737045288,-0.56685298681259155,39.929523468017578,0.051417998969554901,-0.56587898731231689,0.82288402318954468,-176.21714782714844,-0.99866199493408203,-0.033675000071525574,0.039243001490831375,-5.2254271507263184,0,0,0,1
14,2569,0,87919.128786127869,296.519775390625,301.000732421875,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,797.58916299999999,403.79462500000011,0.0055030002258718014,0.82380002737045288,0.56685298681259155,39.929523468017578,0.051417998969554901,0.56587898731231689,-0.82288402318954468,-176.21714782714844,-0.99866199493408203,0.033675000071525574,-0.039243001490831375,-5.2254271507263184,0,0,0,1,0.0055030002258718014,0.82380002737045288,0.56685298681259155,39.929523468017578,0.051417998969554901,0.56587898731231689,-0.82288402318954468,-176.21714782714844,-0.99866199493408203,0.033675000071525574,-0.039243001490831375,-5.2254271507263184,0,0,0,1
15,2569,0,87928.581549240946,296.02316284179688,301.08673095703125,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,810.19588399999998,404.73602099999994,-0.0055030002258718014,0.82380002737045288,-0.56685298681259155,39.929523468017578,-0.051417998969554901,0.56587898731231689,0.82288402318954468,-176.21714782714844,0.99866199493408203,0.033675000071525574,0.039243001490831375,-5.2254271507263184,0,0,0,1,-0.0055030002258718014,0.82380002737045288,-0.56685298681259155,39.929523468017578,-0.051417998969554901,0.56587898731231689,0.82288402318954468,-176.21714782714844,0.99866199493408203,0.033675000071525574,0.039243001490831375,-5.2254271507263184,0,0,0,1
16,2569,0,88255.474746983266,296.28228759765625,300.16281127929688,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,799.28701699999999,404.17644899999993,0.56685298681259155,-0.0055030002258718014,-0.82380002737045288,39.929523468017578,-0.82288402318954468,-0.051417998969554901,-0.56587898731231689,-176.21714782714844,-0.039243001490831375,0.99866199493408203,-0.033675000071525574,-5.2254271507263184,0,0,0,1,0.56685298681259155,-0.0055030002258718014,-0.82380002737045288,39.929523468017578,-0.82288402318954468,-0.051417998969554901,-0.56587898731231689,-176.21714782714844,-0.039243001490831375,0.99866199493408203,-0.033675000071525574,-5.2254271507263184,0,0,0,1
17,2569,0,87343.560532429939,295.57101440429688,299.71685791015625,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,812.00883599999997,403.13636500000007,-0.56685298681259155,0.0055030002258718014,-0.82380002737045288,39.929523468017578,0.82288402318954468,0.051417998969554901,-0.56587898731231689,-176.21714782714844,0.039243001490831375,-0.99866199493408203,-0.033675000071525574,-5.2254271507263184,0,0,0,1,-0.56685298681259155,0.0055030002258718014,-0.82380002737045288,39.929523468017578,0.82288402318954468,0.051417998969554901,-0.56587898731231689,-176.21714782714844,0.039243001490831375,-0.99866199493408203,-0.033675000071525574,-5.2254271507263184,0,0,0,1
18,2569,0,87709.346219346044,295.8974609375,299.69342041015625,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,802.34455000000003,406.40789399999994,0.56685298681259155,0.0055030002258718014,0.82380002737045288,39.929523468017578,-0.82288402318954468,0.051417998969554901,0.56587898731231689,-176.21714782714844,-0.039243001490831375,-0.99866199493408203,0.033675000071525574,-5.2254271507263184,0,0,0,1,0.56685298681259155,0.0055030002258718014,0.82380002737045288,39.929523468017578,-0.82288402318954468,0.051417998969554901,0.56587898731231689,-176.21714782714844,-0.039243001490831375,-0.99866199493408203,0.033675000071525574,-5.2254271507263184,0,0,0,1
19,2569,0,86744.416020946868,294.98968505859375,297.50665283203125,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,809.17773,410.55412100000001,-0.56685298681259155,-0.0055030002258718014,0.82380002737045288,39.929523468017578,0.82288402318954468,-0.051417998969554901,0.56587898731231689,-176.21714782714844,0.039243001490831375,0.99866199493408203,0.033675000071525574,-5.2254271507263184,0,0,0,1,-0.56685298681259155,-0.0055030002258718014,0.82380002737045288,39.929523468017578,0.82288402318954468,-0.051417998969554901,0.56587898731231689,-176.21714782714844,0.039243001490831375,0.99866199493408203,0.033675000071525574,-5.2254271507263184,0,0,0,1
20,2569,0,87754.662490268587,296.09432983398438,297.83901977539062,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,800.04703600000005,400.99250199999994,0.0055030002258718014,0.56685298681259155,-0.82380002737045288,39.929523468017578,0.051417998969554901,-0.82288402318954468,-0.56587898731231689,-176.21714782714844,-0.99866199493408203,-0.039243001490831375,-0.033675000071525574,-5.2254271507263184,0,0,0,1,0.0055030002258718014,0.56685298681259155,-0.82380002737045288,39.929523468017578,0.051417998969554901,-0.82288402318954468,-0.56587898731231689,-176.21714782714844,-0.99866199493408203,-0.039243001490831375,-0.033675000071525574,-5.2254271507263184,0,0,0,1
21,2569,0,88057.830205089529,296.444580078125,298.0352783203125,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,801.88129000000004,404.59941099999992,-0.0055030002258718014,-0.56685298681259155,-0.82380002737045288,39.929523468017578,-0.051417998969554901,0.82288402318954468,-0.56587898731231689,-176.21714782714844,0.99866199493408203,0.039243001490831375,-0.033675000071525574,-5.2254271507263184,0,0,0,1,-0.0055030002258718014,-0.56685298681259155,-0.82380002737045288,39.929523468017578,-0.051417998969554901,0.82288402318954468,-0.56587898731231689,-176.21714782714844,0.99866199493408203,0.039243001490831375,-0.033675000071525574,-5.2254271507263184,0,0,0,1
22,2569,0,87335.669776664072,295.29824829101562,296.8043212890625,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,801.81996900000001,407.08054000000004,-0.0055030002258718014,0.56685298681259155,0.82380002737045288,39.929523468017578,-0.051417998969554901,-0.82288402318954468,0.56587898731231689,-176.21714782714844,0.99866199493408203,-0.039243001490831375,0.033675000071525574,-5.2254271507263184,0,0,0,1,-0.0055030002258718014,0.56685298681259155,0.82380002737045288,39.929523468017578,-0.051417998969554901,-0.82288402318954468,0.56587898731231689,-176.21714782714844,0.99866199493408203,-0.039243001490831375,0.033675000071525574,-5.2254271507263184,0,0,0,1
23,2569,0,87335.004911322496,295.41543579101562,296.50970458984375,0,0,0,0.99999988079071045,2.0917686924804002e-06,1,802.01845900000001,405.61246599999993,0.0055030002258718014,-0.56685298681259155,0.82380002737045288,39.929523468017578,0.051417998969554901,0.82288402318954468,0.56587898731231689,-176.21714782714844,-0.99866199493408203,0.039243001490831375,0.033675000071525574,-5.2254271507263184,0,0,0,1,0.0055030002258718014,-0.56685298681259155,0.82380002737045288,39.929523468017578,0.051417998969554901,0.82288402318954468,0.56587898731231689,-176.21714782714844,-0.99866199493408203,0.039243001490831375,0.033675000071525574,-5.2254271507263184,0,0,0,1


--- ACTUAL OUTPUT: math_test.log
P32_TRANSFER_MATH_PASS axes=24 unique_proper fixed_translation gyro_and_point=PASS no_extrapolation=PASS active_yaml=PASS det=1 orthogonality=2.09177e-06


--- ACTUAL OUTPUT: input_bridge.log
BOOTSTRAP_BRIDGE_PASS scans=49 reference_ns=1645999726479818106


--- ACTUAL OUTPUT: initializer.log
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
[pcl::IterativeClosestPoint::computeTransformation] Not enough correspondences found. Relax your threshold parameters.
FIRST_SEGMENT_INITIALIZER_PASS scans=49 used=49 candidates=24 best_converged=0 best_fitness=85678.3
P32_CALL_LEDGER local_icp=48 map_icp=24


--- ACTUAL OUTPUT: initializer_resource.txt
Command exited with non-zero status 7
user_cpu_sec=37.78
system_cpu_sec=3.44
peak_rss_kib=395820
wall_sec=41.27
