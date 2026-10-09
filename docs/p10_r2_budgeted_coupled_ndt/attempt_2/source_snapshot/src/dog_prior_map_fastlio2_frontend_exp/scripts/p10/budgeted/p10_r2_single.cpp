#include "shadow_logging.hpp"
#include <chrono>
#include <ctime>
#include <iostream>
#include <map>

namespace paper=dog_prior_map_fastlio2_frontend_exp;
namespace {
std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> result; std::istringstream in(text); std::string v;
  while(std::getline(in,v,delimiter)) result.push_back(v); return result;
}
paper::Pose3d pose(const std::string& text) {
  auto v=split(text,';'); if(v.size()!=7) throw std::runtime_error("pose_requires_seven_values");
  paper::Pose3d p; p.position=Eigen::Vector3d(std::stod(v[0]),std::stod(v[1]),std::stod(v[2]));
  p.orientation=Eigen::Quaterniond(std::stod(v[6]),std::stod(v[3]),std::stod(v[4]),std::stod(v[5]));
  if(!p.position.allFinite() || !p.orientation.coeffs().allFinite() || p.orientation.norm()<1e-8)
    throw std::runtime_error("nonfinite_input_pose");
  p.orientation.normalize(); return p;
}
paper::RegistrationCloud cloud(const std::string& path, std::size_t count) {
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in || in.tellg()!=static_cast<std::streamoff>(count*12)) throw std::runtime_error("raw_size_mismatch");
  in.seekg(0); paper::RegistrationCloud c(count);
  for(auto& point:c) { float xyz[3]; in.read(reinterpret_cast<char*>(xyz),12); point={xyz[0],xyz[1],xyz[2]}; }
  if(!in) throw std::runtime_error("raw_read_failed"); return c;
}
}
int main(int argc,char** argv) {
  try {
    if(argc==2 && std::string(argv[1])=="--self-test") return paper::coupledNdtSelfTest()?0:1;
    if(argc!=6) throw std::runtime_error("usage: p10_r2_single MAP MANIFEST OUTPUT METHOD FIXED16");
    const auto config=p10log::config(argv[4],std::string(argv[5])=="1");
    std::ifstream manifest(argv[2]); std::string line; if(!std::getline(manifest,line)) throw std::runtime_error("manifest_missing");
    auto fields=split(line,','); std::map<std::string,std::size_t> columns;
    for(std::size_t i=0;i<fields.size();++i) columns[fields[i]]=i;
    paper::CurrentFrameNdtRegistration ndt{paper::CurrentFrameNdtParameters{}}; std::string reason;
    if(!ndt.loadMap(argv[1],&reason)) throw std::runtime_error("map:"+reason);
    p10log::Logger logger(argv[3]); std::size_t frames=0;
    while(std::getline(manifest,line)) {
      const auto values=split(line,',');
      auto value=[&](const std::string& name)->std::string { return values.at(columns.at(name)); };
      const auto began=std::chrono::steady_clock::now(); const auto cpu=std::clock();
      const uint64_t tx=std::stoull(value("transaction_id")), stamp=std::stoull(value("stamp_ns"));
      auto source=cloud(value("raw_cloud_file"),std::stoull(value("raw_point_count")));
      const auto prepared=paper::preprocessRegistrationCloud(source,paper::CurrentFrameNdtParameters{});
      if(prepared.size()!=std::stoull(value("source_count")) || paper::registrationCloudHash(prepared)!=std::stoull(value("source_hash")))
        throw std::runtime_error("source_admission_mismatch_TX"+std::to_string(tx));
      if(ndt.targetPointCount()!=std::stoull(value("target_count"))) throw std::runtime_error("target_count_mismatch");
      paper::CurrentFrameNdtResult nominal;
      if(!ndt.align(stamp,source,pose(value("prediction_pose")),&nominal,&reason)) throw std::runtime_error("align:"+reason);
      paper::CoupledShadowResult shadow;
      shadow.nominal_pose=p10log::matrix(nominal.raw_map_T_lidar);
      shadow.prediction_pose=p10log::matrix(nominal.initial_map_T_lidar);
      shadow.recommended_pose=shadow.nominal_pose;
      shadow.complete_ndt_calls=nominal.status==paper::CurrentFrameNdtStatus::INSUFFICIENT_POINTS?0:1;
      shadow.status="NOMINAL_INEFFECTIVE_SHADOW_SKIPPED";
      if(nominal.effective && !ndt.shadow(nominal,config,&shadow,&reason)) throw std::runtime_error("shadow:"+reason);
      logger.write(tx,nominal,shadow,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count(),
        1000.0*(std::clock()-cpu)/CLOCKS_PER_SEC); ++frames;
    }
    p10log::resourceReceipt(argv[3]); std::cout << "frames=" << frames << " nominal_unchanged=true\n"; return 0;
  } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
