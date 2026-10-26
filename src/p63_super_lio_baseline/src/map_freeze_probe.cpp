// Engineering fixture using unchanged GPLv3 Super-LIO classes, not a tracking experiment.
#include "lio/super_lio_reloc.h"
#include "ros/ROSWrapper.h"
#include <openssl/sha.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <sys/resource.h>
using namespace LI2Sup;
using namespace BASIC;
struct UpdateAccess : SuperLIO { using SuperLIO::UpdateMap; };
class FreezeProbe : public SuperLIOReLoc {
public:
  std::vector<float> snapshot() const { std::vector<float> v;ivox_->getMap(v);return v; }
  void exerciseDisabledUpdates() {
    ds_undistort_->resize(1);points_body_v3_.assign(1,V3(1000,2000,3000));
    const auto update=&UpdateAccess::UpdateMap;
    for(int i=0;i<600;++i)(this->*update)(); // Virtual dispatch to real relocation override.
  }
  size_t exerciseQueries(const std::vector<float>& v) {
    size_t supported=0;
    for(size_t j=0;j<1000;++j) {
      size_t i=(j*(v.size()/3)/1000)*3;KNNHeapType q;q.reset();
      ivox_->reset_max_group();ivox_->getTopK(V3(v[i],v[i+1],v[i+2]),q);
      if(q.count>=4)++supported;
    }
    return supported;
  }
};
std::string digest(const std::vector<float>& v) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(v.data()),v.size()*sizeof(float),hash);
  std::ostringstream out;for(unsigned char b:hash)out<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);return out.str();
}
int main(int argc,char**argv) {
  ros::init(argc,argv,"p63_map_freeze_probe");ros::NodeHandle nh;LoadParamFromRos(nh);
  if(g_update_map||g_save_map)throw std::runtime_error("probe requires frozen target and no save");
  std::string output;nh.param<std::string>("/p63/probe_receipt",output,"");
  if(output.empty())throw std::runtime_error("missing exclusive probe receipt");
  auto wrapper=std::make_shared<ROSWrapper>();FreezeProbe lio;lio.setROSWrapper(wrapper);
  auto start=std::chrono::steady_clock::now();lio.init();auto before=lio.snapshot();
  if(before.empty())throw std::runtime_error("native PCD/OctVox import failed");
  double load_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  auto begin=std::chrono::steady_clock::now();lio.exerciseDisabledUpdates();size_t supported=lio.exerciseQueries(before);
  auto after=lio.snapshot();if(before!=after)throw std::runtime_error("prior geometry mutated");
  double probe_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  const auto until=ros::WallTime::now()+ros::WallDuration(3.0);
  while(ros::ok()&&ros::WallTime::now()<until){wrapper->spinOnce();ros::WallDuration(.002).sleep();}
  if(lio.snapshot()!=before)throw std::runtime_error("target mutated while spinning");
  struct rusage usage;getrusage(RUSAGE_SELF,&usage);
  std::ofstream f(output);f<<std::setprecision(17)<<"{\n\"mode\":\"MAP_FREEZE_ENGINEERING_ONLY\",\n\"target_points\":"<<before.size()/3
    <<",\n\"target_before_sha256\":\""<<digest(before)<<"\",\n\"target_after_sha256\":\""<<digest(after)
    <<"\",\n\"disabled_update_calls\":600,\n\"knn_queries\":1000,\n\"supported_queries\":"<<supported
    <<",\n\"map_load_ms\":"<<load_ms<<",\n\"probe_ms\":"<<probe_ms<<",\n\"peak_RSS_KiB\":"<<usage.ru_maxrss
    <<",\n\"actual_MID360_frames\":0,\n\"native_NDT_calls\":0,\n\"ESKF_updates\":0\n}\n";
  ROS_INFO("P63_MAP_FREEZE_PROBE_PASS target=%zu queries=%zu actual_sensor_frames=0",before.size()/3,supported);
}
