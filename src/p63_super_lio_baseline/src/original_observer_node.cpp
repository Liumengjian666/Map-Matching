// Read-only observer around the unchanged author's ROSWrapper + SuperLIOReLoc.
#include "lio/super_lio_reloc.h"
#include "ros/ROSWrapper.h"
#include <openssl/sha.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <csignal>
#include <chrono>
#include <sys/resource.h>
using namespace LI2Sup;using namespace BASIC;
class ObservedOriginal : public SuperLIOReLoc {
public:
  int frame() const {return frame_num_;}
  bool ready() const {return kf_->init_;}
  auto state() const {return kf_->GetNavState();}
  std::vector<float> mapSnapshot() const {std::vector<float> v;ivox_->getMap(v);return v;}
  size_t matched() const {
    size_t n=0;for(size_t i=0;i<points_body_v3_.size();++i)if(effect_mask_[i]&&effect_knn_mask_[i])++n;return n;
  }
  size_t sourceSize() const {return ds_undistort_->size();}
  double finalPlaneRms() const {
    const auto t=kf_->GetSE3();double sum=0;size_t n=0;
    for(size_t i=0;i<points_body_v3_.size();++i)if(effect_mask_[i]&&effect_knn_mask_[i]) {
      const auto p=t*points_body_v3_[i];const auto& a=abcd_vec_[i];double e=a[0]*p[0]+a[1]*p[1]+a[2]*p[2]+a[3];sum+=e*e;++n;
    }return n?std::sqrt(sum/n):NAN;
  }
};
std::string digest(const std::vector<float>& v) {
 unsigned char h[SHA256_DIGEST_LENGTH];SHA256(reinterpret_cast<const unsigned char*>(v.data()),v.size()*sizeof(float),h);
 std::ostringstream s;for(auto x:h)s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(x);return s.str();
}
void signalHandler(int){g_flag_run=false;}
int main(int argc,char**argv) {
 ros::init(argc,argv,"p63_original_observer");signal(SIGINT,signalHandler);signal(SIGTERM,signalHandler);ros::NodeHandle nh;
 LoadParamFromRos(nh);if(g_update_map||g_save_map)throw std::runtime_error("prior map must be frozen and save disabled");
 std::string dir;nh.param<std::string>("/p63/output_dir",dir,"");if(dir.empty())throw std::runtime_error("missing output directory");
 std::ofstream log(dir+"/ORIGINAL_TRAJECTORY.csv");log<<"stamp_s,frame,px,py,pz,qx,qy,qz,qw,vx,vy,vz,source_points,plane_matches,final_cached_plane_rms,process_ms,spin_and_process_ms\n";
 auto wrapper=std::make_shared<ROSWrapper>();ObservedOriginal lio;lio.setROSWrapper(wrapper);
 auto start=std::chrono::steady_clock::now();lio.init();auto before=lio.mapSnapshot();if(before.empty())throw std::runtime_error("empty native map");
 std::ofstream initial(dir+"/TARGET_BEFORE.json");initial<<"{\"points\":"<<before.size()/3<<",\"sha256\":\""<<digest(before)<<"\"}\n";initial.close();
 bool failed=false;size_t outputs=0,unsupported=0;ros::Rate rate(500);
 while(ros::ok()&&g_flag_run) {
  auto loop=std::chrono::steady_clock::now();wrapper->spinOnce();const int old=lio.frame();auto process=std::chrono::steady_clock::now();lio.process();
  const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-process).count();
  const double total=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-loop).count();
  if(lio.frame()!=old && lio.ready()) {
    const auto x=lio.state();const auto q=x.R.coeffs();const auto matches=lio.matched();++outputs;
    log<<std::setprecision(17)<<x.timestamp<<','<<lio.frame()<<','<<x.p[0]<<','<<x.p[1]<<','<<x.p[2]<<','<<q[0]<<','<<q[1]<<','<<q[2]<<','<<q[3]<<','<<x.v[0]<<','<<x.v[1]<<','<<x.v[2]<<','<<lio.sourceSize()<<','<<matches<<','<<lio.finalPlaneRms()<<','<<ms<<','<<total<<'\n';log.flush();
    unsupported=matches<4?unsupported+1:0;
    if(!x.p.allFinite()||!x.v.allFinite()||!q.allFinite()||unsupported>=20) {
      ROS_ERROR("P63_TRACKING_FAILURE_STOP finite=%d unsupported=%zu",x.p.allFinite(),unsupported);failed=true;break;
    }
  }
  rate.sleep();
 }
 auto after=lio.mapSnapshot();struct rusage r;getrusage(RUSAGE_SELF,&r);
 std::ofstream final(dir+"/OBSERVER_RECEIPT.json");final<<std::setprecision(17)<<"{\"map_points\":"<<after.size()/3<<",\"map_before_sha256\":\""<<digest(before)<<"\",\"map_after_sha256\":\""<<digest(after)<<"\",\"map_frozen\":"<<(before==after?"true":"false")<<",\"tracked_outputs\":"<<outputs<<",\"tracking_failure_stop\":"<<(failed?"true":"false")<<",\"wall_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<",\"peak_RSS_KiB\":"<<r.ru_maxrss<<"}\n";
 lio.saveMap();lio.printTimeRecord();return failed||before!=after?2:0;
}
