// ELF interposition for measurement only. Every call forwards unchanged arguments
// exactly once to the pristine author implementation; no measurement affects UKF/NDT.
#include <pclomp/ndt_omp.h>
#include <sensor_msgs/PointCloud2.h>
#include <dlfcn.h>
#include <openssl/sha.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <sys/resource.h>
using Point=pcl::PointXYZI;
using Cloud=pcl::PointCloud<Point>;
using Ndt=pclomp::NormalDistributionsTransform<Point,Point>;
using Clock=std::chrono::steady_clock;
thread_local uint64_t scan_stamp=0;
struct Trace {
 std::ofstream ndt,cycles;std::vector<float> baseline;std::string before_sha;const void* target_address=nullptr;
 size_t calls=0;bool frozen=true;std::mutex mutex;
 Trace(){const char* d=std::getenv("P63_TRACE_DIR");if(!d)std::abort();
  ndt.open(std::string(d)+"/NATIVE_NDT.csv");cycles.open(std::string(d)+"/CALLBACKS.csv");
  ndt<<"stamp_ns,ndt_compute_ms,diagnostic_ms,iterations,converged,fitness_all_nn_m2,probability,source_points,target_points,map_frozen,target_xyz_sha256,tx,ty,tz,qx,qy,qz,qw,peak_RSS_KiB\n";
  cycles<<"stamp_ns,callback_ms,ndt_calls_so_far\n";
 }
};
Trace& trace(){static Trace t;return t;}
template<class F> F original(const char* symbol){void* p=dlsym(RTLD_NEXT,symbol);if(!p){fprintf(stderr,"P63_TRACE_RESOLVE_FAILED %s\n",symbol);std::abort();}return reinterpret_cast<F>(p);}
std::string sha(const std::vector<float>& v){unsigned char h[32];SHA256(reinterpret_cast<const unsigned char*>(v.data()),v.size()*4,h);std::ostringstream s;for(auto b:h)s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);return s.str();}
extern "C" void ndt_trace(void*,Cloud&,const Eigen::Matrix4f&) asm("_ZN6pclomp28NormalDistributionsTransformIN3pcl9PointXYZIES2_E21computeTransformationERNS1_10PointCloudIS2_EERKN5Eigen6MatrixIfLi4ELi4ELi0ELi4ELi4EEE");
extern "C" void ndt_trace(void* self,Cloud& output,const Eigen::Matrix4f& guess){
 static auto real=original<void(*)(void*,Cloud&,const Eigen::Matrix4f&)>("_ZN6pclomp28NormalDistributionsTransformIN3pcl9PointXYZIES2_E21computeTransformationERNS1_10PointCloudIS2_EERKN5Eigen6MatrixIfLi4ELi4ELi0ELi4ELi4EEE");
 auto* n=static_cast<Ndt*>(self);auto& t=trace();
 auto target=n->getInputTarget();
 if(t.calls==0){t.target_address=target.get();t.baseline.reserve(target->size()*3);for(const auto& p:*target){t.baseline.push_back(p.x);t.baseline.push_back(p.y);t.baseline.push_back(p.z);}t.before_sha=sha(t.baseline);}
 auto begin=Clock::now();real(self,output,guess);auto end=Clock::now();
 // These reads and an additional all-point NN fitness query are diagnostic overhead.
 double fitness=n->getFitnessScore();const auto m=n->getFinalTransformation();Eigen::Quaternionf q(m.block<3,3>(0,0));
 t.frozen=t.frozen&&target.get()==t.target_address&&target->size()*3==t.baseline.size();
 if(t.frozen)for(size_t i=0;i<target->size();++i)if(std::memcmp(target->points[i].data,t.baseline.data()+3*i,12)!=0){t.frozen=false;break;}
 struct rusage usage;getrusage(RUSAGE_SELF,&usage);
 auto diagnostic_end=Clock::now();std::lock_guard<std::mutex> lock(t.mutex);++t.calls;
 t.ndt<<std::setprecision(17)<<scan_stamp<<','<<std::chrono::duration<double,std::milli>(end-begin).count()<<','<<std::chrono::duration<double,std::milli>(diagnostic_end-end).count()<<','<<n->getFinalNumIteration()<<','<<n->hasConverged()<<','<<fitness<<','<<n->getTransformationProbability()<<','<<n->getInputSource()->size()<<','<<target->size()<<','<<t.frozen<<','<<t.before_sha<<','<<m(0,3)<<','<<m(1,3)<<','<<m(2,3)<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<usage.ru_maxrss<<'\n';t.ndt.flush();
}
extern "C" void callback_trace(void*,const sensor_msgs::PointCloud2ConstPtr&) asm("_ZN16hdl_localization22HdlLocalizationNodelet15points_callbackERKN5boost10shared_ptrIKN11sensor_msgs12PointCloud2_ISaIvEEEEE");
extern "C" void callback_trace(void* self,const sensor_msgs::PointCloud2ConstPtr& cloud){
 static auto real=original<void(*)(void*,const sensor_msgs::PointCloud2ConstPtr&)>("_ZN16hdl_localization22HdlLocalizationNodelet15points_callbackERKN5boost10shared_ptrIKN11sensor_msgs12PointCloud2_ISaIvEEEEE");
 scan_stamp=cloud->header.stamp.toNSec();auto begin=Clock::now();real(self,cloud);auto end=Clock::now();auto& t=trace();
 std::lock_guard<std::mutex> lock(t.mutex);t.cycles<<std::setprecision(17)<<scan_stamp<<','<<std::chrono::duration<double,std::milli>(end-begin).count()<<','<<t.calls<<'\n';t.cycles.flush();
}
