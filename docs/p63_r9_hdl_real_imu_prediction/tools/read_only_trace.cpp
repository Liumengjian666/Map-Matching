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
  ndt<<"stamp_ns,ndt_compute_ms,diagnostic_ms,iterations,converged,fitness_all_nn_m2,probability,source_points,target_points,map_frozen,target_xyz_sha256,tx,ty,tz,qx,qy,qz,qw,peak_RSS_KiB,source_xyz_sha256";
  for(int r=0;r<4;++r)for(int c=0;c<4;++c)ndt<<",guess_"<<r<<c;ndt<<'\n';
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
 const Eigen::Matrix4f recorded_guess=guess;
 auto begin=Clock::now();real(self,output,guess);auto end=Clock::now();
 // These reads and an additional all-point NN fitness query are diagnostic overhead.
 double fitness=n->getFitnessScore();const auto m=n->getFinalTransformation();Eigen::Quaternionf q(m.block<3,3>(0,0));
 t.frozen=t.frozen&&target.get()==t.target_address&&target->size()*3==t.baseline.size();
 if(t.frozen)for(size_t i=0;i<target->size();++i)if(std::memcmp(target->points[i].data,t.baseline.data()+3*i,12)!=0){t.frozen=false;break;}
 std::vector<float> source_xyz;for(const auto& p:*n->getInputSource()){source_xyz.push_back(p.x);source_xyz.push_back(p.y);source_xyz.push_back(p.z);}const auto source_sha=sha(source_xyz);
 struct rusage usage;getrusage(RUSAGE_SELF,&usage);
 auto diagnostic_end=Clock::now();std::lock_guard<std::mutex> lock(t.mutex);++t.calls;
 t.ndt<<std::setprecision(17)<<scan_stamp<<','<<std::chrono::duration<double,std::milli>(end-begin).count()<<','<<std::chrono::duration<double,std::milli>(diagnostic_end-end).count()<<','<<n->getFinalNumIteration()<<','<<n->hasConverged()<<','<<fitness<<','<<n->getTransformationProbability()<<','<<n->getInputSource()->size()<<','<<target->size()<<','<<t.frozen<<','<<t.before_sha<<','<<m(0,3)<<','<<m(1,3)<<','<<m(2,3)<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<usage.ru_maxrss<<','<<source_sha;
 for(int r=0;r<4;++r)for(int c=0;c<4;++c)t.ndt<<','<<recorded_guess(r,c);t.ndt<<'\n';t.ndt.flush();
}
extern "C" void callback_trace(void*,const sensor_msgs::PointCloud2ConstPtr&) asm("_ZN16hdl_localization22HdlLocalizationNodelet15points_callbackERKN5boost10shared_ptrIKN11sensor_msgs12PointCloud2_ISaIvEEEEE");
extern "C" void callback_trace(void* self,const sensor_msgs::PointCloud2ConstPtr& cloud){
 static auto real=original<void(*)(void*,const sensor_msgs::PointCloud2ConstPtr&)>("_ZN16hdl_localization22HdlLocalizationNodelet15points_callbackERKN5boost10shared_ptrIKN11sensor_msgs12PointCloud2_ISaIvEEEEE");
 scan_stamp=cloud->header.stamp.toNSec();auto begin=Clock::now();real(self,cloud);auto end=Clock::now();auto& t=trace();
 std::lock_guard<std::mutex> lock(t.mutex);t.cycles<<std::setprecision(17)<<scan_stamp<<','<<std::chrono::duration<double,std::milli>(end-begin).count()<<','<<t.calls<<'\n';t.cycles.flush();
}

// Additional read-only IMU receipt/predict instrumentation. No state writes.
#include <sensor_msgs/Imu.h>
#include <boost/optional.hpp>
#include <ros/ros.h>
#define private public
#include <hdl_localization/pose_estimator.hpp>
#undef private
#include <hdl_localization/pose_system.hpp>
struct ImuTrace {
 std::ofstream received,predicted;std::mutex lock;size_t count=0;
 ImuTrace(){const std::string d=std::getenv("P63_TRACE_DIR");
  received.open(d+"/IMU_RECEIVED.csv");predicted.open(d+"/IMU_PREDICT.csv");
  received<<"receipt_count,stamp_ns,seq,frame,ax,ay,az,gx,gy,gz\n";
  predicted<<"scan_stamp_ns,imu_stamp_ns,prev_stamp_before_ns,init_stamp_before_ns,last_correction_ns,active_predict,dt_s,predict_ms,prev_stamp_after_ns,ax,ay,az,gx,gy,gz,px,py,pz,vx,vy,vz,qx,qy,qz,qw,finite_state\n";
 }
};
ImuTrace& it(){static ImuTrace t;return t;}
extern "C" void imu_receipt(void*,const sensor_msgs::ImuConstPtr&) asm("_ZN16hdl_localization22HdlLocalizationNodelet12imu_callbackERKN5boost10shared_ptrIKN11sensor_msgs4Imu_ISaIvEEEEE");
extern "C" void imu_receipt(void* self,const sensor_msgs::ImuConstPtr& m){
 static auto real=original<void(*)(void*,const sensor_msgs::ImuConstPtr&)>("_ZN16hdl_localization22HdlLocalizationNodelet12imu_callbackERKN5boost10shared_ptrIKN11sensor_msgs4Imu_ISaIvEEEEE");
 real(self,m); // Return confirms original callback has appended this message.
 auto& t=it();std::lock_guard<std::mutex> lock(t.lock);++t.count;
 t.received<<std::setprecision(17)<<t.count<<','<<m->header.stamp.toNSec()<<','<<m->header.seq<<','<<m->header.frame_id<<','<<m->linear_acceleration.x<<','<<m->linear_acceleration.y<<','<<m->linear_acceleration.z<<','<<m->angular_velocity.x<<','<<m->angular_velocity.y<<','<<m->angular_velocity.z<<'\n';t.received.flush();
}
extern "C" void predict_trace(void*,const ros::Time&,const Eigen::Vector3f&,const Eigen::Vector3f&) asm("_ZN16hdl_localization13PoseEstimator7predictERKN3ros4TimeERKN5Eigen6MatrixIfLi3ELi1ELi0ELi3ELi1EEES9_");
extern "C" void predict_trace(void* self,const ros::Time& stamp,const Eigen::Vector3f& acc,const Eigen::Vector3f& gyro){
 static auto real=original<void(*)(void*,const ros::Time&,const Eigen::Vector3f&,const Eigen::Vector3f&)>("_ZN16hdl_localization13PoseEstimator7predictERKN3ros4TimeERKN5Eigen6MatrixIfLi3ELi1ELi0ELi3ELi1EEES9_");
 auto* p=static_cast<hdl_localization::PoseEstimator*>(self);
 const auto prev=p->prev_stamp,init=p->init_stamp,correction=p->last_correction_stamp;
 const auto actual_init=init.isZero()?stamp:init;
 const bool active=!prev.isZero() && prev!=stamp && (stamp-actual_init).toSec()>=p->cool_time_duration;
 const double dt=active?(stamp-prev).toSec():0;
 auto b=Clock::now();real(self,stamp,acc,gyro);auto e=Clock::now();
 const auto& mean=p->ukf->mean;Eigen::Vector3f v=mean.segment<3>(3),pos=mean.segment<3>(0);Eigen::Quaternionf q(mean[6],mean[7],mean[8],mean[9]);q.normalize();bool finite=v.allFinite()&&pos.allFinite()&&q.coeffs().allFinite();
 auto& t=it();std::lock_guard<std::mutex> lock(t.lock);
 t.predicted<<std::setprecision(17)<<scan_stamp<<','<<stamp.toNSec()<<','<<prev.toNSec()<<','<<init.toNSec()<<','<<correction.toNSec()<<','<<active<<','<<dt<<','<<std::chrono::duration<double,std::milli>(e-b).count()<<','<<p->prev_stamp.toNSec();
 for(int i=0;i<3;++i)t.predicted<<','<<acc[i];for(int i=0;i<3;++i)t.predicted<<','<<gyro[i];
 for(int i=0;i<3;++i)t.predicted<<','<<pos[i];for(int i=0;i<3;++i)t.predicted<<','<<v[i];
 t.predicted<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<finite<<'\n';t.predicted.flush();
 if(!finite || stamp.toNSec()>scan_stamp || dt<0){fprintf(stderr,"R9_INVALID_IMU_PREDICT_STATE_OR_TIME\n");std::abort();}
}
