// Terminal native IMU propagation and actual state/covariance export only.
#include <std_srvs/Trigger.h>

struct R6Terminal {
    std::string directory;
    uint64_t target_ns, expected_imu_ns;
    P61Record& recorder;
    bool finalized=false;
    bool finish(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
        if (finalized) {response.success=true;response.message="ALREADY_FROZEN";return true;}
        bool gate=false;ros::param::get("/r6_gate_passed",gate);
        std::lock_guard<std::mutex> lock(mtx_buffer);
        if (!gate || recorder.count!=2984 || !lidar_buffer.empty() || lidar_pushed) {
            response.success=false;response.message="INVALID_BRIDGE_STATE";return true;
        }
        if (imu_buffer.empty() || r6_latest_raw_imu_ns!=expected_imu_ns) {
            response.success=false;response.message="WAIT_IMU_DELIVERY";return true;
        }
        for(const auto& m:imu_buffer) if(m->header.stamp.toNSec()>target_ns) {
            response.success=false;response.message="FUTURE_IMU_REJECTED";return true;
        }
        const double old_time=Measures.lidar_end_time;
        const ros::Time target(target_ns/1000000000ULL,target_ns%1000000000ULL);
        const uint64_t first_ns=imu_buffer.front()->header.stamp.toNSec();
        const uint64_t last_native_ns=imu_buffer.back()->header.stamp.toNSec();
        const size_t count=imu_buffer.size();
        try {p_imu->PropagateOnlyBefore(imu_buffer,target.toSec(),kf);}
        catch(const std::exception& e) {response.success=false;response.message=e.what();return true;}
        imu_buffer.clear();state_point=kf.get_x();
        const auto s=kf.get_x();
        Eigen::Matrix4d mi=Eigen::Matrix4d::Identity(),il=mi;
        mi.block<3,3>(0,0)=s.rot.toRotationMatrix();mi.block<3,1>(0,3)=s.pos;
        il.block<3,3>(0,0)=s.offset_R_L_I.toRotationMatrix();il.block<3,1>(0,3)=s.offset_T_L_I;
        Eigen::Matrix4d ml=mi*il;
        std::ofstream file(directory+"/initial_state.json");
        file<<std::setprecision(17)<<"{\"target_stamp_ns\":"<<target_ns
            <<",\"native_target_sec\":"<<target.toSec()<<",\"propagation_from_sec\":"<<old_time
            <<",\"first_new_imu_ns\":"<<first_ns<<",\"last_imu_ns\":"<<last_native_ns
            <<",\"last_raw_imu_ns\":"<<expected_imu_ns
            <<",\"new_imu_count\":"<<count<<",\"test_lidar_used\":false,\"gt_used\":false";
        const char* names[]={"T_M_I","T_I_L","T_M_L"};int index=0;
        for(const auto& t:{mi,il,ml}) {
            file<<",\""<<names[index++]<<"\":[";
            for(int r=0;r<4;++r)for(int c=0;c<4;++c){if(r||c)file<<',';file<<t(r,c);}file<<']';
        }
        const char* vectors[]={"velocity_M","gyro_bias_I","accel_bias_I","gravity_M"};index=0;
        for(const Eigen::Vector3d v:{Eigen::Vector3d(s.vel),Eigen::Vector3d(s.bg),Eigen::Vector3d(s.ba),Eigen::Vector3d(s.grav)}) {
            file<<",\""<<vectors[index++]<<"\":["<<v[0]<<','<<v[1]<<','<<v[2]<<']';
        }
        file<<"}\n";file.close();
        std::ofstream covariance(directory+"/initial_covariance.csv");
        covariance<<std::setprecision(17);const auto p=kf.get_P();
        for(int r=0;r<p.rows();++r){for(int c=0;c<p.cols();++c){if(c)covariance<<',';covariance<<p(r,c);}covariance<<'\n';}
        covariance.close();
        finalized=true;response.success=true;response.message="CAUSAL_STATE_EXPORTED";return true;
    }
};
